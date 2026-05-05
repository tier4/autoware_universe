// Copyright 2021 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "node.hpp"

#include "grid_ground_filter.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::ground_segmentation
{
using autoware::pointcloud_preprocessor::get_param;
using autoware::vehicle_info_utils::VehicleInfoUtils;
using autoware_utils::calc_distance3d;
using autoware_utils::deg2rad;
using autoware_utils::normalize_degree;
using autoware_utils::normalize_radian;
using autoware_utils::ScopedTimeTrack;

namespace
{
constexpr float ground_probability = 1.0f;
const std::vector<std::string> default_segmentation_class_names{
  "drivable_surface",
  "other_flat_surface",
  "sidewalk",
  "manmade",
  "vegetation",
  "car",
  "bus",
  "emergency_vehicle",
  "train",
  "truck",
  "tractor_unit",
  "semi_trailer",
  "construction_vehicle",
  "forklift",
  "kart",
  "motorcycle",
  "bicycle",
  "pedestrian",
  "personal_mobility",
  "animal",
  "pushable_pullable",
  "traffic_cone",
  "debris",
  "stroller",
  "other_stuff",
  "noise+ghost_point"};

size_t findClassIndex(const std::vector<std::string> & class_names, const std::string & class_name)
{
  const auto it = std::find(class_names.begin(), class_names.end(), class_name);
  if (it == class_names.end()) {
    throw std::runtime_error("Class '" + class_name + "' not found in segmentation class_names");
  }
  return static_cast<size_t>(std::distance(class_names.begin(), it));
}

}  // namespace

ScanGroundFilterComponent::ScanGroundFilterComponent(const rclcpp::NodeOptions & options)
: autoware::pointcloud_preprocessor::Filter("ScanGroundFilter", options)
{
  // set initial parameters
  {
    // modes
    elevation_grid_mode_ = declare_parameter<bool>("elevation_grid_mode");

    // common parameters
    radial_divider_angle_rad_ =
      static_cast<float>(deg2rad(declare_parameter<double>("radial_divider_angle_deg")));
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);

    // common thresholds
    global_slope_max_angle_rad_ =
      static_cast<float>(deg2rad(declare_parameter<double>("global_slope_max_angle_deg")));
    local_slope_max_angle_rad_ =
      static_cast<float>(deg2rad(declare_parameter<double>("local_slope_max_angle_deg")));
    global_slope_max_ratio_ = std::tan(global_slope_max_angle_rad_);
    local_slope_max_ratio_ = std::tan(local_slope_max_angle_rad_);
    split_points_distance_tolerance_ =
      static_cast<float>(declare_parameter<double>("split_points_distance_tolerance"));

    // vehicle info
    vehicle_info_ = VehicleInfoUtils(*this).getVehicleInfo();

    // non-grid parameters
    use_virtual_ground_point_ = declare_parameter<bool>("use_virtual_ground_point");
    split_height_distance_ = static_cast<float>(declare_parameter<double>("split_height_distance"));

    // grid mode parameters
    use_recheck_ground_cluster_ = declare_parameter<bool>("use_recheck_ground_cluster");
    recheck_start_distance_ =
      static_cast<float>(declare_parameter<double>("recheck_start_distance"));
    use_lowest_point_ = declare_parameter<bool>("use_lowest_point");
    detection_range_z_max_ = static_cast<float>(declare_parameter<double>("detection_range_z_max"));
    low_priority_region_x_ = static_cast<float>(declare_parameter<double>("low_priority_region_x"));
    center_pcl_shift_ = static_cast<float>(declare_parameter<double>("center_pcl_shift"));
    non_ground_height_threshold_ =
      static_cast<float>(declare_parameter<double>("non_ground_height_threshold"));

    // grid parameters
    grid_size_m_ = static_cast<float>(declare_parameter<double>("grid_size_m"));
    grid_mode_switch_radius_ =
      static_cast<float>(declare_parameter<double>("grid_mode_switch_radius"));
    gnd_grid_buffer_size_ = declare_parameter<int>("gnd_grid_buffer_size");
    virtual_lidar_z_ = vehicle_info_.vehicle_height_m;

    // PTV3-style probability pointcloud parameters
    segmentation_class_names_ = declare_parameter<std::vector<std::string>>(
      "segmentation_class_names", default_segmentation_class_names);
    if (segmentation_class_names_.size() <= 1) {
      throw std::runtime_error("segmentation_class_names must contain more than 1 class");
    }
    ground_probability_class_indices_ = {
      findClassIndex(segmentation_class_names_, "drivable_surface")};

    // initialize grid filter
    {
      GridGroundFilterParameter param;
      param.global_slope_max_angle_rad = global_slope_max_angle_rad_;
      param.local_slope_max_angle_rad = local_slope_max_angle_rad_;
      param.radial_divider_angle_rad = radial_divider_angle_rad_;

      param.use_recheck_ground_cluster = use_recheck_ground_cluster_;
      param.recheck_start_distance = recheck_start_distance_;
      param.use_lowest_point = use_lowest_point_;
      param.detection_range_z_max = detection_range_z_max_;
      param.non_ground_height_threshold = non_ground_height_threshold_;

      param.grid_size_m = grid_size_m_;
      param.grid_mode_switch_radius = grid_mode_switch_radius_;
      param.gnd_grid_buffer_size = gnd_grid_buffer_size_;
      param.virtual_lidar_x = vehicle_info_.wheel_base_m / 2.0f + center_pcl_shift_;
      param.virtual_lidar_y = 0.0f;
      param.virtual_lidar_z = virtual_lidar_z_;

      grid_ground_filter_ptr_ = std::make_unique<GridGroundFilter>(param);
    }
  }

  pub_segmentation_probabilities_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/probs/pointcloud", rclcpp::SensorDataQoS().keep_last(max_queue_size_));

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ScanGroundFilterComponent::onParameter, this, _1));

  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, "scan_ground_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");

    bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
    if (use_time_keeper) {
      detailed_processing_time_publisher_ =
        this->create_publisher<autoware_utils::ProcessingTimeDetail>(
          "~/debug/processing_time_detail_ms", 1);
      auto time_keeper = autoware_utils::TimeKeeper(detailed_processing_time_publisher_);
      time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(time_keeper);

      // set time keeper to grid
      grid_ground_filter_ptr_->setTimeKeeper(time_keeper_);
    }
  }
}

void ScanGroundFilterComponent::convertPointcloud(
  const PointCloud2ConstPtr & in_cloud,
  std::vector<PointCloudVector> & out_radial_ordered_points) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  out_radial_ordered_points.resize(radial_dividers_num_);
  PointData current_point;

  const auto inv_radial_divider_angle_rad = 1.0f / radial_divider_angle_rad_;

  const size_t in_cloud_data_size = in_cloud->data.size();
  const size_t in_cloud_point_step = in_cloud->point_step;

  {  // grouping pointcloud by its azimuth angle
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("azimuth_angle_grouping", *time_keeper_);

    pcl::PointXYZ input_point;
    for (size_t data_index = 0; data_index + in_cloud_point_step <= in_cloud_data_size;
         data_index += in_cloud_point_step) {
      // Get Point
      data_accessor_.getPoint(in_cloud, data_index, input_point);

      // determine the azimuth angle group
      auto radius{static_cast<float>(std::hypot(input_point.x, input_point.y))};
      auto theta{normalize_radian(std::atan2(input_point.x, input_point.y), 0.0)};
      auto radial_div{static_cast<size_t>(std::floor(theta * inv_radial_divider_angle_rad))};

      current_point.radius = radius;
      current_point.point_state = PointLabel::INIT;
      current_point.data_index = data_index;

      // store the point in the corresponding radial division
      out_radial_ordered_points[radial_div].emplace_back(current_point);
    }
  }

  {  // sorting pointcloud by distance, on each azimuth angle group
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_) inner_st_ptr = std::make_unique<ScopedTimeTrack>("sort", *time_keeper_);

    for (size_t i = 0; i < radial_dividers_num_; ++i) {
      std::sort(
        out_radial_ordered_points[i].begin(), out_radial_ordered_points[i].end(),
        [](const PointData & a, const PointData & b) { return a.radius < b.radius; });
    }
  }
}

void ScanGroundFilterComponent::calcVirtualGroundOrigin(pcl::PointXYZ & point) const
{
  point.x = vehicle_info_.wheel_base_m;
  point.y = 0;
  point.z = 0;
}

void ScanGroundFilterComponent::classifyPointCloud(
  const PointCloud2ConstPtr & in_cloud,
  const std::vector<PointCloudVector> & in_radial_ordered_clouds,
  pcl::PointIndices & out_no_ground_indices) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  out_no_ground_indices.indices.clear();

  const pcl::PointXYZ init_ground_point(0, 0, 0);
  pcl::PointXYZ virtual_ground_point(0, 0, 0);
  calcVirtualGroundOrigin(virtual_ground_point);

  // run the classification algorithm for each ray (azimuth division)
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); ++i) {
    float prev_gnd_radius = 0.0f;
    float prev_gnd_slope = 0.0f;
    PointsCentroid ground_cluster, non_ground_cluster;
    PointLabel point_label_curr = PointLabel::INIT;

    pcl::PointXYZ prev_gnd_point(0, 0, 0), point_curr, point_prev;

    // iterate over the points in the ray
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); ++j) {
      float points_distance = 0.0f;
      const float local_slope_max_angle = local_slope_max_angle_rad_;

      // set the previous point
      point_prev = point_curr;
      PointLabel point_label_prev = point_label_curr;

      // set the current point
      const PointData & pd = in_radial_ordered_clouds[i][j];
      point_label_curr = pd.point_state;

      data_accessor_.getPoint(in_cloud, pd.data_index, point_curr);
      if (j == 0) {
        bool is_front_side = (point_curr.x > virtual_ground_point.x);
        if (use_virtual_ground_point_ && is_front_side) {
          prev_gnd_point = virtual_ground_point;
        } else {
          prev_gnd_point = init_ground_point;
        }
        prev_gnd_radius = std::hypot(prev_gnd_point.x, prev_gnd_point.y);
        prev_gnd_slope = 0.0f;
        ground_cluster.initialize();
        non_ground_cluster.initialize();
        points_distance = calc_distance3d(point_curr, prev_gnd_point);
      } else {
        points_distance = calc_distance3d(point_curr, point_prev);
      }

      float radius_distance_from_gnd = pd.radius - prev_gnd_radius;
      float height_from_gnd = point_curr.z - prev_gnd_point.z;
      float height_from_obj = point_curr.z - non_ground_cluster.getAverageHeight();
      bool calculate_slope = true;
      bool is_point_close_to_prev =
        (points_distance <
         (pd.radius * radial_divider_angle_rad_ + split_points_distance_tolerance_));

      float global_slope_ratio = point_curr.z / pd.radius;
      // check points which is far enough from previous point
      if (global_slope_ratio > global_slope_max_ratio_) {
        point_label_curr = PointLabel::NON_GROUND;
        calculate_slope = false;
      } else if (
        (point_label_prev == PointLabel::NON_GROUND) &&
        (std::abs(height_from_obj) >= split_height_distance_)) {
        calculate_slope = true;
      } else if (is_point_close_to_prev && std::abs(height_from_gnd) < split_height_distance_) {
        // close to the previous point, set point follow label
        point_label_curr = PointLabel::POINT_FOLLOW;
        calculate_slope = false;
      }
      if (is_point_close_to_prev) {
        height_from_gnd = point_curr.z - ground_cluster.getAverageHeight();
        radius_distance_from_gnd = pd.radius - ground_cluster.getAverageRadius();
      }
      if (calculate_slope) {
        // far from the previous point
        auto local_slope = std::atan2(height_from_gnd, radius_distance_from_gnd);
        if (local_slope - prev_gnd_slope > local_slope_max_angle) {
          // the point is outside of the local slope threshold
          point_label_curr = PointLabel::NON_GROUND;
        } else {
          point_label_curr = PointLabel::GROUND;
        }
      }

      if (point_label_curr == PointLabel::GROUND) {
        ground_cluster.initialize();
        non_ground_cluster.initialize();
      }
      if (point_label_curr == PointLabel::NON_GROUND) {
        out_no_ground_indices.indices.push_back(pd.data_index);
      } else if (  // NOLINT
        (point_label_prev == PointLabel::NON_GROUND) &&
        (point_label_curr == PointLabel::POINT_FOLLOW)) {
        point_label_curr = PointLabel::NON_GROUND;
        out_no_ground_indices.indices.push_back(pd.data_index);
      } else if (  // NOLINT
        (point_label_prev == PointLabel::GROUND) &&
        (point_label_curr == PointLabel::POINT_FOLLOW)) {
        point_label_curr = PointLabel::GROUND;
      } else {
      }

      // update the ground state
      if (point_label_curr == PointLabel::GROUND) {
        prev_gnd_radius = pd.radius;
        prev_gnd_point = pcl::PointXYZ(point_curr.x, point_curr.y, point_curr.z);
        ground_cluster.addPoint(pd.radius, point_curr.z);
        prev_gnd_slope = ground_cluster.getAverageSlope();
      }
      // update the non ground state
      if (point_label_curr == PointLabel::NON_GROUND) {
        non_ground_cluster.addPoint(pd.radius, point_curr.z);
      }
    }
  }
}

void ScanGroundFilterComponent::extractObjectPoints(
  const PointCloud2ConstPtr & in_cloud_ptr, const pcl::PointIndices & in_indices,
  PointCloud2 & out_object_cloud) const
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  size_t output_data_size = 0;

  for (const auto & idx : in_indices.indices) {
    std::memcpy(
      &out_object_cloud.data[output_data_size], &in_cloud_ptr->data[idx],
      in_cloud_ptr->point_step * sizeof(uint8_t));
    output_data_size += in_cloud_ptr->point_step;
  }
}

std::vector<sensor_msgs::msg::PointField> ScanGroundFilterComponent::createProbabilityPointFields()
  const
{
  std::vector<sensor_msgs::msg::PointField> fields;
  fields.reserve(3 + segmentation_class_names_.size());

  auto make_point_field = [](const std::string & name, const uint32_t offset) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    return field;
  };

  fields.push_back(make_point_field("x", 0));
  fields.push_back(make_point_field("y", 4));
  fields.push_back(make_point_field("z", 8));

  for (size_t i = 0; i < segmentation_class_names_.size(); ++i) {
    fields.push_back(
      make_point_field(segmentation_class_names_[i], static_cast<uint32_t>(12 + i * 4)));
  }

  return fields;
}

sensor_msgs::msg::PointCloud2 ScanGroundFilterComponent::createProbabilityPointCloud(
  const PointCloud2ConstPtr & input, const std::unordered_set<size_t> & no_ground_indices) const
{
  PointCloud2 output;
  const size_t input_point_count = input->width * input->height;
  output.header = input->header;
  output.height = 1;
  output.width = input_point_count;
  output.fields = createProbabilityPointFields();
  output.is_bigendian = input->is_bigendian;
  output.is_dense = input->is_dense;
  output.point_step = static_cast<uint32_t>((3 + segmentation_class_names_.size()) * sizeof(float));
  output.row_step = output.width * output.point_step;
  output.data.resize(output.row_step);

  pcl::PointXYZ input_point;

  size_t point_index = 0;
  for (size_t input_data_index = 0; point_index < input_point_count &&
                                    input_data_index + input->point_step <= input->data.size();
       input_data_index += input->point_step, ++point_index) {
    data_accessor_.getPoint(input, input_data_index, input_point);

    const size_t output_data_index = point_index * output.point_step;
    std::memcpy(&output.data[output_data_index], &input_point.x, sizeof(float));
    std::memcpy(&output.data[output_data_index + sizeof(float)], &input_point.y, sizeof(float));
    std::memcpy(&output.data[output_data_index + 2 * sizeof(float)], &input_point.z, sizeof(float));

    const bool is_no_ground = no_ground_indices.find(input_data_index) != no_ground_indices.end() ||
                              input_point.z > detection_range_z_max_;
    for (size_t class_index = 0; class_index < segmentation_class_names_.size(); ++class_index) {
      float probability;
      if (is_no_ground) {
        probability = 0.0f;
      } else {
        const bool is_ground_class = std::find(
          ground_probability_class_indices_.begin(), ground_probability_class_indices_.end(),
          class_index) != ground_probability_class_indices_.end();
        probability = is_ground_class ? ground_probability : 0.0f;
      }
      std::memcpy(
        &output.data[output_data_index + (3 + class_index) * sizeof(float)], &probability,
        sizeof(float));
    }
  }

  return output;
}

void ScanGroundFilterComponent::publishGroundSegmentationProbabilities(
  const PointCloud2ConstPtr & input, const pcl::PointIndices & no_ground_indices)
{
  if (!pub_segmentation_probabilities_) {
    return;
  }

  const auto subscription_count =
    pub_segmentation_probabilities_->get_subscription_count() +
    pub_segmentation_probabilities_->get_intra_process_subscription_count();
  if (subscription_count == 0) {
    return;
  }

  std::unordered_set<size_t> no_ground_index_set;
  no_ground_index_set.reserve(no_ground_indices.indices.size());
  for (const auto index : no_ground_indices.indices) {
    no_ground_index_set.insert(static_cast<size_t>(index));
  }

  auto output =
    std::make_unique<PointCloud2>(createProbabilityPointCloud(input, no_ground_index_set));
  if (!convert_output_costly(output)) {
    return;
  }
  pub_segmentation_probabilities_->publish(std::move(output));
}

void ScanGroundFilterComponent::faster_filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output,
  [[maybe_unused]] const autoware::pointcloud_preprocessor::TransformInfo & transform_info)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::scoped_lock lock(mutex_);
  if (stop_watch_ptr_) stop_watch_ptr_->toc("processing_time", true);

  if (!data_accessor_.isInitialized()) {
    data_accessor_.setField(input);
    grid_ground_filter_ptr_->setDataAccessor(input);
  }

  pcl::PointIndices no_ground_indices;

  if (elevation_grid_mode_) {
    grid_ground_filter_ptr_->process(input, no_ground_indices);
  } else {
    std::vector<PointCloudVector> radial_ordered_points;
    convertPointcloud(input, radial_ordered_points);
    classifyPointCloud(input, radial_ordered_points, no_ground_indices);
  }
  output.row_step = no_ground_indices.indices.size() * input->point_step;
  output.data.resize(output.row_step);
  output.width = no_ground_indices.indices.size();
  output.fields = input->fields;
  output.is_dense = true;
  output.height = input->height;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.header = input->header;

  extractObjectPoints(input, no_ground_indices, output);
  publishGroundSegmentationProbabilities(input, no_ground_indices);
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

// TODO(taisa1): Temporary Implementation: Delete this function definition when all the filter
// nodes conform to new API.
void ScanGroundFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  (void)input;
  (void)indices;
  (void)output;
}

rcl_interfaces::msg::SetParametersResult ScanGroundFilterComponent::onParameter(
  const std::vector<rclcpp::Parameter> & param)
{
  if (get_param(param, "grid_size_m", grid_size_m_)) {
    // grid_ptr_->initialize(grid_size_m_, radial_divider_angle_rad_, grid_mode_switch_radius_);
  }
  if (get_param(param, "grid_mode_switch_radius", grid_mode_switch_radius_)) {
    // grid_ptr_->initialize(grid_size_m_, radial_divider_angle_rad_, grid_mode_switch_radius_);
  }
  double global_slope_max_angle_deg{get_parameter("global_slope_max_angle_deg").as_double()};
  if (get_param(param, "global_slope_max_angle_deg", global_slope_max_angle_deg)) {
    global_slope_max_angle_rad_ = deg2rad(global_slope_max_angle_deg);
    global_slope_max_ratio_ = std::tan(global_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      get_logger(), "Setting global_slope_max_angle_rad to: %f.", global_slope_max_angle_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting global_slope_max_ratio to: %f.", global_slope_max_ratio_);
  }
  double local_slope_max_angle_deg{get_parameter("local_slope_max_angle_deg").as_double()};
  if (get_param(param, "local_slope_max_angle_deg", local_slope_max_angle_deg)) {
    local_slope_max_angle_rad_ = deg2rad(local_slope_max_angle_deg);
    local_slope_max_ratio_ = std::tan(local_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      get_logger(), "Setting local_slope_max_angle_rad to: %f.", local_slope_max_angle_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting local_slope_max_ratio to: %f.", local_slope_max_ratio_);
  }
  double radial_divider_angle_deg{get_parameter("radial_divider_angle_deg").as_double()};
  if (get_param(param, "radial_divider_angle_deg", radial_divider_angle_deg)) {
    radial_divider_angle_rad_ = deg2rad(radial_divider_angle_deg);
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);
    // grid_ptr_->initialize(grid_size_m_, radial_divider_angle_rad_, grid_mode_switch_radius_);
    RCLCPP_DEBUG(
      get_logger(), "Setting radial_divider_angle_rad to: %f.", radial_divider_angle_rad_);
    RCLCPP_DEBUG(get_logger(), "Setting radial_dividers_num to: %zu.", radial_dividers_num_);
  }
  if (get_param(param, "split_points_distance_tolerance", split_points_distance_tolerance_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting split_points_distance_tolerance to: %f.",
      split_points_distance_tolerance_);
  }
  if (get_param(param, "split_height_distance", split_height_distance_)) {
    RCLCPP_DEBUG(get_logger(), "Setting split_height_distance to: %f.", split_height_distance_);
  }
  if (get_param(param, "use_virtual_ground_point", use_virtual_ground_point_)) {
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Setting use_virtual_ground_point to: " << std::boolalpha << use_virtual_ground_point_);
  }
  if (get_param(param, "use_recheck_ground_cluster", use_recheck_ground_cluster_)) {
    RCLCPP_DEBUG_STREAM(
      get_logger(),
      "Setting use_recheck_ground_cluster to: " << std::boolalpha << use_recheck_ground_cluster_);
  }
  if (get_param(param, "recheck_start_distance", recheck_start_distance_)) {
    RCLCPP_DEBUG(get_logger(), "Setting recheck_start_distance to: %f.", recheck_start_distance_);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace autoware::ground_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ground_segmentation::ScanGroundFilterComponent)
