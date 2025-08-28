// Copyright 2025 TIER IV, Inc.
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

#include "autoware/cuda_pointcloud_preprocessor/cuda_outlier_filter/cuda_polar_voxel_outlier_filter_node.hpp"

#include "autoware/pointcloud_preprocessor/filter.hpp"  // for get_param
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"  // for autoware::pointcloud_preprocessor::utils

namespace autoware::cuda_pointcloud_preprocessor
{
CudaPolarVoxelOutlierFilterNode::CudaPolarVoxelOutlierFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_polar_voxexl_outlier_filter", node_options), updater_(this)
{
  // set initial parameters
  {
    filter_params_.radial_resolution_m = declare_parameter<double>("radial_resolution_m");
    filter_params_.azimuth_resolution_rad = declare_parameter<double>("azimuth_resolution_rad");
    filter_params_.elevation_resolution_rad = declare_parameter<double>("elevation_resolution_rad");
    filter_params_.voxel_points_threshold = declare_parameter<int>("voxel_points_threshold");
    filter_params_.min_radius_m = declare_parameter<double>("min_radius_m");
    filter_params_.max_radius_m = declare_parameter<double>("max_radius_m");
    filter_params_.visibility_estimation_max_range_m =
      declare_parameter<double>("visibility_estimation_max_range_m", 20.0);
    filter_params_.use_return_type_classification =
      declare_parameter<bool>("use_return_type_classification");
    filter_params_.filter_secondary_returns = declare_parameter<bool>("filter_secondary_returns");
    filter_params_.secondary_noise_threshold = declare_parameter<int>("secondary_noise_threshold");
    filter_params_.intensity_threshold = declare_parameter<uint8_t>("intensity_threshold", 1);
    filter_params_.visibility_error_threshold =
      declare_parameter<double>("visibility_error_threshold", 0.5);
    filter_params_.visibility_warn_threshold =
      declare_parameter<double>("visibility_warn_threshold", 0.7);
    filter_params_.filter_ratio_error_threshold =
      declare_parameter<double>("filter_ratio_error_threshold", 0.5);
    filter_params_.filter_ratio_warn_threshold =
      declare_parameter<double>("filter_ratio_warn_threshold", 0.7);
    filter_params_.publish_noise_cloud = declare_parameter<bool>("publish_noise_cloud", false);
    filter_params_.visibility_estimation_max_secondary_voxel_count = static_cast<int>(
      declare_parameter<int64_t>("visibility_estimation_max_secondary_voxel_count", 0));
    filter_params_.visualization_estimation_only =
      declare_parameter<bool>("visualization_estimation_only", false);

    // rclcpp always returns integer array as std::vector<int64_t>
    auto primary_return_types_param =
      declare_parameter<std::vector<int64_t>>("primary_return_types");
    primary_return_types_.clear();
    primary_return_types_.reserve(primary_return_types_param.size());
    for (const auto & val : primary_return_types_param) {
      primary_return_types_.push_back(static_cast<int>(val));
    }
  }

  cuda_polar_voxel_outlier_filter_ = std::make_unique<CudaPolarVoxelOutlierFilter>();
  cuda_polar_voxel_outlier_filter_->set_primary_return_types(primary_return_types_);

  std::string diagnostics_hardware_id =
    declare_parameter<std::string>("hardware_id", "cuda_polar_voxel_outlier_filter");

  // Initialize diagnostics
  updater_.setHardwareID(diagnostics_hardware_id);
  updater_.add(
    std::string(this->get_namespace()) + ": visibility_validation", this,
    &CudaPolarVoxelOutlierFilterNode::on_visibility_check);
  updater_.add(
    std::string(this->get_namespace()) + ": filter_ratio_validation", this,
    &CudaPolarVoxelOutlierFilterNode::on_filter_ratio_check);
  updater_.setPeriod(0.1);

  // Create visibility publisher
  visibility_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "~/debug/visibility", rclcpp::SensorDataQoS());

  // Create ratio publisher
  ratio_pub_ = create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "~/debug/filter_ratio", rclcpp::SensorDataQoS());

  sub_ =
    std::make_shared<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(
        &CudaPolarVoxelOutlierFilterNode::pointcloud_callback, this, std::placeholders::_1));

  filtered_cloud_pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");

  // Create noise cloud publisher if enabled
  if (filter_params_.publish_noise_cloud) {
    noise_cloud_pub_ =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, "~/debug/pointcloud_noise");
    RCLCPP_INFO(get_logger(), "Noise cloud publishing enabled");
  } else {
    RCLCPP_INFO(get_logger(), "Noise cloud publishing disabled for performance optimization");
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) { return param_callback(p); });

  RCLCPP_INFO(
    get_logger(),
    "Polar Voxel Outlier Filter initialized - supports PointXYZIRC and PointXYZIRCAEDT "
    "%s",
    filter_params_.visualization_estimation_only
      ? " (visualization estimation only - no point cloud output)"
      : "");
}

void CudaPolarVoxelOutlierFilterNode::pointcloud_callback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg)
{
  auto start = std::chrono::high_resolution_clock::now();
  // Take mutex so that node configuration will not be
  // ovewritten during one frame processing
  std::scoped_lock lock(param_mutex_);

  validate_filter_inputs(msg);

  // Check if the input point cloud has PointXYZIRCAEDT layout (with pre-computed polar coordinates)
  bool has_polar_coords =
    autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzircaedt(*msg);
  bool has_return_type =
    autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzirc(*msg);

  std::unique_ptr<cuda_blackboard::CudaPointCloud2> filtered_cloud;
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> noise_cloud;
  CudaPolarVoxelOutlierFilter::FilterReturn filter_return{};
  if (has_polar_coords) {
    RCLCPP_DEBUG_ONCE(
      get_logger(), "Processing PointXYZIRCAEDT format with pre-computed polar coordinates");
    filter_return = cuda_polar_voxel_outlier_filter_->filter(
      msg, filter_params_, CudaPolarVoxelOutlierFilter::PolarDataType::PreComputed);
  } else if (has_return_type) {
    RCLCPP_DEBUG_ONCE(
      get_logger(), "Processing PointXYZIRC format, computing azimuth and elevation");
    filter_return = cuda_polar_voxel_outlier_filter_->filter(
      msg, filter_params_, CudaPolarVoxelOutlierFilter::PolarDataType::DeriveFromCartesian);
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "PointXYZ format has not been supported by "
      "autoware_cuda_pointcloud_preprocessor::cuda_polar_voxel_outlier_filter yet.");
  }

  filtered_cloud = std::move(filter_return.filtered_cloud);
  noise_cloud = std::move(filter_return.noise_cloud);
  filter_ratio_ = filter_return.filter_ratio;
  visibility_ = filter_return.visibility;

  if (filtered_cloud.get() == nullptr) {
    return;
  }

  // Publish results (skip if visualization estimation only)
  if (!filter_params_.visualization_estimation_only) {
    filtered_cloud_pub_->publish(std::move(filtered_cloud));
    if (filter_params_.publish_noise_cloud && noise_cloud_pub_) {
      noise_cloud_pub_->publish(std::move(noise_cloud));
    }
  } else {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Visualization estimation only mode - no point cloud output generated");
  }

  if (ratio_pub_) {
    autoware_internal_debug_msgs::msg::Float32Stamped ratio_msg;
    ratio_msg.data = static_cast<float>(filter_ratio_.value_or(0.0));
    ratio_msg.stamp = this->now();
    ratio_pub_->publish(ratio_msg);
  }

  if (visibility_pub_ && visibility_.has_value()) {
    autoware_internal_debug_msgs::msg::Float32Stamped visibility_msg;
    visibility_msg.data = static_cast<float>(visibility_.value());
    visibility_msg.stamp = this->now();
    visibility_pub_->publish(visibility_msg);
  }
}

namespace
{
bool is_in_vector(const std::vector<std::string> & vec, const std::string & value)
{
  return std::find(vec.begin(), vec.end(), value) != vec.end();
}
}  // namespace

void CudaPolarVoxelOutlierFilterNode::update_parameter(const rclcpp::Parameter & param)
{
  const auto & name = param.get_name();

  if (name == "radial_resolution_m") {
    filter_params_.radial_resolution_m = param.as_double();
  } else if (name == "azimuth_resolution_rad") {
    filter_params_.azimuth_resolution_rad = param.as_double();
  } else if (name == "elevation_resolution_rad") {
    filter_params_.elevation_resolution_rad = param.as_double();
  } else if (name == "voxel_points_threshold") {
    filter_params_.voxel_points_threshold = static_cast<int>(param.as_int());
  } else if (name == "secondary_noise_threshold") {
    filter_params_.secondary_noise_threshold = static_cast<int>(param.as_int());
  } else if (name == "intensity_threshold") {
    filter_params_.intensity_threshold = static_cast<int>(param.as_int());
  } else if (name == "visibility_estimation_max_secondary_voxel_count") {
    filter_params_.visibility_estimation_max_secondary_voxel_count =
      static_cast<int>(param.as_int());
  } else if (name == "visualization_estimation_only") {
    filter_params_.visualization_estimation_only = param.as_bool();
  } else if (name == "min_radius_m") {
    filter_params_.min_radius_m = param.as_double();
  } else if (name == "max_radius_m") {
    filter_params_.max_radius_m = param.as_double();
  } else if (name == "visibility_estimation_max_range_m") {
    filter_params_.visibility_estimation_max_range_m = param.as_double();
  } else if (name == "visibility_error_threshold") {
    filter_params_.visibility_error_threshold = param.as_double();
  } else if (name == "visibility_warn_threshold") {
    filter_params_.visibility_warn_threshold = param.as_double();
  } else if (name == "filter_ratio_error_threshold") {
    filter_params_.filter_ratio_error_threshold = param.as_double();
  } else if (name == "filter_ratio_warn_threshold") {
    filter_params_.filter_ratio_warn_threshold = param.as_double();
  } else if (name == "use_return_type_classification") {
    filter_params_.use_return_type_classification = param.as_bool();
  } else if (name == "filter_secondary_returns") {
    filter_params_.filter_secondary_returns = param.as_bool();
  } else if (name == "primary_return_types") {
    auto values = param.as_integer_array();
    primary_return_types_.clear();
    for (const auto & val : values) {
      primary_return_types_.push_back(static_cast<int>(val));
    }
    cuda_polar_voxel_outlier_filter_->set_primary_return_types(primary_return_types_);
  } else if (name == "publish_noise_cloud") {
    bool new_value = param.as_bool();
    if (new_value != filter_params_.publish_noise_cloud) {
      filter_params_.publish_noise_cloud = new_value;
      // Recreate publisher if needed
      if (filter_params_.publish_noise_cloud && !noise_cloud_pub_) {
        noise_cloud_pub_ = std::make_unique<
          cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
          *this, "~/debug/pointcloud_noise");
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult CudaPolarVoxelOutlierFilterNode::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(param_mutex_);

  // Parameters that can be updated at runtime
  static const std::vector<std::string> updatable_params = {
    "radial_resolution_m",
    "azimuth_resolution_rad",
    "elevation_resolution_rad",
    "voxel_points_threshold",
    "min_radius_m",
    "max_radius_m",
    "visibility_estimation_max_range_m",
    "use_return_type_classification",
    "filter_secondary_returns",
    "secondary_noise_threshold",
    "intensity_threshold",
    "visibility_estimation_max_secondary_voxel_count",
    "visualization_estimation_only",
    "primary_return_types",
    "publish_noise_cloud",
    "visibility_error_threshold",
    "visibility_warn_threshold",
    "filter_ratio_error_threshold",
    "filter_ratio_warn_threshold"};

  auto result = rcl_interfaces::msg::SetParametersResult{};
  result.successful = true;

  for (const auto & param : p) {
    if (!is_in_vector(updatable_params, param.get_name())) {
      result.successful = false;
      result.reason = "Parameter " + param.get_name() + " is not updatable";
      return result;
    }

    try {
      update_parameter(param);

      RCLCPP_DEBUG(get_logger(), "Updated parameter: %s", param.get_name().c_str());
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = "Failed to update parameter " + param.get_name() + ": " + e.what();
      return result;
    }
  }

  return result;
}

void CudaPolarVoxelOutlierFilterNode::on_visibility_check(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Take mutex so that node configuration will not be
  // ovewritten during one frame processing
  std::scoped_lock lock(param_mutex_);

  if (!visibility_.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No visibility data available");
    return;
  }

  double visibility_value = visibility_.value();

  if (visibility_value < filter_params_.visibility_error_threshold) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Low visibility detected - potential adverse weather conditions");
  } else if (visibility_value < filter_params_.visibility_warn_threshold) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Reduced visibility detected - monitor environmental conditions");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Visibility within normal range");
  }

  stat.add("Visibility", visibility_value);
  stat.add("Error Threshold", filter_params_.visibility_error_threshold);
  stat.add("Warning Threshold", filter_params_.visibility_warn_threshold);
  stat.add("Estimation Range (m)", filter_params_.visibility_estimation_max_range_m);
  stat.add("Max Secondary Voxels", filter_params_.visibility_estimation_max_secondary_voxel_count);
}

void CudaPolarVoxelOutlierFilterNode::on_filter_ratio_check(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Take mutex so that node configuration will not be
  // ovewritten during one frame processing
  std::scoped_lock lock(param_mutex_);

  if (!filter_ratio_.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No filter ratio data available");
    return;
  }

  double ratio_value = filter_ratio_.value();

  if (ratio_value < filter_params_.filter_ratio_error_threshold) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Very low filter ratio - excessive noise or sensor malfunction");
  } else if (ratio_value < filter_params_.filter_ratio_warn_threshold) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Low filter ratio - increased noise levels detected");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Filter ratio within normal range");
  }

  stat.add("Filter Ratio", ratio_value);
  stat.add("Error Threshold", filter_params_.filter_ratio_error_threshold);
  stat.add("Warning Threshold", filter_params_.filter_ratio_warn_threshold);
  stat.add("Filtering Mode", filter_params_.use_return_type_classification ? "Advanced" : "Simple");
  stat.add("Visualization Only", filter_params_.visualization_estimation_only ? "Yes" : "No");
}

void CudaPolarVoxelOutlierFilterNode::validate_filter_inputs(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud)
{
  if (!input_cloud) {
    RCLCPP_ERROR(get_logger(), "Input point cloud is null");
    throw std::invalid_argument("Input point cloud is null");
  }

  bool has_return_type =
    autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzirc(
      *input_cloud);
  if (filter_params_.use_return_type_classification && !has_return_type) {
    RCLCPP_ERROR(
      get_logger(),
      "Advanced mode (use_return_type_classification=true) requires 'return_type' field. "
      "Set use_return_type_classification=false for simple mode or ensure input has return_type "
      "field.");
    throw std::invalid_argument("Advanced mode requires return_type field");
  }
}

}  // namespace autoware::cuda_pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaPolarVoxelOutlierFilterNode)
