// Copyright 2026 TIER IV, Inc.
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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/detection_area_stop.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_utils/detection_area_utils.hpp"

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
namespace
{
namespace detection_area = autoware::trajectory_modifier::utils::detection_area;

using detection_area::can_clear_stop_state;
using detection_area::feasible_stop_distance_by_max_acceleration;
using detection_area::get_detected_object;
using detection_area::get_obstacle_points;
using detection_area::get_stop_point;
using detection_area::object_label_to_string;

constexpr double ego_nearest_distance{5.0};
constexpr double ego_nearest_yaw_deviation{1.5707963267948966};
constexpr double stopped_velocity_threshold{1e-3};

void append_debug_status(std::string & status, const std::string & message)
{
  if (!status.empty()) status += "; ";
  status += message;
}

std::vector<geometry_msgs::msg::Point> get_object_polygon_points(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto polygon = autoware_utils::to_polygon2d(pose, object.shape);
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(polygon.outer().size());
  for (const auto & point : polygon.outer()) {
    points.push_back(autoware_utils::create_point(point.x(), point.y(), pose.position.z));
  }
  return points;
}
}  // namespace

void DetectionAreaStop::on_initialize(const TrajectoryModifierParams & params)
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "modifier_detection_area_stop");
  debug_viz_pub_ = node_ptr->create_publisher<MarkerArray>("~/detection_area_stop/debug/marker", 1);
  pub_debug_text_ = node_ptr->create_publisher<StringStamped>("~/detection_area_stop/debug/text", 1);
  enabled_ = params.use_detection_area_stop;
  update_params(params.detection_area);
}

void DetectionAreaStop::update_params(const TrajectoryModifierParams & params)
{
  enabled_ = params.use_detection_area_stop;
  update_params(params.detection_area);
}

void DetectionAreaStop::update_params(const TrajectoryModifierParams::DetectionArea & params)
{
  planner_param_.stop_margin = params.stop_margin;
  planner_param_.use_dead_line = params.use_dead_line;
  planner_param_.dead_line_margin = params.dead_line_margin;
  planner_param_.state_clear_time = params.state_clear_time;
  planner_param_.hold_stop_margin_distance = params.hold_stop_margin_distance;
  planner_param_.distance_to_judge_over_stop_line = params.distance_to_judge_over_stop_line;
  planner_param_.suppress_pass_judge_when_stopping = params.suppress_pass_judge_when_stopping;
  planner_param_.enable_detected_obstacle_logging = params.enable_detected_obstacle_logging;
  planner_param_.unstoppable_policy = params.unstoppable_policy;
  planner_param_.max_deceleration = params.max_deceleration;
  planner_param_.delay_response_time = params.delay_response_time;

  planner_param_.target_filtering.pointcloud = params.target_filtering.pointcloud;
  planner_param_.target_filtering.unknown = params.target_filtering.unknown;
  planner_param_.target_filtering.car = params.target_filtering.car;
  planner_param_.target_filtering.truck = params.target_filtering.truck;
  planner_param_.target_filtering.bus = params.target_filtering.bus;
  planner_param_.target_filtering.trailer = params.target_filtering.trailer;
  planner_param_.target_filtering.motorcycle = params.target_filtering.motorcycle;
  planner_param_.target_filtering.bicycle = params.target_filtering.bicycle;
  planner_param_.target_filtering.pedestrian = params.target_filtering.pedestrian;
  planner_param_.target_filtering.animal = params.target_filtering.animal;
  planner_param_.target_filtering.hazard = params.target_filtering.hazard;
  planner_param_.target_filtering.over_drivable = params.target_filtering.over_drivable;
  planner_param_.target_filtering.under_drivable = params.target_filtering.under_drivable;
}

void DetectionAreaStop::begin_cycle(const InputData & input)
{
  first_candidate_in_cycle_ = true;
  cycle_pointcloud_.reset();
  debug_status_.clear();
  last_candidate_modified_ = false;

  if (!enabled_) {
    debug_status_ = "disabled";
    modules_.clear();
    route_lanelet_ids_.clear();
    last_lanelet_map_.reset();
    return;
  }

  if (!input.current_odometry || !input.lanelet_map || !input.route) {
    debug_status_ = "fail-open: missing ";
    if (!input.current_odometry) debug_status_ += "odometry ";
    if (!input.lanelet_map) debug_status_ += "map ";
    if (!input.route) debug_status_ += "route ";
    modules_.clear();
    route_lanelet_ids_.clear();
    last_lanelet_map_.reset();
    return;
  }

  rebuild_modules(input);
  reset_candidate_debug();
  if (modules_.empty()) debug_status_ = "no DetectionArea modules on preferred route";
  update_cycle_observations(input);
}

void DetectionAreaStop::end_cycle() {}

void DetectionAreaStop::reset_candidate_debug()
{
  for (auto & module : modules_) {
    module.stop_pose.reset();
    module.dead_line_pose.reset();
    module.stop_point_arc_length = 0.0;
    module.dead_line_passed = false;
    module.candidate_modified = false;
    module.candidate_policy.clear();
  }
}

std::shared_ptr<const DetectionAreaStop::PointCloud> DetectionAreaStop::make_map_pointcloud(
  const InputData & input) const
{
  if (!input.obstacle_pointcloud || input.obstacle_pointcloud->data.empty()) {
    return nullptr;
  }

  auto pointcloud = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*input.obstacle_pointcloud, *pointcloud);
  if (input.obstacle_pointcloud->header.frame_id == "map") {
    return pointcloud;
  }

  try {
    const auto transform = context_->tf_buffer.lookupTransform(
      "map", input.obstacle_pointcloud->header.frame_id, tf2::TimePointZero);
    const auto isometry = tf2::transformToEigen(transform.transform).cast<float>();
    autoware_utils::transform_pointcloud(*pointcloud, *pointcloud, isometry);
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM DetectionAreaStop] Cannot transform pointcloud to map: %s", error.what());
    return nullptr;
  }
  return pointcloud;
}

void DetectionAreaStop::update_cycle_observations(const InputData & input)
{
  cycle_pointcloud_ = make_map_pointcloud(input);
  if (
    planner_param_.target_filtering.pointcloud && input.obstacle_pointcloud &&
    !cycle_pointcloud_) {
    append_debug_status(debug_status_, "pointcloud unavailable");
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM DetectionAreaStop] Pointcloud detection is unavailable for this cycle");
  }
  if (!cycle_pointcloud_ && !input.predicted_objects) {
    append_debug_status(debug_status_, "fail-open: no pointcloud or predicted objects");
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM DetectionAreaStop] Neither pointcloud nor predicted objects are available");
  }

  const auto now = get_clock()->now();
  const bool is_stopped =
    std::abs(input.current_odometry->twist.twist.linear.x) < stopped_velocity_threshold;

  for (auto & module : modules_) {
    module.has_obstacle = false;
    module.detection_source.clear();
    module.obstacle_points.clear();
    module.object_polygons.clear();

    if (planner_param_.target_filtering.pointcloud && cycle_pointcloud_) {
      module.obstacle_points =
        get_obstacle_points(module.regulatory_element->detectionAreas(), *cycle_pointcloud_);
      if (!module.obstacle_points.empty()) {
        module.has_obstacle = true;
        module.detection_source = "pointcloud";
      }
    }

    if (!module.has_obstacle && input.predicted_objects) {
      const auto detected_object = get_detected_object(
        module.regulatory_element->detectionAreas(), *input.predicted_objects,
        planner_param_.target_filtering);
      if (detected_object) {
        module.has_obstacle = true;
        module.object_polygons.push_back(get_object_polygon_points(*detected_object));
        const auto label = autoware::object_recognition_utils::getHighestProbLabel(
          detected_object->classification);
        module.detection_source = object_label_to_string(label);
      }
    }

    if (module.has_obstacle) {
      module.last_obstacle_found_time = now;
      if (planner_param_.enable_detected_obstacle_logging) {
        RCLCPP_INFO_THROTTLE(
          get_node_ptr()->get_logger(), *get_clock(), 1000,
          "[TM DetectionAreaStop] DetectionArea %ld detected obstacle from %s",
          module.regulatory_element->id(), module.detection_source.c_str());
      }
    }

    if (can_clear_stop_state(
          module.last_obstacle_found_time, now, planner_param_.state_clear_time)) {
      module.last_obstacle_found_time.reset();
      if (!planner_param_.suppress_pass_judge_when_stopping || !is_stopped) {
        if (module.state != State::GO) {
          RCLCPP_INFO(
            get_node_ptr()->get_logger(), "[TM DetectionAreaStop] DetectionArea %ld: STOP -> GO",
            module.regulatory_element->id());
        }
        module.state = State::GO;
      }
    }
  }
}

void DetectionAreaStop::rebuild_modules(const InputData & input)
{
  std::vector<lanelet::Id> route_ids;
  route_ids.reserve(input.route->segments.size());
  for (const auto & segment : input.route->segments) {
    route_ids.push_back(segment.preferred_primitive.id);
  }
  if (route_ids == route_lanelet_ids_ && input.lanelet_map == last_lanelet_map_) return;

  std::unordered_map<std::string, Module> previous_modules;
  for (auto & module : modules_) {
    previous_modules.emplace(
      module_key(module.lane_id, module.regulatory_element->id()), std::move(module));
  }

  modules_.clear();
  route_lanelet_ids_ = route_ids;
  last_lanelet_map_ = input.lanelet_map;
  std::unordered_set<std::string> registered_keys;

  for (const auto & segment : input.route->segments) {
    std::optional<lanelet::ConstLanelet> lane;
    try {
      lane = input.lanelet_map->laneletLayer.get(segment.preferred_primitive.id);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 5000,
        "[TM DetectionAreaStop] Cannot find route lanelet %ld: %s",
        segment.preferred_primitive.id, error.what());
      continue;
    }

    for (const auto & regulatory_element : lane->regulatoryElementsAs<DetectionArea>()) {
      const auto key = module_key(lane->id(), regulatory_element->id());
      if (!registered_keys.insert(key).second) continue;

      Module module;
      module.lane_id = lane->id();
      module.regulatory_element = regulatory_element;
      if (const auto previous = previous_modules.find(key); previous != previous_modules.end()) {
        module.state = previous->second.state;
        module.last_obstacle_found_time = previous->second.last_obstacle_found_time;
        module.forward_offset_to_stop_line = previous->second.forward_offset_to_stop_line;
      }
      modules_.push_back(std::move(module));
    }
  }
}

bool DetectionAreaStop::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, const InputData & input)
{
  if (!enabled_ || traj_points.size() < 2 || modules_.empty() || !input.current_odometry) {
    return false;
  }
  auto copy = traj_points;
  return process_trajectory(copy, input, false, false);
}

bool DetectionAreaStop::modify_trajectory(
  TrajectoryPoints & traj_points, const InputData & input)
{
  reset_candidate_debug();
  last_candidate_modified_ = false;
  if (!enabled_ || traj_points.size() < 2 || modules_.empty() || !input.current_odometry) {
    return false;
  }
  const bool update_state = first_candidate_in_cycle_;
  const bool modified = process_trajectory(traj_points, input, update_state, true);
  last_candidate_modified_ = modified;
  first_candidate_in_cycle_ = false;
  return modified;
}

bool DetectionAreaStop::process_trajectory(
  TrajectoryPoints & traj_points, const InputData & input, const bool update_state,
  const bool apply_modification)
{
  bool modified = false;
  for (auto & module : modules_) {
    modified = process_module(module, traj_points, input, update_state, apply_modification) ||
               modified;
  }
  return modified;
}

bool DetectionAreaStop::process_module(
  Module & module, TrajectoryPoints & traj_points, const InputData & input,
  const bool update_state, const bool apply_modification)
{
  const auto path_result = Trajectory::Builder{}.build(traj_points);
  if (!path_result) return false;
  auto path = *path_result;

  const auto self_s = autoware::experimental::trajectory::find_first_nearest_index(
    path, input.current_odometry->pose.pose, ego_nearest_distance, ego_nearest_yaw_deviation);
  if (!self_s) return false;

  const auto stop_line = module.regulatory_element->stopLine();
  const auto stop_point_s_opt = get_stop_point(
    path, stop_line, planner_param_.stop_margin,
    context_->vehicle_info.max_longitudinal_offset_m - module.forward_offset_to_stop_line);
  if (!stop_point_s_opt) return false;

  const double stop_point_s = *stop_point_s_opt;
  const double distance_to_stop = stop_point_s - *self_s;
  double modified_stop_point_s = stop_point_s;
  module.stop_point_arc_length = stop_point_s;
  module.stop_pose = path.compute(std::clamp(stop_point_s, 0.0, path.length())).point.pose;
  const bool is_stopped =
    std::abs(input.current_odometry->twist.twist.linear.x) < stopped_velocity_threshold;

  if (is_stopped && distance_to_stop < planner_param_.hold_stop_margin_distance) {
    modified_stop_point_s = *self_s;
  }

  std::optional<double> dead_line_s;
  if (planner_param_.use_dead_line) {
    dead_line_s = get_stop_point(
      path, stop_line, -planner_param_.dead_line_margin,
      context_->vehicle_info.max_longitudinal_offset_m);
    if (dead_line_s && *dead_line_s - *self_s < 0.0) {
      module.dead_line_passed = true;
      module.dead_line_pose =
        path.compute(std::clamp(*dead_line_s, 0.0, path.length())).point.pose;
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 1000,
        "[TM DetectionAreaStop] DetectionArea %ld is over the dead line",
        module.regulatory_element->id());
      if (update_state) module.state = State::GO;
      return false;
    }
    if (dead_line_s) {
      module.dead_line_pose =
        path.compute(std::clamp(*dead_line_s, 0.0, path.length())).point.pose;
    }
  }

  if (module.state == State::GO && !module.has_obstacle) return false;

  if (
    module.state != State::STOP &&
    distance_to_stop < -planner_param_.distance_to_judge_over_stop_line) {
    return false;
  }

  const double current_velocity = input.current_odometry->twist.twist.linear.x;
  const double braking_distance =
    std::max(0.0, current_velocity) * planner_param_.delay_response_time +
    current_velocity * current_velocity / (2.0 * std::max(1e-3, planner_param_.max_deceleration));
  const bool has_enough_distance =
    current_velocity < stopped_velocity_threshold || distance_to_stop > braking_distance;

  if (module.state != State::STOP && !has_enough_distance) {
    if (planner_param_.unstoppable_policy == "go") {
      module.candidate_policy = "go";
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 1000,
        "[TM DetectionAreaStop] Insufficient braking distance, policy: go");
      return false;
    }
    if (planner_param_.unstoppable_policy == "stop_after_stopline") {
      module.candidate_policy = "stop_after_stopline";
      const double offset = std::max(
        feasible_stop_distance_by_max_acceleration(
          current_velocity, planner_param_.max_deceleration) - distance_to_stop,
        0.0);
      if (update_state) module.forward_offset_to_stop_line = offset;
      modified_stop_point_s =
        stop_point_s + (update_state ? offset : module.forward_offset_to_stop_line);
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 1000,
        "[TM DetectionAreaStop] Insufficient braking distance, policy: stop_after_stopline");
    }
    if (module.candidate_policy.empty()) module.candidate_policy = "force_stop";
    if (update_state) set_state(module, State::STOP);
    if (!apply_modification) return true;
    const bool modified = insert_stop_velocity(
      path, stop_point_s, modified_stop_point_s, input.current_odometry->pose.pose, module);
    if (modified) traj_points = path.restore();
    return modified;
  }

  if (update_state) set_state(module, State::STOP);
  if (!module.has_obstacle && module.state == State::GO) return false;
  module.candidate_policy = "normal";
  if (!apply_modification) return true;
  const bool modified = insert_stop_velocity(
    path, stop_point_s, modified_stop_point_s, input.current_odometry->pose.pose, module);
  if (modified) traj_points = path.restore();
  return modified;
}

bool DetectionAreaStop::insert_stop_velocity(
  Trajectory & path, const double stop_point_s, const double modified_stop_point_s,
  const geometry_msgs::msg::Pose & self_pose, Module & module)
{
  const double start_s = std::clamp(modified_stop_point_s, 0.0, path.length());
  path.longitudinal_velocity_mps().range(start_s, path.length()).set(0.0);
  const double factor_s = std::clamp(stop_point_s, 0.0, path.length());
  const auto stop_pose = path.compute(factor_s).point.pose;
  planning_factor_interface_->add(
    path.restore(), self_pose, stop_pose, PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true, 0.0, 0.0,
    module.detection_source);
  module.candidate_modified = true;

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[TM DetectionAreaStop] Inserted stop for DetectionArea %ld (%s)",
    module.regulatory_element->id(), module.detection_source.c_str());
  return true;
}

void DetectionAreaStop::publish_debug_data(const std::string & ns) const
{
  const auto now = get_clock()->now();
  const auto green = autoware_utils::create_marker_color(0.0, 1.0, 0.0, 1.0);
  const auto red = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 1.0);
  const auto white = autoware_utils::create_marker_color(1.0, 1.0, 1.0, 1.0);
  const auto orange = autoware_utils::create_marker_color(1.0, 0.5, 0.0, 1.0);
  const auto magenta = autoware_utils::create_marker_color(1.0, 0.0, 1.0, 1.0);
  const auto gray = autoware_utils::create_marker_color(0.6, 0.6, 0.6, 1.0);

  MarkerArray marker_array;
  int marker_id = 0;
  const auto lifetime = rclcpp::Duration::from_seconds(0.2);

  const auto add_line_marker = [&](const std::string & marker_ns,
                                   const std::vector<geometry_msgs::msg::Point> & points,
                                   const std_msgs::msg::ColorRGBA & color, const double width) {
    if (points.size() < 2) return;
    auto marker = autoware_utils::create_default_marker(
      "map", now, marker_ns, marker_id++, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(width, width, width), color);
    marker.lifetime = lifetime;
    marker.points = points;
    marker_array.markers.push_back(marker);
  };

  const auto add_point_marker = [&](const std::string & marker_ns,
                                    const geometry_msgs::msg::Point & point,
                                    const std_msgs::msg::ColorRGBA & color, const double scale) {
    auto marker = autoware_utils::create_default_marker(
      "map", now, marker_ns, marker_id++, Marker::SPHERE,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.lifetime = lifetime;
    marker.pose.position = point;
    marker_array.markers.push_back(marker);
  };

  const auto add_text_marker = [&](const std::string & marker_ns,
                                   const geometry_msgs::msg::Point & point,
                                   const std::string & text,
                                   const std_msgs::msg::ColorRGBA & color) {
    auto marker = autoware_utils::create_default_marker(
      "map", now, marker_ns, marker_id++, Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(0.0, 0.0, 0.8), color);
    marker.lifetime = lifetime;
    marker.pose.position = point;
    marker.text = text;
    marker_array.markers.push_back(marker);
  };

  for (const auto & module : modules_) {
    if (!module.regulatory_element) continue;
    const auto module_ns = ns + "/detection_area/" + module_key(
      module.lane_id, module.regulatory_element->id());
    const auto state_color = module.state == State::STOP ? red : green;

    for (const auto & detection_area : module.regulatory_element->detectionAreas()) {
      const auto polygon = lanelet::utils::to2D(detection_area).basicPolygon();
      std::vector<geometry_msgs::msg::Point> polygon_points;
      polygon_points.reserve(polygon.size() + 1);
      for (const auto & point : polygon) {
        polygon_points.push_back(autoware_utils::create_point(point.x(), point.y(), 0.0));
      }
      if (!polygon_points.empty()) polygon_points.push_back(polygon_points.front());
      add_line_marker(module_ns + "/polygon", polygon_points, state_color, 0.1);

      if (polygon_points.size() >= 2) {
        geometry_msgs::msg::Point centroid{};
        for (const auto & point : polygon_points) {
          centroid.x += point.x;
          centroid.y += point.y;
          centroid.z += point.z;
        }
        const auto divisor = static_cast<double>(polygon_points.size() - 1);
        centroid.x /= divisor;
        centroid.y /= divisor;
        centroid.z /= divisor;

        const auto stop_line = module.regulatory_element->stopLine();
        if (!stop_line.empty()) {
          const auto first = stop_line.front().basicPoint();
          const auto last = stop_line.back().basicPoint();
          const auto stop_line_center = autoware_utils::create_point(
            (first.x() + last.x()) * 0.5, (first.y() + last.y()) * 0.5,
            (first.z() + last.z()) * 0.5);
          add_line_marker(
            module_ns + "/correspondence",
            std::vector<geometry_msgs::msg::Point>{centroid, stop_line_center}, gray, 0.05);
        }
      }
    }

    const auto stop_line = module.regulatory_element->stopLine();
    std::vector<geometry_msgs::msg::Point> stop_line_points;
    for (const auto & point : stop_line) {
      const auto basic_point = point.basicPoint();
      stop_line_points.push_back(
        autoware_utils::create_point(basic_point.x(), basic_point.y(), basic_point.z()));
    }
    add_line_marker(module_ns + "/stop_line", stop_line_points, white, 0.15);

    for (const auto & point : module.obstacle_points) {
      add_point_marker(module_ns + "/obstacle_points", point, magenta, 0.25);
    }
    for (const auto & polygon : module.object_polygons) {
      auto polygon_points = polygon;
      if (!polygon_points.empty()) polygon_points.push_back(polygon_points.front());
      add_line_marker(module_ns + "/object_polygons", polygon_points, magenta, 0.12);
    }

    if (module.dead_line_pose) {
      add_point_marker(
        module_ns + "/dead_line", module.dead_line_pose->position,
        module.dead_line_passed ? red : orange, 0.35);
    }
    if (module.stop_pose) {
      add_point_marker(module_ns + "/stop_point", module.stop_pose->position, red, 0.45);
      if (module.candidate_modified) {
        add_text_marker(
          module_ns + "/stop_text", module.stop_pose->position,
          "DetectionArea STOP: " + module.candidate_policy, red);
      }
    }
  }

  debug_viz_pub_->publish(marker_array);

  std::ostringstream text;
  text << std::fixed << std::setprecision(2) << std::boolalpha;
  text << "DETECTION AREA STOP MODIFIER:\n";
  text << "\tCANDIDATE: " << ns << "\n";
  text << "\tMODIFIED: " << last_candidate_modified_ << "\n";
  text << "\tSTATUS: " << (debug_status_.empty() ? "ready" : debug_status_) << "\n";
  text << "\tMODULES: " << modules_.size() << "\n";
  for (const auto & module : modules_) {
    if (!module.regulatory_element) continue;
    text << "\tDetectionArea " << module.regulatory_element->id() << " lane "
         << module.lane_id << ": state=" << (module.state == State::STOP ? "STOP" : "GO")
         << ", source=" << (module.detection_source.empty() ? "none" : module.detection_source)
         << ", pointcloud_points=" << module.obstacle_points.size()
         << ", object_polygons=" << module.object_polygons.size();
    if (module.stop_pose) {
      text << ", stop_s=" << module.stop_point_arc_length;
    }
    if (planner_param_.use_dead_line) {
      text << ", dead_line=" << (module.dead_line_passed ? "passed" : "clear");
    }
    if (!module.candidate_policy.empty()) {
      text << ", policy=" << module.candidate_policy;
    }
    text << ", candidate_modified=" << module.candidate_modified << "\n";
  }

  StringStamped debug_text;
  debug_text.stamp = now;
  debug_text.data = text.str();
  pub_debug_text_->publish(debug_text);
}

void DetectionAreaStop::set_state(Module & module, const State state)
{
  if (module.state == state) return;
  const auto old_state = module.state == State::GO ? "GO" : "STOP";
  const auto new_state = state == State::GO ? "GO" : "STOP";
  RCLCPP_INFO(
    get_node_ptr()->get_logger(), "[TM DetectionAreaStop] DetectionArea %ld: %s -> %s",
    module.regulatory_element->id(), old_state, new_state);
  module.state = state;
}

std::string DetectionAreaStop::module_key(
  const lanelet::Id lane_id, const lanelet::Id regulatory_element_id)
{
  return std::to_string(lane_id) + ":" + std::to_string(regulatory_element_id);
}
}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::DetectionAreaStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
