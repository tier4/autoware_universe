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
#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
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

double trajectory_length(const TrajectoryPoints & trajectory)
{
  if (trajectory.size() < 2) return 0.0;
  return motion_utils::calcSignedArcLength(trajectory, 0, trajectory.size() - 1);
}

std::vector<lanelet::Id> collect_route_lanelet_ids(
  const autoware_planning_msgs::msg::LaneletRoute & route)
{
  std::vector<lanelet::Id> ids;
  for (const auto & segment : route.segments) {
    ids.push_back(segment.preferred_primitive.id);
    for (const auto & primitive : segment.primitives) {
      ids.push_back(primitive.id);
    }
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  return ids;
}
}  // namespace

void DetectionAreaStop::on_initialize(const TrajectoryModifierParams & params)
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "modifier_detection_area_stop");
  debug_viz_pub_ = node_ptr->create_publisher<MarkerArray>("~/detection_area_stop/debug/marker", 1);
  pub_debug_text_ =
    node_ptr->create_publisher<StringStamped>("~/detection_area_stop/debug/text", 1);
  enabled_ = params.use_detection_area_stop;
  params_ = params.detection_area_stop;
  stopping_params_ = params.stopping_constraints;
  trajectory_time_step_ = params.trajectory_time_step;
}

void DetectionAreaStop::update_params(const TrajectoryModifierParams & params)
{
  enabled_ = params.use_detection_area_stop;
  params_ = params.detection_area_stop;
  stopping_params_ = params.stopping_constraints;
  trajectory_time_step_ = params.trajectory_time_step;
}

bool DetectionAreaStop::check_inputs(const TrajectoryModifierData & input) const
{
  return input.current_odometry && input.lanelet_map && input.route;
}

ProcessingResult DetectionAreaStop::process(TrajectoryPoints & points, TrajectoryModifierData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("DetectionAreaStop::process", *get_time_keeper());
  // The framework processes candidate indices in order, starting at zero for each callback.
  // Refresh physical observations once, then reuse them without candidate-dependent updates.
  if (input.candidate_index == 0U || !cycle_initialized_) {
    prepare_cycle(input);
  }
  const auto modified = modify_trajectory(points, input);
  return modified ? ProcessingResult::Modified : ProcessingResult::Unchanged;
}

void DetectionAreaStop::prepare_cycle(const TrajectoryModifierData & input)
{
  cycle_initialized_ = true;
  cycle_time_ = get_clock()->now();
  cycle_odometry_ = input.current_odometry;
  cycle_acceleration_ =
    input.current_acceleration ? input.current_acceleration->accel.accel.linear.x : 0.0;
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

  if (!check_inputs(input)) {
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
  if (modules_.empty()) debug_status_ = "no DetectionArea modules on route";
  update_cycle_observations(input);
  update_physical_stop_state(input);
}

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
  const TrajectoryModifierData & input) const
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

void DetectionAreaStop::update_cycle_observations(const TrajectoryModifierData & input)
{
  cycle_pointcloud_ = make_map_pointcloud(input);
  if (params_.target_filtering.pointcloud && input.obstacle_pointcloud && !cycle_pointcloud_) {
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

  const auto now = cycle_time_;
  for (auto & module : modules_) {
    module.has_obstacle = false;
    module.detection_source.clear();
    module.obstacle_points.clear();
    module.object_polygons.clear();

    if (params_.target_filtering.pointcloud && cycle_pointcloud_) {
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
        params_.target_filtering);
      if (detected_object) {
        module.has_obstacle = true;
        module.object_polygons.push_back(get_object_polygon_points(*detected_object));
        const auto label =
          autoware::object_recognition_utils::getHighestProbLabel(detected_object->classification);
        module.detection_source = object_label_to_string(label);
      }
    }

    if (module.has_obstacle) {
      module.last_obstacle_found_time = now;
      if (params_.enable_detected_obstacle_logging) {
        RCLCPP_INFO_THROTTLE(
          get_node_ptr()->get_logger(), *get_clock(), 1000,
          "[TM DetectionAreaStop] DetectionArea %ld detected obstacle from %s",
          module.regulatory_element->id(), module.detection_source.c_str());
      }
    }

    if (can_clear_stop_state(module.last_obstacle_found_time, now, params_.state_clear_time)) {
      module.last_obstacle_found_time.reset();
    }
  }
}

void DetectionAreaStop::update_physical_stop_state(const TrajectoryModifierData & input)
{
  const auto & ego = cycle_odometry_->pose.pose;
  const bool stopped = !utils::is_ego_vehicle_moving(
    cycle_odometry_->twist.twist, stopping_params_.ego_stopped_vel_th);
  for (auto & module : modules_) {
    const bool was_force_stop_required = module.force_stop_required;
    module.force_stop_required = false;
    const auto lane = input.lanelet_map->laneletLayer.get(module.lane_id);
    const auto point = lanelet::BasicPoint2d{ego.position.x, ego.position.y};
    module.physical_stop_distance.reset();
    module.physical_deadline_passed = false;
    if (!lanelet::geometry::inside(lane, point)) {
      set_state(module, State::GO);
      continue;
    }
    TrajectoryPoints centerline;
    const auto lane_centerline = lane.centerline();
    for (size_t i = 0; i < lane_centerline.size(); ++i) {
      const auto & p = lane_centerline[i];
      TrajectoryPoint tp;
      tp.pose.position = autoware_utils::create_point(p.x(), p.y(), p.z());
      if (lane_centerline.size() > 1) {
        const auto & a = lane_centerline[i == 0 ? 0 : i - 1];
        const auto & b = lane_centerline[i == 0 ? 1 : i];
        const auto yaw = std::atan2(b.y() - a.y(), b.x() - a.x());
        tp.pose.orientation.w = std::cos(yaw * 0.5);
        tp.pose.orientation.z = std::sin(yaw * 0.5);
      }
      centerline.push_back(tp);
    }
    const auto path = Trajectory::Builder{}.build(centerline);
    if (!path) {
      set_state(module, State::GO);
      continue;
    }
    // Map centerlines can contain only two far-apart points. Project continuously
    // before applying association limits, rather than requiring a nearby base point.
    const double self_s =
      autoware::experimental::trajectory::find_nearest_index(*path, ego.position);
    const auto projected_pose = path->compute(self_s).pose;
    if (
      autoware_utils::calc_distance2d(projected_pose, ego) > ego_nearest_distance ||
      std::abs(autoware_utils_geometry::calc_yaw_deviation(projected_pose, ego)) >
        ego_nearest_yaw_deviation) {
      set_state(module, State::GO);
      continue;
    }
    const auto stop_s = get_stop_point(
      *path, module.regulatory_element->stopLine(), params_.stop_margin,
      context_->vehicle_info.max_longitudinal_offset_m);
    if (!stop_s) {
      set_state(module, State::GO);
      continue;
    }
    module.physical_stop_distance = *stop_s - self_s;
    if (params_.use_dead_line) {
      const auto deadline = get_stop_point(
        *path, module.regulatory_element->stopLine(), -params_.dead_line_margin,
        context_->vehicle_info.max_longitudinal_offset_m);
      if (deadline && *deadline < self_s) {
        module.physical_deadline_passed = true;
        set_state(module, State::GO);
        continue;
      }
    }
    const auto distance = *stop_s - self_s;
    const bool obstacle_active =
      !can_clear_stop_state(module.last_obstacle_found_time, cycle_time_, params_.state_clear_time);
    const bool keep_hold = module.state == State::STOP && params_.suppress_pass_judge_when_stopping;
    // A force-stop started before passing the line remains required while braking beyond it.
    // Once an actual STOP was observed, resuming past the line releases this obligation.
    module.force_stop_required =
      params_.unstoppable_policy == "force_stop" && (obstacle_active || keep_hold) &&
      ((was_force_stop_required && (module.state != State::STOP || stopped)) ||
       distance >= -params_.distance_to_judge_over_stop_line);
    const bool near_stop = distance <= params_.hold_stop_margin_distance + 1e-3 &&
                           (distance >= -params_.distance_to_judge_over_stop_line ||
                            params_.unstoppable_policy == "stop_after_stopline" ||
                            module.force_stop_required);
    set_state(
      module, stopped && near_stop && (obstacle_active || keep_hold) ? State::STOP : State::GO);
  }
}

void DetectionAreaStop::rebuild_modules(const TrajectoryModifierData & input)
{
  const auto route_ids = collect_route_lanelet_ids(*input.route);
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

  for (const auto lane_id : route_ids) {
    std::optional<lanelet::ConstLanelet> lane;
    try {
      lane = input.lanelet_map->laneletLayer.get(lane_id);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 5000,
        "[TM DetectionAreaStop] Cannot find route lanelet %ld: %s", lane_id, error.what());
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
        module.force_stop_required = previous->second.force_stop_required;
      }
      modules_.push_back(std::move(module));
    }
  }
}

bool DetectionAreaStop::modify_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryModifierData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "DetectionAreaStop::modify_trajectory", *get_time_keeper());
  reset_candidate_debug();
  last_candidate_modified_ = false;
  if (!enabled_ || traj_points.size() < 2 || modules_.empty() || !input.current_odometry) {
    publish_debug_string();
    return false;
  }

  // Candidate debug is separate from the immutable physical/observation snapshot.
  const auto debug_path = Trajectory::Builder{}.build(traj_points);
  if (debug_path) {
    const auto ego_s = autoware::experimental::trajectory::find_first_nearest_index(
      *debug_path, cycle_odometry_->pose.pose, ego_nearest_distance, ego_nearest_yaw_deviation);
    for (auto & module : modules_) {
      const auto stop_s = get_stop_point(
        *debug_path, module.regulatory_element->stopLine(), params_.stop_margin,
        context_->vehicle_info.max_longitudinal_offset_m);
      if (stop_s) {
        module.stop_point_arc_length = *stop_s;
        module.stop_pose = debug_path->compute(std::clamp(*stop_s, 0.0, debug_path->length())).pose;
      }
      if (params_.use_dead_line) {
        const auto deadline = get_stop_point(
          *debug_path, module.regulatory_element->stopLine(), -params_.dead_line_margin,
          context_->vehicle_info.max_longitudinal_offset_m);
        if (deadline) {
          module.dead_line_pose =
            debug_path->compute(std::clamp(*deadline, 0.0, debug_path->length())).pose;
          module.dead_line_passed = ego_s && *deadline < *ego_s;
        } else {
          module.dead_line_passed = module.physical_deadline_passed;
        }
      }
    }
  }
  auto decision = find_stop_decision(traj_points, input);
  if (decision) {
    auto & module = modules_.at(decision->module_index);
    module.stop_pose = decision->stop_pose;
    module.stop_point_arc_length = decision->stop_point_arc_length;
    module.candidate_policy = decision->policy;
  }
  if (!decision) {
    if (should_hold_stop_at_ego(traj_points, input)) {
      last_candidate_modified_ = hold_stop_at_ego(traj_points, input);
      publish_debug_string();
      return last_candidate_modified_;
    }
    publish_debug_string();
    return false;
  }

  last_candidate_modified_ = set_stop_point(traj_points, input, *decision);
  publish_debug_string();
  return last_candidate_modified_;
}

std::optional<DetectionAreaStop::StopDecision> DetectionAreaStop::find_stop_decision(
  const TrajectoryPoints & traj_points, const TrajectoryModifierData & input) const
{
  std::optional<StopDecision> nearest;
  for (const auto & module : modules_) {
    auto decision = evaluate_module(module, traj_points, input);
    if (!decision) continue;
    if (!nearest || decision->stop_point_arc_length < nearest->stop_point_arc_length) {
      nearest = std::move(decision);
    }
  }
  return nearest;
}

std::optional<DetectionAreaStop::StopDecision> DetectionAreaStop::evaluate_module(
  const Module & module, const TrajectoryPoints & traj_points,
  const TrajectoryModifierData & /*input*/) const
{
  const auto path_result = Trajectory::Builder{}.build(traj_points);
  if (!path_result) {
    return std::nullopt;
  }
  const auto & path = *path_result;

  const auto self_s = autoware::experimental::trajectory::find_first_nearest_index(
    path, cycle_odometry_->pose.pose, ego_nearest_distance, ego_nearest_yaw_deviation);
  if (!self_s) {
    return std::nullopt;
  }

  const auto stop_line = module.regulatory_element->stopLine();
  auto stop_point_s_opt = get_stop_point(
    path, stop_line, params_.stop_margin, context_->vehicle_info.max_longitudinal_offset_m);
  const bool continues_after_stop_line =
    params_.unstoppable_policy == "stop_after_stopline" ||
    (params_.unstoppable_policy == "force_stop" && module.force_stop_required);
  if (
    !stop_point_s_opt && continues_after_stop_line &&
    module.physical_stop_distance && *module.physical_stop_distance < 0.0) {
    // Once the line is behind the candidate horizon, the ego/map observation still
    // anchors the active stopping obligation; no other candidate supplies state.
    const auto candidate_ego = path.compute(*self_s).pose.position;
    const auto lane = last_lanelet_map_->laneletLayer.get(module.lane_id);
    if (lanelet::geometry::inside(lane, lanelet::BasicPoint2d{candidate_ego.x, candidate_ego.y})) {
      stop_point_s_opt = *self_s + *module.physical_stop_distance;
    }
  }
  if (!stop_point_s_opt || module.physical_deadline_passed) {
    return std::nullopt;
  }

  const double stop_point_s = *stop_point_s_opt;
  const double distance_to_stop = stop_point_s - *self_s;
  StopDecision decision;
  decision.module_index = static_cast<size_t>(&module - modules_.data());
  const bool is_stopped = !utils::is_ego_vehicle_moving(
    cycle_odometry_->twist.twist, stopping_params_.ego_stopped_vel_th);
  const auto now = cycle_time_;

  if (params_.use_dead_line) {
    const auto dead_line_s = get_stop_point(
      path, stop_line, -params_.dead_line_margin, context_->vehicle_info.max_longitudinal_offset_m);
    if (dead_line_s && *dead_line_s - *self_s < 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 1000,
        "[TM DetectionAreaStop] DetectionArea %ld is over the dead line",
        module.regulatory_element->id());
      return std::nullopt;
    }
    if (dead_line_s) {
      decision.dead_line_pose = path.compute(std::clamp(*dead_line_s, 0.0, path.length())).pose;
    }
  }

  if (
    module.state != State::STOP &&
    can_clear_stop_state(module.last_obstacle_found_time, now, params_.state_clear_time)) {
    return std::nullopt;
  }

  if (
    module.state != State::STOP && !continues_after_stop_line &&
    distance_to_stop < -params_.distance_to_judge_over_stop_line) {
    return std::nullopt;
  }

  const auto trajectory_length_m = trajectory_length(traj_points);
  double target_stop_s = stop_point_s;
  if (is_stopped && distance_to_stop < params_.hold_stop_margin_distance) {
    target_stop_s = *self_s;
  }

  const double current_velocity = cycle_odometry_->twist.twist.linear.x;
  const double braking_distance =
    std::max(0.0, current_velocity) * stopping_params_.delay_response_time +
    feasible_stop_distance_by_max_acceleration(
      current_velocity, stopping_params_.maximum_deceleration);
  const bool has_enough_distance = is_stopped || distance_to_stop > braking_distance;

  if (!has_enough_distance) {
    if (params_.unstoppable_policy == "go") {
      decision.policy = "go";
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 1000,
        "[TM DetectionAreaStop] Insufficient braking distance, policy: go");
      return std::nullopt;
    }
    if (params_.unstoppable_policy == "stop_after_stopline") {
      decision.policy = "stop_after_stopline";
      const double offset = std::max(
        feasible_stop_distance_by_max_acceleration(
          current_velocity, stopping_params_.maximum_deceleration) -
          distance_to_stop,
        0.0);
      target_stop_s = stop_point_s + offset;
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 1000,
        "[TM DetectionAreaStop] Insufficient braking distance, policy: stop_after_stopline");
    } else {
      decision.policy = "force_stop";
    }
  } else {
    decision.policy = "normal";
  }

  if (!is_stopped) {
    // The clamp operates on distance from ego, while candidate arc lengths start at
    // the trajectory origin. Use the cycle snapshot for every candidate's braking state.
    const double remaining_length = std::max(0.0, trajectory_length_m - *self_s);
    // Apply the braking constraint before capping to the candidate horizon, which may be
    // shorter than the minimum stopping distance. This keeps the shared clamp's bounds valid.
    const double clamped_distance = utils::clamp_stop_point_arc_length(
      target_stop_s - *self_s, std::numeric_limits<double>::infinity(), current_velocity,
      cycle_acceleration_, stopping_params_.maximum_deceleration, stopping_params_.jerk_limit,
      stopping_params_.delay_response_time);
    target_stop_s = *self_s + std::min(clamped_distance, remaining_length);
  }
  target_stop_s = std::clamp(target_stop_s, 0.0, trajectory_length_m);
  decision.stop_point_arc_length = target_stop_s;
  decision.stop_pose = path.compute(std::clamp(target_stop_s, 0.0, path.length())).pose;
  return decision;
}

bool DetectionAreaStop::set_stop_point(
  TrajectoryPoints & traj_points, const TrajectoryModifierData & /*input*/, StopDecision & decision)
{
  autoware_utils_debug::ScopedTimeTrack st("DetectionAreaStop::set_stop_point", *get_time_keeper());

  if (utils::stop_point_exists(traj_points, decision.stop_point_arc_length)) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM DetectionAreaStop] Preceding (or duplicate) stop point exists, skip inserting stop "
      "point");
    return false;
  }

  const auto ego_arc_length =
    motion_utils::calcSignedArcLength(traj_points, 0, cycle_odometry_->pose.pose.position);
  const auto ego_to_stop_arc_length = decision.stop_point_arc_length - ego_arc_length;
  const bool replaced_at_ego =
    ego_to_stop_arc_length < stopping_params_.arrived_distance_threshold ||
    !utils::insert_stop_point(traj_points, decision.stop_point_arc_length, trajectory_time_step_);
  if (replaced_at_ego) {
    utils::replace_trajectory_with_stop_point(
      traj_points, cycle_odometry_->pose.pose, trajectory_time_step_);
  }

  const auto & stop_pose = traj_points.back().pose;
  const auto & ego_pose = cycle_odometry_->pose.pose;
  auto distance =
    motion_utils::calcSignedArcLength(traj_points, ego_pose.position, stop_pose.position);
  if (std::isnan(distance) || distance < 1e-3) distance = 0.0;
  planning_factor_interface_->add(
    distance, stop_pose, PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{});

  auto & module = modules_.at(decision.module_index);
  module.candidate_modified = true;
  module.stop_pose = stop_pose;
  module.stop_point_arc_length = decision.stop_point_arc_length;
  module.dead_line_pose = decision.dead_line_pose;
  module.candidate_policy = decision.policy;

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[TM DetectionAreaStop] Inserted stop for DetectionArea %ld (%s) at arc length %f m",
    module.regulatory_element->id(), module.detection_source.c_str(),
    decision.stop_point_arc_length);
  return true;
}

bool DetectionAreaStop::candidate_relates_to_active_stop(
  const TrajectoryPoints & traj_points, const TrajectoryModifierData & /*input*/,
  const Module & module) const
{
  const auto path_result = Trajectory::Builder{}.build(traj_points);
  if (!path_result) return false;
  const auto & path = *path_result;

  const auto self_s = autoware::experimental::trajectory::find_first_nearest_index(
    path, cycle_odometry_->pose.pose, ego_nearest_distance, ego_nearest_yaw_deviation);
  if (!self_s) return false;

  const auto stop_line = module.regulatory_element->stopLine();
  const auto stop_point_s = get_stop_point(
    path, stop_line, params_.stop_margin, context_->vehicle_info.max_longitudinal_offset_m);
  if (!stop_point_s) return false;

  if (params_.use_dead_line) {
    const auto dead_line_s = get_stop_point(
      path, stop_line, -params_.dead_line_margin, context_->vehicle_info.max_longitudinal_offset_m);
    if (dead_line_s && *dead_line_s - *self_s < 0.0) {
      return false;
    }
  }
  return true;
}

bool DetectionAreaStop::should_hold_stop_at_ego(
  const TrajectoryPoints & traj_points, const TrajectoryModifierData & input) const
{
  if (!input.current_odometry) return false;

  const bool is_stopped = !utils::is_ego_vehicle_moving(
    cycle_odometry_->twist.twist, stopping_params_.ego_stopped_vel_th);
  if (!is_stopped) return false;

  for (const auto & module : modules_) {
    if (module.state != State::STOP) continue;

    if (!candidate_relates_to_active_stop(traj_points, input, module)) {
      continue;
    }
    return true;
  }
  return false;
}

bool DetectionAreaStop::hold_stop_at_ego(
  TrajectoryPoints & traj_points, const TrajectoryModifierData & /*input*/)
{
  utils::replace_trajectory_with_stop_point(
    traj_points, cycle_odometry_->pose.pose, trajectory_time_step_);
  return true;
}

void DetectionAreaStop::publish_debug_string() const
{
  std::ostringstream text;
  text << std::fixed << std::setprecision(2) << std::boolalpha;
  text << "DETECTION AREA STOP MODIFIER:\n";
  text << "\tMODIFIED: " << last_candidate_modified_ << "\n";
  text << "\tSTATUS: " << (debug_status_.empty() ? "ready" : debug_status_) << "\n";
  text << "\tMODULES: " << modules_.size() << "\n";
  for (const auto & module : modules_) {
    if (!module.regulatory_element) continue;
    text << "\tDetectionArea " << module.regulatory_element->id() << " lane " << module.lane_id
         << ": state=" << (module.state == State::STOP ? "STOP" : "GO")
         << ", source=" << (module.detection_source.empty() ? "none" : module.detection_source)
         << ", pointcloud_points=" << module.obstacle_points.size()
         << ", object_polygons=" << module.object_polygons.size();
    if (module.stop_pose) {
      text << ", stop_s=" << module.stop_point_arc_length;
    }
    if (params_.use_dead_line) {
      text << ", dead_line=" << (module.dead_line_passed ? "passed" : "clear");
    }
    if (!module.candidate_policy.empty()) {
      text << ", policy=" << module.candidate_policy;
    }
    text << ", candidate_modified=" << module.candidate_modified << "\n";
  }

  StringStamped debug_text;
  debug_text.stamp = get_clock()->now();
  debug_text.data = text.str();
  pub_debug_text_->publish(debug_text);
}

void DetectionAreaStop::publish_debug_data(const std::string & ns) const
{
  const auto now = cycle_time_;
  const auto green = autoware_utils::create_marker_color(0.0, 1.0, 0.0, 1.0);
  const auto red = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 1.0);
  const auto white = autoware_utils::create_marker_color(1.0, 1.0, 1.0, 1.0);
  const auto orange = autoware_utils::create_marker_color(1.0, 0.5, 0.0, 1.0);
  const auto magenta = autoware_utils::create_marker_color(1.0, 0.0, 1.0, 1.0);
  const auto gray = autoware_utils::create_marker_color(0.6, 0.6, 0.6, 1.0);

  MarkerArray marker_array;
  int marker_id = 0;
  const auto lifetime = rclcpp::Duration::from_seconds(0.2);

  const auto add_line_marker = [&](
                                 const std::string & marker_ns,
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

  const auto add_point_marker = [&](
                                  const std::string & marker_ns,
                                  const geometry_msgs::msg::Point & point,
                                  const std_msgs::msg::ColorRGBA & color, const double scale) {
    auto marker = autoware_utils::create_default_marker(
      "map", now, marker_ns, marker_id++, Marker::SPHERE,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.lifetime = lifetime;
    marker.pose.position = point;
    marker_array.markers.push_back(marker);
  };

  const auto add_text_marker = [&](
                                 const std::string & marker_ns,
                                 const geometry_msgs::msg::Point & point, const std::string & text,
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
    const auto module_ns =
      ns + "/detection_area/" + module_key(module.lane_id, module.regulatory_element->id());
    const auto state_color = module.state == State::STOP ? red : green;

    for (const auto & detection_area_polygon : module.regulatory_element->detectionAreas()) {
      const auto polygon = lanelet::utils::to2D(detection_area_polygon).basicPolygon();
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
