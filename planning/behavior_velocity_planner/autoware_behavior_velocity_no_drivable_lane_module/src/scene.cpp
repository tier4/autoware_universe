// Copyright 2023 TIER IV, Inc.
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

#include "scene.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "util.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::behavior_velocity_planner
{
using autoware_utils::create_point;

NoDrivableLaneModule::NoDrivableLaneModule(
  const int64_t module_id, const int64_t lane_id, const PlannerParam & planner_param,
  const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  planner_param_(planner_param),
  debug_data_(),
  state_(State::INIT)
{
}

bool NoDrivableLaneModule::modifyPathVelocity(PathWithLaneId * _path)
{
  auto path = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>::Builder{}.build(
    _path->points);
  if (!path) {
    return false;
  }

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto & lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto & no_drivable_lane = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto & no_drivable_lane_polygon =
    lanelet::utils::to2D(no_drivable_lane).polygon2d().basicPolygon();

  const auto path_polygon_intersection =
    getPathIntersectionWithNoDrivableLanePolygon(*path, no_drivable_lane_polygon, 2);

  const auto ego_front_s = experimental::trajectory::find_nearest_index(*path, ego_pos) +
                           planner_data_->vehicle_info_.max_longitudinal_offset_m;

  initialize_debug_data(*path, no_drivable_lane, ego_pos, path_polygon_intersection);

  switch (state_) {
    case State::INIT: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "Init");
      }

      handle_init_state(ego_front_s, path_polygon_intersection);

      break;
    }

    case State::APPROACHING: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "Approaching ");
      }

      handle_approaching_state(*path, ego_front_s, path_polygon_intersection);

      break;
    }

    case State::INSIDE_NO_DRIVABLE_LANE: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "INSIDE_NO_DRIVABLE_LANE");
      }

      handle_inside_no_drivable_lane_state(*path, ego_front_s);

      break;
    }

    case State::STOPPED: {
      if (planner_param_.print_debug_info) {
        RCLCPP_INFO(logger_, "STOPPED");
      }

      handle_stopped_state(*path, ego_front_s);

      break;
    }

    default: {
      RCLCPP_ERROR(logger_, "ERROR. Undefined case");
      return false;
    }
  }

  _path->points = path->restore();

  return true;
}

void NoDrivableLaneModule::handle_init_state(
  const double ego_front_s,
  const PathWithNoDrivableLanePolygonIntersection & path_polygon_intersection)
{
  if (
    path_polygon_intersection.is_first_path_point_inside_polygon ||
    (path_polygon_intersection.first_intersection_s &&
     *path_polygon_intersection.first_intersection_s - ego_front_s <= planner_param_.stop_margin)) {
    state_ = State::INSIDE_NO_DRIVABLE_LANE;
  } else {
    state_ = State::APPROACHING;
  }
}

void NoDrivableLaneModule::handle_approaching_state(
  experimental::trajectory::Trajectory<PathPointWithLaneId> & path, const double ego_front_s,
  const PathWithNoDrivableLanePolygonIntersection & path_polygon_intersection)
{
  if (!path_polygon_intersection.first_intersection_s) {
    return;
  }

  const auto longitudinal_offset =
    -(planner_param_.stop_margin + planner_data_->vehicle_info_.max_longitudinal_offset_m);
  const auto stop_s = *path_polygon_intersection.first_intersection_s + longitudinal_offset;

  path.longitudinal_velocity_mps().range(stop_s, path.length()).set(0.0);

  // Get stop point and stop factor
  planning_factor_interface_->add(
    path.restore(), planner_data_->current_odometry->pose, path.compute(stop_s).point.pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  debug_data_.stop_pose = path.compute(stop_s + debug_data_.base_link2front).point.pose;

  // Move to stopped state if stopped
  const auto distance_ego_first_intersection =
    *path_polygon_intersection.first_intersection_s - ego_front_s;
  if (
    (distance_ego_first_intersection <= planner_param_.stop_margin) &&
    planner_data_->isVehicleStopped()) {
    if (planner_param_.print_debug_info) {
      RCLCPP_INFO(logger_, "APPROACHING -> STOPPED");
      RCLCPP_INFO_STREAM(
        logger_, "distance_ego_first_intersection = " << distance_ego_first_intersection);
    }

    if (distance_ego_first_intersection < 0.0) {
      RCLCPP_ERROR(
        logger_, "Failed to stop before no drivable lane but ego stopped. Change state to STOPPED");
    }

    state_ = State::STOPPED;
  }
}

void NoDrivableLaneModule::handle_inside_no_drivable_lane_state(
  experimental::trajectory::Trajectory<PathPointWithLaneId> & path, const double ego_front_s)
{
  // Insert stop point
  path.longitudinal_velocity_mps()
    .range(ego_front_s - planner_data_->vehicle_info_.max_longitudinal_offset_m, path.length())
    .set(0.0);

  // Get stop point and stop factor
  planning_factor_interface_->add(
    path.restore(), planner_data_->current_odometry->pose, path.compute(0).point.pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  debug_data_.stop_pose = path.compute(debug_data_.base_link2front).point.pose;

  // Move to stopped state if stopped
  if (planner_data_->isVehicleStopped()) {
    if (planner_param_.print_debug_info) {
      RCLCPP_INFO(logger_, "APPROACHING -> STOPPED");
    }
    state_ = State::STOPPED;
  }
}

void NoDrivableLaneModule::handle_stopped_state(
  experimental::trajectory::Trajectory<PathPointWithLaneId> & path, const double ego_front_s)
{
  // Insert stop pose
  const auto ego_s = ego_front_s - planner_data_->vehicle_info_.max_longitudinal_offset_m;
  path.longitudinal_velocity_mps().range(ego_s, path.length()).set(0.0);

  // Get stop point and stop factor
  planning_factor_interface_->add(
    path.restore(), planner_data_->current_odometry->pose, path.compute(ego_s).point.pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  debug_data_.stop_pose = path.compute(ego_s + debug_data_.base_link2front).point.pose;
}

void NoDrivableLaneModule::initialize_debug_data(
  const experimental::trajectory::Trajectory<PathPointWithLaneId> & path,
  const lanelet::Lanelet & no_drivable_lane, const geometry_msgs::msg::Point & ego_pos,
  const PathWithNoDrivableLanePolygonIntersection & path_polygon_intersection)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  if (path_polygon_intersection.first_intersection_s) {
    debug_data_.first_intersection_point =
      path.compute(*path_polygon_intersection.first_intersection_s).point.pose.position;
  }
  if (path_polygon_intersection.second_intersection_s) {
    debug_data_.second_intersection_point =
      path.compute(*path_polygon_intersection.second_intersection_s).point.pose.position;
  }

  for (const auto & p : no_drivable_lane.polygon2d().basicPolygon()) {
    debug_data_.no_drivable_lane_polygon.push_back(create_point(p.x(), p.y(), ego_pos.z));
  }
}

}  // namespace autoware::behavior_velocity_planner
