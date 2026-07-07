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

#include "autoware/behavior_path_freespace_area_module/scene.hpp"

#include "autoware/behavior_path_freespace_area_module/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"

#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::freespace_planning_algorithms::AstarSearch;
using autoware::freespace_planning_algorithms::RRTStar;
using autoware::freespace_planning_algorithms::VehicleShape;
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::findNearestIndex;

namespace
{
// RAII running-flag, mirroring goal_planner's ScopedFlag: set before any shared state is
// touched and cleared on every exit path.
class ScopedRunningFlag
{
public:
  explicit ScopedRunningFlag(std::atomic<bool> & flag) : flag_(flag) { flag_.store(true); }
  ~ScopedRunningFlag() { flag_.store(false); }

private:
  std::atomic<bool> & flag_;
};
}  // namespace

void FreespaceAreaPlanner::onTimer()
{
  const ScopedRunningFlag running_flag(ctx_->is_running);

  FreespaceAreaPlanRequest req;
  {
    std::lock_guard<std::mutex> guard(ctx_->mutex);
    if (!ctx_->request || !ctx_->request->valid) {
      return;
    }
    req = *ctx_->request;
  }

  FreespaceAreaPlanResponse res;
  res.stamp = clock_->now();

  try {
    algo_->setMap(req.costmap);

    if (replan_when_obstacle_found_ && !req.latched_trajectory.poses.empty()) {
      res.obstacle_on_latched = algo_->hasObstacleOnTrajectory(req.latched_trajectory);
    }

    if (req.need_plan && algo_->makePlan(req.start_pose, req.goal_pose)) {
      res.success = true;
      res.waypoints = algo_->getWaypoints();
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 3000, "freespace_area planning failed: %s", e.what());
    res.success = false;
  }

  {
    std::lock_guard<std::mutex> guard(ctx_->mutex);
    // Preserve the newest successful waypoints if this cycle only did an obstacle re-check.
    if (!res.success && ctx_->response.success && !req.need_plan) {
      res.success = true;
      res.waypoints = ctx_->response.waypoints;
      res.stamp = ctx_->response.stamp;
    }
    ctx_->response = res;
  }
}

FreespaceAreaModule::FreespaceAreaModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<FreespaceAreaParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map, planning_factor_interface},  // NOLINT
  parameters_{parameters},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo()},
  worker_ctx_{std::make_shared<FreespaceAreaWorkerContext>()}
{
  const VehicleShape vehicle_shape(vehicle_info_, parameters_->planner_vehicle_shape_margin);
  if (parameters_->planner_algorithm == "astar") {
    algo_ = std::make_shared<AstarSearch>(
      parameters_->common_param, vehicle_shape, parameters_->astar_param, clock_);
  } else if (parameters_->planner_algorithm == "rrtstar") {
    algo_ = std::make_shared<RRTStar>(
      parameters_->common_param, vehicle_shape, parameters_->rrtstar_param, clock_);
  } else {
    throw std::runtime_error(
      "freespace_area: unknown planner.algorithm '" + parameters_->planner_algorithm + "'");
  }

  timer_cb_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto period_ns = rclcpp::Rate(parameters_->planner_update_rate).period();
  timer_ = rclcpp::create_timer(
    &node, clock_, period_ns,
    [worker = std::make_unique<FreespaceAreaPlanner>(
       worker_ctx_, getLogger(), clock_, algo_, parameters_->replan_when_obstacle_found)]() {
      worker->onTimer();
    },
    timer_cb_group_);
}

FreespaceAreaModule::~FreespaceAreaModule()
{
  // Stop future worker invocations. A callback that is already in flight is safe: it owns the
  // worker context (and the algorithm object) via shared_ptr, so no spin-wait is needed here
  // (a spin-wait would also block the planner main thread for up to the A* time limit).
  if (timer_) {
    timer_->cancel();
  }
}

void FreespaceAreaModule::initVariables()
{
  state_ = FreespaceAreaState::IDLE;
  mode_ = FreespaceAreaMode::NONE;
  composed_path_ = PathWithLaneId();
  latched_response_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  stopped_since_.reset();
  {
    std::lock_guard<std::mutex> guard(worker_ctx_->mutex);
    worker_ctx_->request.reset();
    worker_ctx_->response = FreespaceAreaPlanResponse();
  }
  resetPathCandidate();
  resetPathReference();
}

void FreespaceAreaModule::processOnEntry()
{
  initVariables();
}

void FreespaceAreaModule::processOnExit()
{
  initVariables();
}

void FreespaceAreaModule::updateData()
{
  if (
    !planner_data_ || !planner_data_->route_handler ||
    !planner_data_->route_handler->isHandlerReady()) {
    return;
  }
  // Track route changes so the latch can be invalidated when the route/goal is replaced.
  const auto uuid = planner_data_->route_handler->getRouteUuid().uuid;
  if (!route_uuid_ || *route_uuid_ != uuid) {
    if (route_uuid_) {
      // route changed -> drop the latch
      composed_path_ = PathWithLaneId();
      state_ = FreespaceAreaState::IDLE;
    }
    route_uuid_ = uuid;
  }
}

bool FreespaceAreaModule::isExecutionReady() const
{
  return true;
}

bool FreespaceAreaModule::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }
  if (!planner_data_ || !planner_data_->route_handler) {
    return false;
  }
  // Fast exit on normal routes: no areas -> zero overhead.
  if (planner_data_->route_handler->getRouteAreas().empty()) {
    return false;
  }
  const auto ctx = evaluateActivation();
  return ctx.has_value() && ctx->active;
}

std::optional<FreespaceAreaModule::ActivationContext> FreespaceAreaModule::evaluateActivation()
  const
{
  const auto & rh = planner_data_->route_handler;
  if (!rh || rh->getRouteAreas().empty() || !planner_data_->self_odometry) {
    return std::nullopt;
  }

  const auto ego_pose = planner_data_->self_odometry->pose.pose;

  const auto area_at_ego = rh->getRouteAreaAtPose(ego_pose);
  const bool ego_inside = area_at_ego.has_value();

  std::optional<autoware::route_handler::AreaTransit> transit;
  lanelet::ConstLanelet closest;
  if (rh->getClosestLaneletWithinRoute(ego_pose, &closest)) {
    transit = rh->getNextAreaTransit(closest);
  }

  if (!transit && !ego_inside) {
    return std::nullopt;
  }

  ActivationContext ctx;
  ctx.ego_inside_area = ego_inside;

  if (transit) {
    ctx.area = transit->area;
    ctx.entry_lanelets = transit->entry_lanelets;
    ctx.exit_lanelets = transit->exit_lanelets;
    ctx.mode =
      transit->exit_lanelets.empty() ? FreespaceAreaMode::TERMINAL : FreespaceAreaMode::TRANSIT;
  } else {
    ctx.area = *area_at_ego;
    ctx.mode = rh->isGoalInRouteArea() ? FreespaceAreaMode::TERMINAL : FreespaceAreaMode::TRANSIT;
  }

  bool within_lookahead = ego_inside;
  if (transit && !ego_inside && !transit->entry_lanelets.empty()) {
    const auto entry_path =
      rh->getCenterLinePath(transit->entry_lanelets, 0.0, std::numeric_limits<double>::max());
    const auto & prev_path = getPreviousModuleOutput().path;
    if (!entry_path.points.empty() && !prev_path.points.empty()) {
      const auto & entry_end = entry_path.points.back().point.pose.position;
      const double dist = calcSignedArcLength(prev_path.points, ego_pose.position, entry_end);
      within_lookahead = dist <= parameters_->activation_lookahead_distance;
    }
  }

  ctx.active = within_lookahead || ego_inside;
  return ctx;
}

geometry_msgs::msg::Pose FreespaceAreaModule::computeStartPose(const ActivationContext & ctx) const
{
  const auto & rh = planner_data_->route_handler;
  if (ctx.ego_inside_area || ctx.entry_lanelets.empty()) {
    return planner_data_->self_odometry->pose.pose;
  }
  const auto entry_path =
    rh->getCenterLinePath(ctx.entry_lanelets, 0.0, std::numeric_limits<double>::max());
  if (entry_path.points.empty()) {
    return planner_data_->self_odometry->pose.pose;
  }
  // Inset the A* start back along the entry centerline: the boundary pose sits exactly on the
  // Area edge, where the footprint collision check is marginal by construction (the outside of
  // the lane<->area corner is not covered by any free-space polygon).
  const auto inset_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    entry_path.points, entry_path.points.back().point.pose.position,
    -parameters_->junction_inset_distance);
  if (inset_pose) {
    return *inset_pose;
  }
  return entry_path.points.back().point.pose;
}

geometry_msgs::msg::Pose FreespaceAreaModule::computeGoalPose(const ActivationContext & ctx) const
{
  const auto & rh = planner_data_->route_handler;
  if (ctx.mode == FreespaceAreaMode::TERMINAL || ctx.exit_lanelets.empty()) {
    return rh->getGoalPose();
  }
  const auto exit_path =
    rh->getCenterLinePath(ctx.exit_lanelets, 0.0, std::numeric_limits<double>::max());
  if (exit_path.points.empty()) {
    return rh->getGoalPose();
  }
  // Inset the A* goal forward along the exit centerline (same rationale as computeStartPose).
  const auto inset_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    exit_path.points, exit_path.points.front().point.pose.position,
    parameters_->junction_inset_distance);
  if (inset_pose) {
    return *inset_pose;
  }
  return exit_path.points.front().point.pose;
}

bool FreespaceAreaModule::isLatchInvalid(const geometry_msgs::msg::Pose & start_pose)
{
  if (composed_path_.points.empty()) {
    return true;
  }

  const auto ego_pose = planner_data_->self_odometry->pose.pose;

  // lateral deviation from the latched path
  const double lateral =
    std::abs(autoware::motion_utils::calcLateralOffset(composed_path_.points, ego_pose.position));
  if (lateral > parameters_->replan_lateral_deviation) {
    RCLCPP_INFO(getLogger(), "freespace_area: replan (lateral deviation %.2f m)", lateral);
    return true;
  }

  // obstacle on the latched area segment (reported by the worker)
  bool obstacle_on_latched = false;
  {
    std::lock_guard<std::mutex> guard(worker_ctx_->mutex);
    obstacle_on_latched = worker_ctx_->response.obstacle_on_latched;
  }
  if (parameters_->replan_when_obstacle_found && obstacle_on_latched) {
    RCLCPP_INFO(getLogger(), "freespace_area: replan (obstacle on latched trajectory)");
    return true;
  }

  // unexpected stuck (not at goal, not at a cusp)
  const double v = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  const bool near_goal =
    autoware_utils::calc_distance2d(ego_pose.position, latched_goal_pose_.position) <
    parameters_->goal_position_tolerance * 3.0;
  if (v < 0.05 && !near_goal) {
    if (!stopped_since_) {
      stopped_since_ = clock_->now();
    } else if ((clock_->now() - *stopped_since_).seconds() > parameters_->stuck_time_threshold) {
      stopped_since_.reset();
      RCLCPP_INFO(getLogger(), "freespace_area: replan (stuck)");
      return true;
    }
  } else {
    stopped_since_.reset();
  }

  (void)start_pose;
  return false;
}

BehaviorModuleOutput FreespaceAreaModule::plan()
{
  BehaviorModuleOutput previous_output = getPreviousModuleOutput();

  const auto ctx_opt = evaluateActivation();
  if (!ctx_opt || !ctx_opt->active) {
    return previous_output;
  }
  current_ctx_ = *ctx_opt;
  mode_ = current_ctx_.mode;

  const auto start_pose = computeStartPose(current_ctx_);
  const auto goal_pose = computeGoalPose(current_ctx_);

  const bool costmap_fresh =
    planner_data_->costmap &&
    (clock_->now() - rclcpp::Time(planner_data_->costmap->header.stamp)).seconds() < 1.0;

  const bool need_replan = isLatchInvalid(start_pose);
  if (need_replan) {
    composed_path_ = PathWithLaneId();
    state_ = FreespaceAreaState::PLANNING;
  }

  // Publish request to the worker.
  requestPlan(start_pose, goal_pose);
  if (!costmap_fresh) {
    std::lock_guard<std::mutex> guard(worker_ctx_->mutex);
    if (worker_ctx_->request) worker_ctx_->request->valid = false;
  }

  // Adopt a fresh successful plan if available (copy the response out under the lock, then
  // compose outside of it).
  std::optional<FreespaceAreaPlanResponse> adopted;
  {
    std::lock_guard<std::mutex> guard(worker_ctx_->mutex);
    const auto & response = worker_ctx_->response;
    if (
      response.success && response.stamp > latched_response_stamp_ &&
      !response.waypoints.waypoints.empty()) {
      adopted = response;
    }
  }
  if (adopted) {
    latched_response_stamp_ = adopted->stamp;
    latched_start_pose_ = start_pose;
    latched_goal_pose_ = goal_pose;

    // Build the composed path from the freespace waypoints.
    lanelet::ConstLanelets lanes = current_ctx_.entry_lanelets;
    lanes.insert(lanes.end(), current_ctx_.exit_lanelets.begin(), current_ctx_.exit_lanelets.end());
    PathWithLaneId area_path = utils::convertWayPointsToPathWithLaneId(
      adopted->waypoints, parameters_->planner_velocity, lanes);
    freespace_area_utils::encodeTravelDirectionOrientation(area_path);

    composed_path_ = area_path;
    state_ = FreespaceAreaState::LATCHED;
  }

  if (composed_path_.points.empty()) {
    // still planning: let the vehicle keep following the previous (lane) path toward the area.
    return previous_output;
  }

  return composeOutput();
}

void FreespaceAreaModule::requestPlan(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  if (!planner_data_->costmap) {
    return;
  }
  FreespaceAreaPlanRequest req;
  req.valid = true;
  req.need_plan = (state_ != FreespaceAreaState::LATCHED);
  req.costmap = *planner_data_->costmap;
  req.start_pose = start_pose;
  req.goal_pose = goal_pose;
  if (!composed_path_.points.empty()) {
    req.latched_trajectory = freespace_area_utils::toPoseArray(composed_path_);
  }
  std::lock_guard<std::mutex> guard(worker_ctx_->mutex);
  worker_ctx_->request = req;
}

BehaviorModuleOutput FreespaceAreaModule::composeOutput()
{
  BehaviorModuleOutput output;
  const auto previous_output = getPreviousModuleOutput();

  PathWithLaneId path;
  path.header = planner_data_->route_handler->getRouteHeader();

  // 1. Truncate the previous (lane) path at the area entry (nearest to the A* start pose).
  const auto & prev_points = previous_output.path.points;
  if (!prev_points.empty()) {
    const auto entry_idx = findNearestIndex(prev_points, latched_start_pose_.position);
    const size_t end_idx = std::min(entry_idx, prev_points.size() - 1);
    path.points.assign(prev_points.begin(), prev_points.begin() + end_idx + 1);
  }

  auto append_point = [&path](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & p) {
    if (!path.points.empty()) {
      const auto & last = path.points.back().point.pose.position;
      if (autoware_utils::calc_distance2d(last, p.point.pose.position) < 0.05) {
        return;
      }
    }
    path.points.push_back(p);
  };

  // 2. Append the composed area path (already cusp-encoded, positive velocities).
  const size_t area_start_index = path.points.size();
  for (const auto & p : composed_path_.points) {
    append_point(p);
  }

  // 3. Transit mode: append the exit-lane centerline continuation. Start from the point nearest
  // to the A* goal (which is inset into the exit lane) so the path does not double back over the
  // insetted segment.
  if (mode_ == FreespaceAreaMode::TRANSIT && !current_ctx_.exit_lanelets.empty()) {
    const auto exit_path = planner_data_->route_handler->getCenterLinePath(
      current_ctx_.exit_lanelets, 0.0, std::numeric_limits<double>::max());
    if (!exit_path.points.empty()) {
      const auto goal_idx = findNearestIndex(exit_path.points, latched_goal_pose_.position);
      for (size_t i = goal_idx; i < exit_path.points.size(); ++i) {
        append_point(exit_path.points[i]);
      }
    }
  }

  // 4. Light junction blend: interpolate velocity from the lane speed into the area speed over
  // junction_blend_distance so there is no step change at the entry junction.
  if (area_start_index > 0 && area_start_index < path.points.size()) {
    const double lane_speed = path.points[area_start_index - 1].point.longitudinal_velocity_mps;
    const double blend = parameters_->junction_blend_distance;
    for (size_t i = area_start_index; i < path.points.size(); ++i) {
      const double s = calcSignedArcLength(path.points, area_start_index, i);
      if (s >= blend) break;
      const double r = (blend > 1e-3) ? (s / blend) : 1.0;
      auto & v = path.points[i].point.longitudinal_velocity_mps;
      v = lane_speed * (1.0 - r) + v * r;
    }
  }

  // 5. Terminal mode ends with zero velocity at the goal.
  if (mode_ == FreespaceAreaMode::TERMINAL && !path.points.empty()) {
    path.points.back().point.longitudinal_velocity_mps = 0.0;
  }

  output.path = path;
  output.reference_path = previous_output.reference_path;
  output.turn_signal_info = previous_output.turn_signal_info;

  // Drivable area: build a corridor (tube) around the whole composed path via drivable_margin.
  // The planner_manager uses the "single free space pull over" branch whenever drivable_margin is
  // non-zero (see PlannerManager::generateCombinedDrivableArea), which offsets the path bounds by
  // this margin. This works through arbitrary Area geometry without needing lanelet bounds.
  DrivableAreaInfo drivable_area_info;
  drivable_area_info.drivable_margin =
    0.5 * vehicle_info_.vehicle_width_m + parameters_->planner_vehicle_shape_margin;
  output.drivable_area_info = drivable_area_info;

  return output;
}

BehaviorModuleOutput FreespaceAreaModule::planWaitingApproval()
{
  return plan();
}

CandidateOutput FreespaceAreaModule::planCandidate() const
{
  CandidateOutput output;
  output.path_candidate = composed_path_;
  return output;
}

bool FreespaceAreaModule::canTransitSuccessState()
{
  if (!planner_data_ || !planner_data_->self_odometry || composed_path_.points.empty()) {
    return false;
  }
  const auto ego_pose = planner_data_->self_odometry->pose.pose;

  if (mode_ == FreespaceAreaMode::TERMINAL) {
    const double dist =
      autoware_utils::calc_distance2d(ego_pose.position, latched_goal_pose_.position);
    const double yaw_diff = std::abs(
      autoware_utils::normalize_radian(
        tf2::getYaw(ego_pose.orientation) - tf2::getYaw(latched_goal_pose_.orientation)));
    const double v = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
    const bool arrived = dist < parameters_->goal_position_tolerance &&
                         yaw_diff < autoware_utils::deg2rad(parameters_->goal_yaw_tolerance_deg) &&
                         v < 0.1;
    if (arrived) {
      state_ = FreespaceAreaState::COMPLETED;
      return true;
    }
    return false;
  }

  // TRANSIT: success once ego's closest lanelet is an exit lanelet and it has passed the junction.
  const auto & rh = planner_data_->route_handler;
  if (!rh || current_ctx_.exit_lanelets.empty()) {
    return false;
  }
  lanelet::ConstLanelet closest;
  if (!rh->getClosestLaneletWithinRoute(ego_pose, &closest)) {
    return false;
  }
  const bool on_exit_lane = std::any_of(
    current_ctx_.exit_lanelets.begin(), current_ctx_.exit_lanelets.end(),
    [&closest](const auto & l) { return l.id() == closest.id(); });
  if (!on_exit_lane) {
    return false;
  }
  // ensure ego has passed the exit junction: the junction pose (A* goal) is behind ego in the
  // junction's own frame, or ego is within the blend window of it.
  const double goal_yaw = tf2::getYaw(latched_goal_pose_.orientation);
  const double dx = ego_pose.position.x - latched_goal_pose_.position.x;
  const double dy = ego_pose.position.y - latched_goal_pose_.position.y;
  const double lon = std::cos(goal_yaw) * dx + std::sin(goal_yaw) * dy;
  const double dist_to_goal =
    autoware_utils::calc_distance2d(ego_pose.position, latched_goal_pose_.position);
  if (lon > 0.0 || dist_to_goal < parameters_->junction_blend_distance) {
    state_ = FreespaceAreaState::COMPLETED;
    return true;
  }
  return false;
}

}  // namespace autoware::behavior_path_planner
