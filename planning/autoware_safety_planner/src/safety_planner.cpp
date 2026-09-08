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

#include "safety_planner.hpp"

#include "utils/reference_path_smoother.hpp"

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/trajectory/threshold.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware/trajectory/utils/reference_path.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

//! centerline の goal 周辺 (search_radius_m 以内) を捨て、goal 手前 1.0 / 0.5 m の点と goal を
//! 加えてスプラインで作り直す (goal_planner の smooth goal connection と同じ構成)。
//! 接続区間がレーン外に出る場合は半径を 1 m ずつ縮めて再試行し、goal が経路から遠すぎる
//! (半径の外) 場合と構築に失敗した場合は nullopt
std::optional<PathPointTrajectory> connect_reference_path_to_goal(
  const PathPointTrajectory & path, const Pose & goal,
  const lanelet::ConstLanelets & route_lanelets, const lanelet::LaneletMapConstPtr & lanelet_map,
  const double search_radius_m)
{
  using autoware_utils_geometry::calc_distance2d;
  //! [m] goal 手前に置く接続点の距離 (goal_planner と同じ値)
  constexpr double PRE_GOAL_DISTANCE_M = 1.0;
  constexpr double PRE_MID_GOAL_DISTANCE_M = 0.5;
  //! [m] 接続区間のレーン内判定のサンプル間隔
  constexpr double VALIDATION_STEP_M = 1.0;
  //! [m] 再試行ごとの半径の縮め幅
  constexpr double RADIUS_REDUCE_M = 1.0;

  const double s_goal = experimental::trajectory::closest(path, goal.position);
  if (calc_distance2d(path.compute(s_goal).point.pose, goal) > search_radius_m) {
    return std::nullopt;  // goal が前方窓の外、または横に遠すぎて接続対象でない
  }
  const auto points = path.restore();
  const auto bases = path.get_underlying_bases();

  const double goal_yaw = autoware_utils_geometry::get_rpy(goal).z;
  const double goal_z = path.compute(s_goal).point.pose.position.z;
  const auto make_goal_point = [&](const double back_distance) {
    PathPointWithLaneId point = points.back();
    point.point.pose = goal;
    point.point.pose.position.x -= std::cos(goal_yaw) * back_distance;
    point.point.pose.position.y -= std::sin(goal_yaw) * back_distance;
    point.point.pose.position.z = goal_z;
    return point;
  };

  const auto is_inside_map_lane = [&](const geometry_msgs::msg::Pose & pose) {
    const lanelet::BasicPoint2d p{pose.position.x, pose.position.y};
    for (const auto & lanelet : route_lanelets) {
      if (lanelet::geometry::inside(lanelet, p)) {
        return true;
      }
    }
    // 路肩の goal など route 外の lanelet も走行可能領域に含める
    return !experimental::lanelet2_utils::get_road_lanelets_at(lanelet_map, p.x(), p.y()).empty() ||
           !experimental::lanelet2_utils::get_shoulder_lanelets_at(lanelet_map, p.x(), p.y())
              .empty();
  };

  std::optional<PathPointTrajectory> last_built;
  for (double radius = search_radius_m; radius >= 0.0; radius -= RADIUS_REDUCE_M) {
    // goal 手前で、goal から radius より遠い最後の点まで残す (goal 側から後退して探す)
    std::size_t cut_index = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (bases[i] > s_goal) {
        break;
      }
      cut_index = i;
      if (calc_distance2d(points[i].point.pose, goal) > radius) {
        continue;
      }
      cut_index = i == 0 ? 0 : i - 1;
      break;
    }

    std::vector<PathPointWithLaneId> connected(
      points.begin(), points.begin() + static_cast<std::ptrdiff_t>(cut_index) + 1);
    connected.push_back(make_goal_point(PRE_GOAL_DISTANCE_M));
    connected.push_back(make_goal_point(PRE_MID_GOAL_DISTANCE_M));
    connected.push_back(make_goal_point(0.0));
    for (auto it = connected.end() - 3; it != connected.end(); ++it) {
      it->lane_ids = points[cut_index].lane_ids;
    }

    auto built = experimental::trajectory::pretty_build(connected);
    if (!built) {
      continue;
    }
    last_built = std::move(*built);

    bool valid = true;
    for (double s = bases[cut_index]; s <= last_built->length(); s += VALIDATION_STEP_M) {
      if (!is_inside_map_lane(last_built->compute(s).point.pose)) {
        valid = false;
        break;
      }
    }
    if (valid) {
      break;
    }
  }
  return last_built;
}

}  // namespace

SafetyPlanner::SafetyPlanner(const Params & params, std::shared_ptr<TimeKeeper> time_keeper)
: params_(params), time_keeper_(std::move(time_keeper))
{
  load_constraint_generator_plugins();
  load_trajectory_planner_plugin();
}

void SafetyPlanner::load_constraint_generator_plugins()
{
  const auto logger = rclcpp::get_logger("safety_planner");
  constraint_generator_loader_ = std::make_unique<ConstraintGeneratorLoader>(
    "autoware_safety_planner", "autoware::safety_planner::ConstraintGeneratorInterface");

  for (const auto & class_name : params_.constraint_generator_plugins) {
    try {
      auto plugin = constraint_generator_loader_->createSharedInstance(class_name);
      plugin->on_initialize(time_keeper_, params_);
      const auto name = plugin->get_name();
      constraint_generator_plugins_.push_back(std::move(plugin));
      RCLCPP_INFO(
        logger, "Loaded constraint generator plugin: %s (%s)", name.c_str(), class_name.c_str());
    } catch (const pluginlib::PluginlibException & e) {
      RCLCPP_ERROR(
        logger, "Failed to load constraint generator plugin '%s': %s", class_name.c_str(),
        e.what());
    }
  }
}

std::vector<std::string> SafetyPlanner::get_constraint_generator_plugin_names() const
{
  std::vector<std::string> names;
  names.reserve(constraint_generator_plugins_.size());
  for (const auto & plugin : constraint_generator_plugins_) {
    names.push_back(plugin->get_name());
  }
  return names;
}

void SafetyPlanner::load_trajectory_planner_plugin()
{
  const auto logger = rclcpp::get_logger("safety_planner");
  trajectory_planner_loader_ = std::make_unique<TrajectoryPlannerLoader>(
    "autoware_safety_planner", "autoware::safety_planner::TrajectoryPlannerInterface");

  const auto & class_name = params_.trajectory_planner_plugin;
  try {
    trajectory_planner_ = trajectory_planner_loader_->createSharedInstance(class_name);
    trajectory_planner_->on_initialize(time_keeper_, params_);
    RCLCPP_INFO(
      logger, "Loaded trajectory planner plugin: %s (%s)", trajectory_planner_->get_name().c_str(),
      class_name.c_str());
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(
      logger, "Failed to load trajectory planner plugin '%s': %s", class_name.c_str(), e.what());
  }
}

std::string SafetyPlanner::get_trajectory_planner_plugin_name() const
{
  return trajectory_planner_ ? trajectory_planner_->get_name() : std::string{};
}

tl::expected<PathPointTrajectory, std::string> SafetyPlanner::build_reference_path(
  const SafetyPlannerInput & input) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & route_manager = *input.route_manager;
  const double forward_length = params_.reference_path.forward_length_m;
  const double backward_length = params_.reference_path.backward_length_m;

  const auto lane_sequence =
    route_manager.get_lanelet_sequence_on_route(forward_length, backward_length);

  auto reference_path = experimental::trajectory::build_reference_path(
    lane_sequence.as_lanelets(), route_manager.current_lanelet(), input.odometry.pose.pose,
    route_manager.lanelet_map_ptr(), route_manager.routing_graph_ptr(),
    route_manager.traffic_rules_ptr(), forward_length, backward_length);

  if (!reference_path) {
    return tl::unexpected("Failed to build reference path: " + reference_path.error());
  }

  const auto & policy = params_.reference_path.policy;
  if (policy == "goal_connection" || policy == "goal_connection_and_smooth") {
    if (
      auto connected = connect_reference_path_to_goal(
        *reference_path, input.goal_pose, lane_sequence.as_lanelets(),
        route_manager.lanelet_map_ptr(), params_.reference_path.goal_connection.search_radius_m)) {
      reference_path = std::move(*connected);
    }
  }
  if (policy == "goal_connection_and_smooth") {
    // ego 側は固定しない (ego 足元の経路は sampler が ego 姿勢から作り直すので、生の centerline に
    // 固定すると ego から離れた経路になる)。固定は goal 側だけ
    if (
      auto smoothed = smooth_reference_path(
        *reference_path, lane_sequence.as_lanelets(), input.vehicle_info.max_lateral_offset_m,
        params_.reference_path.smoother.clearance_m)) {
      reference_path = std::move(*smoothed);
    }
  }

  // goal_pose より先だけを crop する。後方 (backward_length_m 分) は残す —
  // 制約の射影が ego 後方の footprint・後方から来る物体を扱うため。
  // goal がまだ前方 (reference_path の終端より先) にある間は終端が最近傍になるので、
  // 実質「後方端から前方終端まで」になる。
  const double s_ego =
    experimental::trajectory::closest(*reference_path, input.odometry.pose.pose.position);
  const double s_goal =
    experimental::trajectory::closest(*reference_path, input.goal_pose.position);

  // closest() は弦近似なので、goal の真横 (路肩の goal 等、中心線から離れた位置) では s_ego が
  // s_goal を数 mm 追い越して見えることがある。この範囲は「goal に居る」とみなして経路を ego
  // で打ち切り (残距離 0 → 停止軌道になる)、本当に通り過ぎた場合だけ失敗にする
  constexpr double GOAL_OVERRUN_TOLERANCE_M = 1.0;
  if (s_goal - s_ego < -GOAL_OVERRUN_TOLERANCE_M) {
    return tl::unexpected(
      "goal_pose is behind ego on the reference path (length = " + std::to_string(s_goal - s_ego) +
      ")");
  }

  reference_path->crop(0.0, std::max(s_goal, s_ego));
  return std::move(reference_path.value());
}

tl::expected<SafetyPlannerResult, std::string> SafetyPlanner::plan(const SafetyPlannerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  auto reference_path = build_reference_path(input);
  if (!reference_path) {
    return tl::unexpected(reference_path.error());
  }
  const PlannerContext context(input, std::move(reference_path.value()));

  // 制約ジェネレータープラグインを呼び出して制約のリストを生成する
  auto constraints = calculate_constraints(context);

  // certainty ごとに 2 セットへ振り分ける:
  // normal = DEFINITE のみ / cautious = DEFINITE + POSSIBLE
  std::vector<Constraint> normal_list;
  std::vector<Constraint> cautious_list;
  for (const auto & [plugin_name, output] : constraints) {
    for (const auto & constraint : output.constraints) {
      if (constraint.certainty == Certainty::DEFINITE) {
        normal_list.push_back(constraint);
      }
      cautious_list.push_back(constraint);
    }
  }

  // 軌道生成はプラグインの仕事 (制約のコンパイル・rough_planner / optimizer の
  // 呼び出し方は実装詳細)
  SafetyPlannerResult result;
  if (trajectory_planner_) {
    const TrajectoryPlannerInput input{context, normal_list, cautious_list};
    auto planner_result = trajectory_planner_->plan(input);
    result.normal_trajectory = std::move(planner_result.normal_trajectory);
    result.cautious_trajectory = std::move(planner_result.cautious_trajectory);
    result.debug.compiled_constraints = std::move(planner_result.debug.compiled_constraints);
    result.debug.rough_plan_result = std::move(planner_result.debug.rough_plan_result);
    result.debug.trajectory_optimizer_result =
      std::move(planner_result.debug.trajectory_optimizer_result);
  }

  result.debug.constraint_generator_outputs = std::move(constraints);
  result.debug.reference_path = context.reference_path;
  return result;
}

std::map<std::string, ConstraintGeneratorOutput> SafetyPlanner::calculate_constraints(
  const PlannerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  std::map<std::string, ConstraintGeneratorOutput> constraints;
  for (const auto & plugin : constraint_generator_plugins_) {
    constraints.emplace(plugin->get_name(), plugin->generate_constraints(context));
  }

  return constraints;
}

}  // namespace autoware::safety_planner
