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

#include "utils/frenet_utils.hpp"
#include "utils/reference_path_smoother.hpp"
#include "utils/trajectory_postprocess.hpp"

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/trajectory/threshold.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware/trajectory/utils/crop.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware/trajectory/utils/reference_path.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>

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

std::optional<PathPointTrajectory> connect_reference_path_to_goal(
  const PathPointTrajectory & path, const Pose & goal,
  const lanelet::ConstLanelets & route_lanelets, const lanelet::LaneletMapConstPtr & lanelet_map,
  const double search_radius_m, const double shift_length_m, const double max_curvature,
  const double vehicle_half_width_m)
{
  using autoware_utils_geometry::calc_distance2d;
  //! [m] spacing at which the shifted stretch is resampled and checked against the lanes
  constexpr double SHIFT_STEP_M = 1.0;
  //! [rad] beyond this the shift cannot express the goal heading: the path would have to double
  //! back across the lane, which is not a goal connection any more
  constexpr double MAX_GOAL_YAW_OFFSET_RAD = M_PI / 3.0;
  //! [rad] heading error at the goal the correction passes aim for, and above which the ramp
  //! length is rejected and a shorter one tried. Not tightened further: the elastic band that
  //! follows reshapes the connection and adds a few tenths of a degree of its own anyway
  constexpr double GOAL_YAW_TOLERANCE_RAD = 0.02;
  //! [m] the shortest ramp tried; below this the connection is a corner, not a shift
  constexpr double MIN_SHIFT_LENGTH_M = 5.0;
  //! how often the built path is measured and the ramp corrected on what it missed
  constexpr int NUM_PASSES = 4;

  const double s_goal = experimental::trajectory::closest(path, goal.position);
  if (calc_distance2d(path.compute(s_goal).point.pose, goal) > search_radius_m) {
    return std::nullopt;  // the goal is outside the window ahead, or too far off to the side
  }

  // The centerline already arrives at the goal with (almost) its heading; what separates the two
  // is the lateral offset of the goal. So the shape of the centerline is kept and only that offset
  // is taken up, by a quintic lateral ramp ending at the goal. Why not a spline through the goal,
  // as goal_planner does: it has to represent the whole corner of a curved lane with one cubic,
  // which swings past the goal heading and back, and in a rotary reaches a curvature the steering
  // cannot follow
  const double l_goal = lateral_offset_at(path, s_goal, Point2d{goal.position.x, goal.position.y});
  const double goal_yaw = autoware_utils_geometry::get_rpy(goal).z;
  const double goal_yaw_offset =
    autoware_utils_math::normalize_radian(goal_yaw - path.azimuth(s_goal));
  if (std::abs(goal_yaw_offset) > MAX_GOAL_YAW_OFFSET_RAD) {
    return std::nullopt;
  }
  // The tangent of the shifted path is (1 - k l) t + l' n, so the heading it reaches at the goal
  // is azimuth + atan(l' / (1 - k l)), not azimuth + atan(l'). On a tight lane 1 - k l is far from
  // 1 (~2 in a rotary) and leaving the factor out turns the path only half as far as the goal asks
  const double scale_at_goal = 1.0 - path.curvature(s_goal) * l_goal;
  const auto original = path.restore();
  const auto bases = path.get_underlying_bases();

  // Road lanelets off the route are not accepted: a goal in the neighboring lane would otherwise
  // be connected, i.e. a lane change, which is left to the driver / the other planner
  const auto is_inside_route_or_shoulder_lane = [&](const geometry_msgs::msg::Pose & pose) {
    const lanelet::BasicPoint2d p{pose.position.x, pose.position.y};
    for (const auto & lanelet : route_lanelets) {
      if (lanelet::geometry::inside(lanelet, p)) {
        return true;
      }
    }
    return !experimental::lanelet2_utils::get_shoulder_lanelets_at(lanelet_map, p.x(), p.y())
              .empty();
  };

  //! The connection with the shift spread over shift_length, or nullopt when it leaves the lane or
  //! does not arrive at the goal heading
  const auto attempt = [&](const double shift_length) -> std::optional<PathPointTrajectory> {
    // The ramp is anchored on the goal, so the connection keeps the same shape from cycle to
    // cycle. Once the window starts inside the ramp it is entered partway, which is where the ego
    // already is; squeezing the whole shift into what is left of the window instead would tighten
    // the curve every cycle as the goal comes closer
    const double s_ramp = s_goal - shift_length;
    const double s_begin = std::max(0.0, s_ramp);
    if (s_goal - s_begin < SHIFT_STEP_M) {
      return std::nullopt;  // no room to shift: the goal is at the very start of the window
    }
    const auto n = static_cast<std::size_t>(std::ceil((s_goal - s_begin) / SHIFT_STEP_M));
    const double ds = (s_goal - s_begin) / static_cast<double>(n);
    std::vector<double> arc_lengths;
    std::vector<double> curvature;
    std::vector<double> offset_min;
    std::vector<double> offset_max;
    for (std::size_t i = 0; i <= n; ++i) {
      const double s = std::min(s_begin + static_cast<double>(i) * ds, s_goal);
      arc_lengths.push_back(s);
      curvature.push_back(path.curvature(s));
      // The room the lane leaves around the centerline. Both 0 and l_goal are always admitted:
      // the centerline and the goal are where the connection has to start and end, whatever the
      // lane says about them (a goal on a shoulder projects outside the room)
      const auto [room_right, room_left] =
        lateral_room(path.compute(s).point.pose.position, route_lanelets, vehicle_half_width_m);
      offset_min.push_back(std::min({-room_right, 0.0, l_goal}));
      offset_max.push_back(std::max({room_left, 0.0, l_goal}));
    }

    // l(u) with u = (s - s_ramp) / shift_length, joining the centerline with matched value, slope
    // and curvature and reaching (l_goal, dl_goal, straight) at the goal
    const auto ramp = [&](const double dl_goal) {
      const double c3 = 10.0 * l_goal - 4.0 * dl_goal;
      const double c4 = -15.0 * l_goal + 7.0 * dl_goal;
      const double c5 = 6.0 * l_goal - 3.0 * dl_goal;
      std::vector<double> profile;
      profile.reserve(n + 1);
      for (const double s : arc_lengths) {
        const double u = (s - s_ramp) / shift_length;
        profile.push_back(u * u * u * (c3 + u * (c4 + u * c5)));
      }
      return profile;
    };

    const auto build = [&](const std::vector<double> & profile) {
      std::vector<PathPointWithLaneId> connected;
      for (std::size_t i = 0; i < original.size() && bases[i] < s_begin; ++i) {
        connected.push_back(original[i]);
      }
      // s_begin itself is resampled rather than taken from the original points: when the window
      // starts inside the ramp it already carries a lateral offset
      const std::size_t first_shifted = connected.size();
      for (std::size_t i = 0; i <= n; ++i) {
        const auto shifted = to_world_pose(path, arc_lengths[i], profile[i]);
        auto point = path.compute(arc_lengths[i]);
        point.point.pose.position.x = shifted.position.x();
        point.point.pose.position.y = shifted.position.y();
        connected.push_back(point);
      }
      // The headings have to describe the shifted positions, not the centerline they came from
      for (std::size_t i = std::max<std::size_t>(first_shifted, 1); i < connected.size(); ++i) {
        const auto & p0 = connected[i - 1].point.pose.position;
        const auto & p1 = connected[i + 1 < connected.size() ? i + 1 : i].point.pose.position;
        connected[i].point.pose.orientation =
          autoware_utils_geometry::create_quaternion_from_yaw(std::atan2(p1.y - p0.y, p1.x - p0.x));
      }
      connected.back().point.pose.position.x = goal.position.x;
      connected.back().point.pose.position.y = goal.position.y;
      connected.back().point.pose.orientation = goal.orientation;
      return experimental::trajectory::pretty_build(connected);
    };

    // Two things are only known once the path is built: the ramp can exceed the curvature the
    // steering can hold (the lane it lies on already does, in a rotary, and the shift adds to it),
    // and the spline built through the samples does not arrive at exactly the heading their slope
    // asks for. Both are corrected on what the built path measures: the curvature bound handed to
    // the QP is tightened by the ratio it missed, the terminal slope of the ramp by the heading it
    // missed. The curvature is best effort - where the lane itself is tighter than the steering
    // limit there is no solution and the QP says so - while the heading decides acceptance
    double dl_goal = shift_length * scale_at_goal * std::tan(goal_yaw_offset);
    double limit = max_curvature;
    std::optional<PathPointTrajectory> built;
    for (int pass = 0; pass < NUM_PASSES; ++pass) {
      auto offsets = ramp(dl_goal);
      const auto optimized =
        optimize_goal_shift(offsets, curvature, ds, limit, offset_min, offset_max);
      if (optimized) {
        offsets = *optimized;
      }
      built = build(offsets);
      if (!built) {
        return std::nullopt;
      }
      double curvature_max = 0.0;
      for (double s = s_begin; s <= built->length(); s += SHIFT_STEP_M) {
        curvature_max = std::max(curvature_max, std::abs(built->curvature(s)));
      }
      const double yaw_error =
        autoware_utils_math::normalize_radian(goal_yaw - built->azimuth(built->length()));
      if (
        (curvature_max <= max_curvature && std::abs(yaw_error) < GOAL_YAW_TOLERANCE_RAD) ||
        pass + 1 == NUM_PASSES) {
        break;
      }
      if (curvature_max > max_curvature) {
        limit *= max_curvature / curvature_max;
      }
      dl_goal += shift_length * scale_at_goal * std::tan(yaw_error);
    }

    if (
      std::abs(autoware_utils_math::normalize_radian(goal_yaw - built->azimuth(built->length()))) >
      GOAL_YAW_TOLERANCE_RAD) {
      return std::nullopt;
    }
    for (double s = s_begin; s <= built->length(); s += SHIFT_STEP_M) {
      if (!is_inside_route_or_shoulder_lane(built->compute(s).point.pose)) {
        return std::nullopt;
      }
    }
    return built;
  };

  // A long ramp keeps the curvature down, but the further the goal heading is from the lane it
  // sits in, the more the ramp has to swing out before turning back to reach it, and outside the
  // lane bound there is nothing to be had. So the longest ramp that works is taken
  for (double shift_length = shift_length_m; shift_length >= MIN_SHIFT_LENGTH_M;
       shift_length *= 0.5) {
    if (auto built = attempt(shift_length)) {
      return built;
    }
  }
  return std::nullopt;
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

  const auto & window_lanelets = lane_sequence.as_lanelets();

  // build_reference_path() extends the path beyond lane_sequence with the whole routing graph, so
  // when ego's lane leaves the route before a lane change the tail is off-route. It is cut by the
  // lane_ids rather than by projecting the route end onto the path: a point outside the window
  // projects onto whichever end is geometrically closer, which may be the start behind ego.
  {
    const auto points = reference_path->restore();
    const auto bases = reference_path->get_underlying_bases();
    const auto is_on_route = [&](const PathPointWithLaneId & point) {
      return std::any_of(
        point.lane_ids.begin(), point.lane_ids.end(), [&](const lanelet::Id lane_id) {
          return std::any_of(
            window_lanelets.begin(), window_lanelets.end(),
            [&](const auto & lanelet) { return lanelet.id() == lane_id; });
        });
    };
    std::size_t last_on_route = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (is_on_route(points[i])) {
        last_on_route = i;
      }
    }
    const double s_ego =
      experimental::trajectory::closest(*reference_path, input.odometry.pose.pose.position);
    reference_path->crop(0.0, std::max(bases[last_on_route], s_ego));
  }

  // The goal is handled only once its lanelet is inside the window (or beside it, for a goal that
  // needs a lane change): projecting a goal that lies beyond the window is meaningless for the
  // same reason as above, and on a looping route it would even attach the path to a goal behind.
  // The lanelet alone is not enough since it may be longer than the remaining window, so the
  // projection is also required to land near the goal (see below).
  const auto goal_lanelet = route_manager.get_closest_preferred_route_lanelet(input.goal_pose);
  const bool goal_in_window =
    goal_lanelet &&
    std::any_of(window_lanelets.begin(), window_lanelets.end(), [&](const auto & lanelet) {
      return lanelet.id() == goal_lanelet->id() ||
             lanelet.leftBound().id() == goal_lanelet->rightBound().id() ||
             lanelet.rightBound().id() == goal_lanelet->leftBound().id();
    });

  const auto & policy = params_.reference_path.policy;
  if (goal_in_window && (policy == "goal_connection" || policy == "goal_connection_and_smooth")) {
    if (
      auto connected = connect_reference_path_to_goal(
        *reference_path, input.goal_pose, window_lanelets, route_manager.lanelet_map_ptr(),
        params_.reference_path.goal_connection.search_radius_m,
        params_.reference_path.goal_connection.shift_length_m,
        params_.reference_path.goal_connection.curvature_margin *
          std::tan(input.vehicle_info.max_steer_angle_rad) / input.vehicle_info.wheel_base_m,
        input.vehicle_info.max_lateral_offset_m)) {
      reference_path = std::move(*connected);
    }
  }
  if (policy == "goal_connection_and_smooth") {
    if (
      auto smoothed = smooth_reference_path(
        *reference_path, window_lanelets, input.vehicle_info.max_lateral_offset_m,
        params_.reference_path.smoother.clearance_m)) {
      reference_path = std::move(*smoothed);
    }
  }

  if (!goal_in_window) {
    return std::move(reference_path.value());
  }

  const double s_ego =
    experimental::trajectory::closest(*reference_path, input.odometry.pose.pose.position);
  const double s_goal =
    experimental::trajectory::closest(*reference_path, input.goal_pose.position);
  if (
    autoware_utils_geometry::calc_distance2d(
      reference_path->compute(s_goal).point.pose, input.goal_pose) >
    params_.reference_path.goal_connection.search_radius_m) {
    return std::move(reference_path.value());
  }

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

  auto constraints = calculate_constraints(context);

  // Split by certainty: normal = DEFINITE only, cautious = DEFINITE + POSSIBLE
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

  SafetyPlannerResult result;
  if (trajectory_planner_) {
    const TrajectoryPlannerInput input{context, normal_list, cautious_list};
    auto planner_result = trajectory_planner_->plan_trajectories(input);
    if (params_.engage_velocity.enable) {
      if (planner_result.normal_trajectory) {
        auto & trajectory = planner_result.normal_trajectory->trajectory;
        trajectory = set_engage_speed(trajectory, params_.engage_velocity.velocity_hard_mps);
      }
      if (planner_result.cautious_trajectory) {
        auto & trajectory = planner_result.cautious_trajectory->trajectory;
        trajectory = set_engage_speed(trajectory, params_.engage_velocity.velocity_hard_mps);
      }
    }
    result.normal_trajectory = std::move(planner_result.normal_trajectory);
    result.cautious_trajectory = std::move(planner_result.cautious_trajectory);
    result.debug.normal = std::move(planner_result.normal_debug);
    result.debug.cautious = std::move(planner_result.cautious_debug);
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
