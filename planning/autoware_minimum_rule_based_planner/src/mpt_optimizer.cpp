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

#include "mpt_optimizer.hpp"

#include <autoware/acados_mpt_optimizer/utils.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <magic_enum.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <ios>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{
//! [m/s] a point at or below this speed is a stop point, not a slow one.
constexpr double STOP_VELOCITY_THRESHOLD = 1.0e-3;
//! [m] how far the optimised height may sit outside the height range of the input path.
constexpr double HEIGHT_TOLERANCE = 1.0;

/**
 * @brief Is every optimised height plausible for this input path?
 *
 * The optimiser does not model z - it carries the input path's height through - so the output must
 * stay inside the input's height range. This is checked rather than trusted because the height is
 * not inert: the post-resample step derives the pitch of every published pose from consecutive z
 * values, so a height that is off by metres produces poses that point out of the road plane, and
 * the controller follows them. That failure is silent in every other signal.
 */
bool heights_are_plausible(
  const std::vector<mpt::OptimizedPoint> & points, const TrajectoryPoints & input)
{
  auto [min_it, max_it] = std::minmax_element(
    input.begin(), input.end(),
    [](const auto & a, const auto & b) { return a.pose.position.z < b.pose.position.z; });
  const double lower = min_it->pose.position.z - HEIGHT_TOLERANCE;
  const double upper = max_it->pose.position.z + HEIGHT_TOLERANCE;
  return std::all_of(points.begin(), points.end(), [lower, upper](const auto & point) {
    return std::isfinite(point.z) && point.z >= lower && point.z <= upper;
  });
}

std::string join(const std::vector<std::string> & values)
{
  return std::accumulate(
    values.begin(), values.end(), std::string{}, [](const std::string & acc, const auto & value) {
      return acc.empty() ? value : acc + ", " + value;
    });
}
}  // namespace

mpt::OptimizerParams make_mpt_optimizer_params(const Params & params, const VehicleInfo & vehicle)
{
  const auto & source = params.acados_mpt;
  mpt::OptimizerParams result;

  result.vehicle.wheel_base = vehicle.wheel_base_m;
  result.vehicle.max_steer_angle = vehicle.max_steer_angle_rad;
  result.vehicle.front_overhang = vehicle.front_overhang_m;
  result.vehicle.rear_overhang = vehicle.rear_overhang_m;
  result.vehicle.width = vehicle.vehicle_width_m;

  // Two tiers per limit: `max` is what the NLP is constrained with, `hard_max` what makes a result
  // unusable. See acados_mpt_optimizer's Limit.
  result.limits.velocity = {source.limits.velocity.max, source.limits.velocity.hard_max};
  result.limits.acceleration = {
    source.limits.acceleration.min, source.limits.acceleration.max,
    source.limits.acceleration.hard_min, source.limits.acceleration.hard_max};
  result.limits.lateral_acceleration = {
    source.limits.lateral_acceleration.max, source.limits.lateral_acceleration.hard_max};
  result.limits.jerk = {source.limits.jerk.max, source.limits.jerk.hard_max};
  result.limits.steer_rate = {source.limits.steer_rate.max, source.limits.steer_rate.hard_max};
  result.limits.max_curvature = source.limits.max_curvature;
  result.limits.max_curvature_rate = source.limits.max_curvature_rate;
  result.limits.max_acceleration_rate = source.limits.max_acceleration_rate;

  result.weights.position = source.weights.position;
  result.weights.yaw = source.weights.yaw;
  result.weights.curvature = source.weights.curvature;
  result.weights.curvature_rate = source.weights.curvature_rate;
  result.weights.squared_speed = source.weights.squared_speed;
  result.weights.acceleration = source.weights.acceleration;
  result.weights.acceleration_rate = source.weights.acceleration_rate;
  result.weights.stretch = source.weights.stretch;
  result.weights.terminal_position = source.weights.terminal_position;
  result.weights.terminal_yaw = source.weights.terminal_yaw;
  result.weights.terminal_curvature = source.weights.terminal_curvature;
  result.weights.terminal_squared_speed = source.weights.terminal_squared_speed;
  result.weights.soft_limit = source.weights.soft_limit;

  result.stretch.min_stretch = source.stretch.min_stretch;
  result.stretch.max_stretch = source.stretch.max_stretch;
  result.stretch.max_station_error = source.stretch.max_station_error;

  result.solver.max_iterations = static_cast<int>(source.solver.max_iterations);
  result.solver.tolerance = source.solver.tolerance;
  result.solver.warm_start = source.solver.warm_start;
  result.solver.terminal_position_tolerance = source.solver.terminal_position_tolerance;
  result.solver.terminal_yaw_tolerance = source.solver.terminal_yaw_tolerance;
  result.solver.max_arc_length = source.solver.max_arc_length;
  result.solver.arc_length_per_stage_warn = source.solver.arc_length_per_stage_warn;
  result.solver.verification_tolerance = source.solver.verification_tolerance;
  result.solver.max_dynamics_residual = source.solver.max_dynamics_residual;
  result.solver.reference_speed_follows_curvature = source.solver.reference_speed_follows_curvature;
  result.solver.curvature_cap_window = source.solver.curvature_cap_window;

  // The corridor stays disabled (nothing feeds it boundaries or objects yet): this step optimises
  // the path shape and speed alone. `omit_constraint_rows` decides whether its rows are nonetheless
  // in the problem - they are not free, so it is the switch the solve-time comparison is made with.
  result.corridor.omit_constraint_rows = source.corridor.omit_constraint_rows;
  return result;
}

TrajectoryPoints crop_behind_ego(
  const TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & ego_pose)
{
  if (traj_points.empty()) {
    return traj_points;
  }
  const auto nearest = autoware::motion_utils::findNearestIndex(traj_points, ego_pose.position);
  return TrajectoryPoints{
    traj_points.begin() + static_cast<TrajectoryPoints::difference_type>(nearest),
    traj_points.end()};
}

mpt::OptimizationInput make_optimization_input(
  const TrajectoryPoints & traj_points, const Odometry & odometry, const double acceleration,
  const std::optional<double> & ego_curvature)
{
  mpt::OptimizationInput input;
  input.path.reserve(traj_points.size());
  input.velocity_limits.reserve(traj_points.size());

  double arc_length = 0.0;
  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto & point = traj_points[i];
    mpt::Pose2d pose;
    pose.x = point.pose.position.x;
    pose.y = point.pose.position.y;
    pose.z = point.pose.position.z;
    pose.yaw = tf2::getYaw(point.pose.orientation);
    input.path.push_back(pose);

    const double velocity = point.longitudinal_velocity_mps;
    input.velocity_limits.push_back(velocity);

    if (i > 0) {
      arc_length += autoware_utils::calc_distance2d(
        traj_points[i - 1].pose.position, traj_points[i].pose.position);
    }
    // The planner has already decided where to stop (modifier / map-based stop points); the first
    // zero is that station. Everything past it is zero too, which the per-point cap carries.
    if (!input.stop_arc_length && velocity <= STOP_VELOCITY_THRESHOLD) {
      input.stop_arc_length = arc_length;
    }
  }

  input.ego.pose.x = odometry.pose.pose.position.x;
  input.ego.pose.y = odometry.pose.pose.position.y;
  input.ego.pose.z = odometry.pose.pose.position.z;
  input.ego.pose.yaw = tf2::getYaw(odometry.pose.pose.orientation);
  input.ego.velocity = odometry.twist.twist.linear.x;
  input.ego.acceleration = acceleration;
  input.ego.curvature = ego_curvature;

  // Mirrors the velocity smoother, which forces the last point to zero.
  input.stop_at_end = true;
  return input;
}

TrajectoryPoints to_trajectory_points(const std::vector<mpt::OptimizedPoint> & points)
{
  TrajectoryPoints traj_points;
  traj_points.reserve(points.size());
  for (const auto & point : points) {
    TrajectoryPoint traj_point;
    traj_point.pose.position.x = point.x;
    traj_point.pose.position.y = point.y;
    traj_point.pose.position.z = point.z;
    traj_point.pose.orientation = autoware_utils::create_quaternion_from_yaw(point.yaw);
    traj_point.longitudinal_velocity_mps = static_cast<float>(point.velocity);
    traj_point.acceleration_mps2 = static_cast<float>(point.acceleration);
    // The solver knows the arc length and the speed at every stage, so this is the exact time of
    // the trajectory it just produced. It freezes at the first stop point, where the elapsed time
    // stops being meaningful - the same convention the velocity smoother follows.
    traj_point.time_from_start =
      rclcpp::Duration::from_seconds(std::max(0.0, point.time_from_start));
    traj_points.push_back(traj_point);
  }
  return traj_points;
}

void MptOptimizer::build_debug_markers(
  const Odometry & odometry, const TrajectoryPoints & input,
  const std::vector<mpt::OptimizedPoint> & output)
{
  // The geometry the whole pipeline works in; the trajectory headers say the same.
  const std::string frame_id = "map";
  const auto stamp = clock_->now();

  auto make_points_marker =
    [&](const std::string & ns, const double scale, const std_msgs::msg::ColorRGBA & color) {
      auto marker = autoware_utils_visualization::create_default_marker(
        frame_id, stamp, ns, 0, visualization_msgs::msg::Marker::SPHERE_LIST,
        autoware_utils_visualization::create_marker_scale(scale, scale, scale), color);
      // Long enough to survive a cycle at 10 Hz, short enough that the markers vanish when the
      // optimiser stops running instead of freezing the last solve on screen.
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      return marker;
    };

  // Bigger and opaque: this is the pose every other quantity is measured from.
  auto ego = make_points_marker(
    "ego", 0.6, autoware_utils_visualization::create_marker_color(1.0, 1.0, 1.0, 0.9));
  ego.points.push_back(odometry.pose.pose.position);

  auto input_points = make_points_marker(
    "input", 0.2, autoware_utils_visualization::create_marker_color(0.4, 0.4, 1.0, 0.8));
  for (const auto & point : input) {
    input_points.points.push_back(point.pose.position);
  }

  auto output_points = make_points_marker(
    "output", 0.25, autoware_utils_visualization::create_marker_color(1.0, 0.5, 0.0, 0.9));
  for (const auto & point : output) {
    output_points.points.push_back(autoware_utils::create_point(point.x, point.y, point.z));
  }

  debug_markers_.markers.clear();
  debug_markers_.markers.push_back(ego);
  debug_markers_.markers.push_back(input_points);
  debug_markers_.markers.push_back(output_points);
}

void MptOptimizer::build_debug_info(
  const mpt::OptimizationInput & input, const mpt::Result & result)
{
  const auto & params = optimizer_.params();
  std::ostringstream out;
  out << std::fixed << std::setprecision(3);

  // The ego state is the *hard* initial condition of the NLP, so it belongs first: a solve that
  // looks wrong is usually pinned to something wrong.
  out << "ego: v " << input.ego.velocity << " m/s, a " << input.ego.acceleration << " m/s^2";
  if (input.ego.curvature) {
    const double steer = std::atan(*input.ego.curvature * params.vehicle.wheel_base);
    out << ", kappa0 " << *input.ego.curvature << " 1/m (steer "
        << autoware_utils_math::rad2deg(steer) << " deg, measured)";
    // The lateral acceleration the vehicle is *already* pulling. Stage 0 of the NLP is pinned to
    // this state and carries no nonlinear constraint, so when this exceeds the limit the report
    // says so and no solve can do anything about it - that is the usual explanation for a lateral
    // acceleration violation that persists for a whole curve. See ConstraintReport::WorstCase.
    const double ego_lateral_acceleration =
      input.ego.velocity * input.ego.velocity * *input.ego.curvature;
    out << ", a_lat " << ego_lateral_acceleration << " m/s^2 (pinned"
        << (std::abs(ego_lateral_acceleration) > params.limits.lateral_acceleration.max
              ? ", ALREADY OVER THE LIMIT"
              : "")
        << ")";
  } else {
    out << ", kappa0 estimated from the path (unreliable; see debug_info docs)";
  }
  out << "\n";

  out << "input: " << input.path.size() << " points, " << mpt::utils::total_arc_length(input.path)
      << " m";
  if (input.stop_arc_length) {
    out << ", stop at " << *input.stop_arc_length << " m";
  } else {
    out << ", no stop point";
  }
  out << ", stop_at_end " << (input.stop_at_end ? "true" : "false") << "\n";

  // What the input itself asks for: its own curvature times the speed caps it carries. When this is
  // already over the limit, the NLP can only obey by slowing down below the cap, which takes
  // min_acceleration metres to do - so a violation that decays over a curve is the *input* being
  // infeasible for the limit, not the solver ignoring it. Estimated from the raw input points, so
  // it is noisier than the solver's own curvature state.
  {
    const auto curvatures = mpt::utils::compute_curvatures(input.path);
    const auto stations = mpt::utils::compute_arc_lengths(input.path);
    double worst = 0.0;
    std::size_t worst_index = 0;
    for (size_t i = 0; i < input.path.size() && i < curvatures.size(); ++i) {
      const double cap =
        i < input.velocity_limits.size() ? input.velocity_limits[i] : params.limits.velocity.max;
      const double demanded = std::abs(cap * cap * curvatures[i]);
      if (demanded > worst) {
        worst = demanded;
        worst_index = i;
      }
    }
    out << "input demand: worst a_lat " << worst << " m/s^2 at " << stations[worst_index]
        << " m (cap " << input.velocity_limits[worst_index] << " m/s, kappa "
        << curvatures[worst_index] << " 1/m), nominal limit "
        << params.limits.lateral_acceleration.max << "\n";
  }

  out << "solver: " << magic_enum::enum_name(result.status) << " in " << std::setprecision(1)
      << result.solve_time_s * 1000.0 << " ms, " << result.sqp_iterations << " SQP iterations"
      << std::setprecision(3) << ", acados " << result.solver_status << ", qp " << result.qp_status
      << ", corridor rows " << (params.corridor.omit_constraint_rows ? "absent" : "present")
      << "\n";
  out << "residuals: stat " << std::scientific << std::setprecision(2)
      << result.residual_stationarity << ", eq " << result.residual_equality << ", ineq "
      << result.residual_inequality << ", comp " << result.residual_complementarity << ", kkt "
      << result.kkt_norm << std::fixed << std::setprecision(3) << "\n";
  out << "grid: " << result.station_step << " m per stage, output " << result.output_arc_length
      << " m over " << result.points.size() << " points, stretch [" << result.report.min_stretch
      << ", " << result.report.max_stretch << "]"
      << (result.input_path_cropped ? ", input cropped to the horizon" : "") << "\n";

  // Worst case against the limit, so a number that is about to bind is visible before it does.
  const auto & report = result.report;
  out << "limits (worst / nominal / hard): v " << report.max_velocity << " / "
      << params.limits.velocity.max << " / " << params.limits.velocity.hard() << ", a ["
      << report.min_acceleration << ", " << report.max_acceleration << "] / ["
      << params.limits.acceleration.min << ", " << params.limits.acceleration.max << "] / ["
      << params.limits.acceleration.hard_lower() << ", " << params.limits.acceleration.hard_upper()
      << "], a_lat " << report.max_abs_lateral_acceleration << " (stages 1+, ego " << std::showpos
      << report.ego_lateral_acceleration << std::noshowpos << ") / "
      << params.limits.lateral_acceleration.max << " / "
      << params.limits.lateral_acceleration.hard() << ", jerk " << report.max_abs_jerk << " / "
      << params.limits.jerk.max << " / " << params.limits.jerk.hard() << ", steer_rate "
      << report.max_abs_steer_rate << " / " << params.limits.steer_rate.max << " / "
      << params.limits.steer_rate.hard() << ", kappa " << report.max_abs_curvature << " / "
      << mpt::utils::effective_curvature_limit(params) << " (single tier)\n";
  // Where each of those worst cases sits. Stage 0 is the ego's own motion; a violation spanning
  // many stages is the solver failing to respect the limit over the horizon.
  const auto where = [](const mpt::ConstraintReport::WorstCase & worst, const std::size_t stages) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(2) << "stage " << worst.index << " (s "
         << worst.arc_length << " m, v " << worst.velocity << " m/s, kappa " << std::setprecision(4)
         << worst.curvature << " 1/m, " << worst.violating_stages << "/" << stages << " over)";
    return text.str();
  };
  const std::size_t stages = result.points.size();
  out << "worst at: a_lat " << where(report.worst_lateral_acceleration, stages) << ", jerk "
      << where(report.worst_jerk, stages) << ", steer_rate "
      << where(report.worst_steer_rate, stages) << ", kappa "
      << where(report.worst_curvature, stages) << "\n";
  out << "terminal: position error " << report.terminal_position_error << " m, yaw error "
      << report.terminal_yaw_error << " rad, v " << report.terminal_velocity << " m/s\n";
  out << "station error: " << report.max_station_error << " / " << params.stretch.max_station_error
      << " m\n";
  // The nominal tier is slacked by design, so an excess there is news but not a rejection; keep the
  // two on separate lines so `grep "verdict: LIMIT VIOLATION"` only catches the real failures.
  if (!report.nominal_exceedances.empty()) {
    out << "nominal exceeded (reported, not fatal): " << join(report.nominal_exceedances) << "\n";
  }
  out << "verdict: " << (report.satisfied ? "every hard limit satisfied" : "LIMIT VIOLATION");
  if (!report.violations.empty()) {
    out << " (" << join(report.violations) << ")";
  }
  if (!result.message.empty()) {
    out << " | " << result.message;
  }
  out << "\n";

  debug_info_ = out.str();
}

MptOptimizer::MptOptimizer(
  const mpt::OptimizerParams & params, const EngageParams & engage, const rclcpp::Logger & logger,
  rclcpp::Clock::SharedPtr clock, std::shared_ptr<VelocitySmoother> velocity_smoother)
: optimizer_(params, logger, clock),
  velocity_smoother_(std::move(velocity_smoother)),
  engage_(engage),
  logger_(logger),
  clock_(std::move(clock))
{
}

std::vector<double> MptOptimizer::smoothed_guess_velocities(
  const TrajectoryPoints & traj_points, const Odometry & odometry, const double acceleration) const
{
  if (!velocity_smoother_ || traj_points.size() < 2) {
    return {};
  }
  const auto & params = optimizer_.params();

  // Cap the speeds by the path's own curvature first: that is the part of the profile the NLP's
  // reference gets right and the smoother's lateral filter does not (its floor is 2.74 m/s).
  mpt::Path shape;
  shape.reserve(traj_points.size());
  for (const auto & point : traj_points) {
    mpt::Pose2d pose;
    pose.x = point.pose.position.x;
    pose.y = point.pose.position.y;
    shape.push_back(pose);
  }
  const auto stations = mpt::utils::compute_arc_lengths(shape);
  const double spacing =
    stations.back() / std::max<double>(1.0, static_cast<double>(shape.size() - 1));
  const auto curvatures = mpt::utils::smooth_absolute_curvatures(
    mpt::utils::compute_curvatures(shape), spacing, params.solver.curvature_cap_window);

  auto capped = traj_points;
  for (size_t i = 0; i < capped.size(); ++i) {
    if (curvatures[i] < 1.0e-9) continue;
    const double cap = std::sqrt(params.limits.lateral_acceleration.max / curvatures[i]);
    capped[i].longitudinal_velocity_mps =
      std::min(capped[i].longitudinal_velocity_mps, static_cast<float>(cap));
  }

  // The smoother resamples and clips, so its result lives on its own stations; map it back.
  velocity_smoother_->optimize(capped, odometry, acceleration, /*update_prev_output=*/false);
  if (capped.size() < 2) {
    return {};
  }
  mpt::Path smoothed_shape;
  smoothed_shape.reserve(capped.size());
  for (const auto & point : capped) {
    mpt::Pose2d pose;
    pose.x = point.pose.position.x;
    pose.y = point.pose.position.y;
    smoothed_shape.push_back(pose);
  }
  const auto smoothed_stations = mpt::utils::compute_arc_lengths(smoothed_shape);
  std::vector<double> guess;
  guess.reserve(traj_points.size());
  for (const double station : stations) {
    const auto upper =
      std::lower_bound(smoothed_stations.begin(), smoothed_stations.end(), station);
    if (upper == smoothed_stations.begin()) {
      guess.push_back(capped.front().longitudinal_velocity_mps);
      continue;
    }
    if (upper == smoothed_stations.end()) {
      guess.push_back(capped.back().longitudinal_velocity_mps);
      continue;
    }
    const auto index = static_cast<size_t>(std::distance(smoothed_stations.begin(), upper));
    const double span = smoothed_stations[index] - smoothed_stations[index - 1];
    const double ratio = span < 1.0e-9 ? 0.0 : (station - smoothed_stations[index - 1]) / span;
    const double previous = capped[index - 1].longitudinal_velocity_mps;
    guess.push_back(previous + ratio * (capped[index].longitudinal_velocity_mps - previous));
  }
  return guess;
}

void MptOptimizer::apply_engage_speed(
  TrajectoryPoints & traj_points, const TrajectoryPoints & input, const Odometry & odometry) const
{
  if (!engage_.enable || traj_points.empty() || input.size() < 2) return;
  if (odometry.twist.twist.linear.x >= engage_.speed) return;

  // Where to stop is decided on the *input*: the optimised profile begins at zero simply because
  // the vehicle is standing still, and reading that as a stop point would mean never departing.
  double stop_distance = std::numeric_limits<double>::infinity();
  const auto closest = autoware::motion_utils::findNearestIndex(input, odometry.pose.pose.position);
  if (
    const auto stop_index =
      autoware::motion_utils::searchZeroVelocityIndex(input, closest, input.size())) {
    stop_distance = autoware::motion_utils::calcSignedArcLength(input, closest, *stop_index);
    // Something wants the vehicle stopped right here; departing would drive through it.
    if (stop_distance <= engage_.stop_dist_to_prohibit_engage) return;
  }

  // Only up to the stop point. The velocity smoother raises every point instead, which erases the
  // stop points the modifiers wrote and is the known cause of creeping past a stopped vehicle.
  double travelled = 0.0;
  for (size_t i = 0; i < traj_points.size(); ++i) {
    if (i > 0) {
      travelled += autoware_utils::calc_distance2d(
        traj_points[i - 1].pose.position, traj_points[i].pose.position);
    }
    if (travelled >= stop_distance) break;
    traj_points[i].longitudinal_velocity_mps =
      std::max(traj_points[i].longitudinal_velocity_mps, static_cast<float>(engage_.speed));
    traj_points[i].acceleration_mps2 =
      std::max(traj_points[i].acceleration_mps2, static_cast<float>(engage_.acceleration));
  }
}

void MptOptimizer::update_params(const mpt::OptimizerParams & params)
{
  optimizer_.update_params(params);
}

std::optional<TrajectoryPoints> MptOptimizer::optimize(
  const TrajectoryPoints & traj_points, const Odometry & odometry, const double acceleration,
  const std::optional<double> & ego_curvature)
{
  // Rebuilt on every call, so nothing on screen outlives the solve it came from. An early return
  // below leaves the namespaces it did not reach empty rather than stale.
  build_debug_markers(odometry, {}, {});
  const auto not_solved = [this](const std::string & reason) {
    debug_info_ = "solver: not run (" + reason + ")\n";
  };
  if (traj_points.size() < 2) {
    not_solved("the trajectory holds fewer than two points");
    return std::nullopt;
  }
  // Everything the NLP is told about is measured from here on: the path it optimises, the per-point
  // caps, the stop station and the engage distance.
  const auto forward_points = crop_behind_ego(traj_points, odometry.pose.pose);
  if (forward_points.size() < 2) {
    not_solved("the ego sits at the end of the trajectory");
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[mpt] the ego sits at the end of the trajectory (%zu of %zu points ahead); falling back.",
      forward_points.size(), traj_points.size());
    return std::nullopt;
  }
  // b = v^2 cannot express a direction, so a reverse trajectory would come back as a forward one.
  const bool has_reverse = std::any_of(
    forward_points.begin(), forward_points.end(),
    [](const auto & p) { return p.longitudinal_velocity_mps < 0.0F; });
  if (has_reverse) {
    not_solved("the trajectory reverses, which b = v^2 cannot express");
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "[mpt] reverse trajectory is not supported; falling back.");
    return std::nullopt;
  }

  auto input = make_optimization_input(forward_points, odometry, acceleration, ego_curvature);
  input.guess_velocities = smoothed_guess_velocities(forward_points, odometry, acceleration);
  const auto result = optimizer_.optimize(input);

  // Before the usability checks: a rejected solve is exactly the one worth looking at, and the
  // points are the last SQP iterate (the library returns them for this purpose).
  build_debug_markers(odometry, forward_points, result.points);
  build_debug_info(input, result);

  if (!result.usable()) {
    const std::string violations =
      result.report.violations.empty() ? "" : " violations: " + join(result.report.violations);
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "[mpt] unusable result (%s): %s%s. Falling back.",
      std::string(magic_enum::enum_name(result.status)).c_str(), result.message.c_str(),
      violations.c_str());
    return std::nullopt;
  }
  if (!heights_are_plausible(result.points, forward_points)) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "[mpt] the optimised height left the input path's range (first point %.2f m); the pose pitch "
      "downstream would be meaningless. Falling back.",
      result.points.empty() ? 0.0 : result.points.front().z);
    return std::nullopt;
  }
  if (result.status == mpt::ResultStatus::NOT_CONVERGED) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[mpt] feasible but not converged after %d iterations (kkt %.3e); the horizon was hard.",
      result.sqp_iterations, result.kkt_norm);
  }
  RCLCPP_DEBUG(
    logger_, "[mpt] solved in %.1f ms, %d SQP iterations, %.1f m output.",
    result.solve_time_s * 1000.0, result.sqp_iterations, result.output_arc_length);

  auto optimized = to_trajectory_points(result.points);
  apply_engage_speed(optimized, forward_points, odometry);
  return optimized;
}

}  // namespace autoware::minimum_rule_based_planner
