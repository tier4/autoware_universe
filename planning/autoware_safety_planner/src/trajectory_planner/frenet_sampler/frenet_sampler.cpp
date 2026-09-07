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

#include "frenet_sampler.hpp"

#include "../../utils/trajectory_conversion.hpp"

#include <autoware_frenet_planner/polynomials.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

//! 弧長 s で有効な VELOCITY 上限 (全域 + region 限定の両方)
double velocity_limit_at(
  const CompiledConstraints & compiled_constraints, const KinematicLimits & limits, const double s)
{
  double v_max = limits.v_hard;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (bound.quantity == BoundedQuantity::VELOCITY && bound.s0 <= s && s <= bound.s1) {
      v_max = std::min(v_max, bound.max);
    }
  }
  return v_max;
}

//! 全域 (region 無し) の ScalarBound の上限を量ごとに集める (collect_kinematic_limits が
//! 読まない LAT_ACCEL / LON_JERK / STEER_ANGLE / STEER_RATE / CURVATURE 用)
struct GlobalBounds
{
  double lat_accel{INF};
  double lon_jerk{INF};
  double curvature{INF};
  double steer_angle{INF};
  double steer_rate{INF};
};

GlobalBounds collect_global_bounds(const CompiledConstraints & compiled_constraints)
{
  GlobalBounds bounds;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (!(bound.s0 == -INF && bound.s1 == INF)) {
      continue;
    }
    switch (bound.quantity) {
      case BoundedQuantity::LAT_ACCEL:
        bounds.lat_accel = std::min(bounds.lat_accel, bound.max);
        break;
      case BoundedQuantity::LON_JERK:
        bounds.lon_jerk = std::min(bounds.lon_jerk, bound.max);
        break;
      case BoundedQuantity::CURVATURE:
        bounds.curvature = std::min(bounds.curvature, bound.max);
        break;
      case BoundedQuantity::STEER_ANGLE:
        bounds.steer_angle = std::min(bounds.steer_angle, bound.max);
        break;
      case BoundedQuantity::STEER_RATE:
        bounds.steer_rate = std::min(bounds.steer_rate, bound.max);
        break;
      default:
        break;  // VELOCITY / LON_ACCEL は KinematicLimits 側
    }
  }
  return bounds;
}

//! 等間隔サンプル列 (先頭 s0・間隔 res) の線形補間。範囲外は端でクランプ
double interpolate_uniform(
  const std::vector<double> & values, const double s0, const double res, const double s)
{
  if (values.size() < 2) {
    return values.front();
  }
  const double u = std::clamp((s - s0) / res, 0.0, static_cast<double>(values.size() - 1));
  const auto i = std::min(static_cast<std::size_t>(u), values.size() - 2);
  const double r = u - static_cast<double>(i);
  return values[i] * (1.0 - r) + values[i + 1] * r;
}

//! yaw 列の線形補間 (2π の巻き込みを考慮)
double interpolate_uniform_angle(
  const std::vector<double> & yaws, const double s0, const double res, const double s)
{
  if (yaws.size() < 2) {
    return yaws.front();
  }
  const double u = std::clamp((s - s0) / res, 0.0, static_cast<double>(yaws.size() - 1));
  const auto i = std::min(static_cast<std::size_t>(u), yaws.size() - 2);
  const double r = u - static_cast<double>(i);
  const double d = autoware_utils_math::normalize_radian(yaws[i + 1] - yaws[i]);
  return autoware_utils_math::normalize_radian(yaws[i] + d * r);
}

}  // namespace

TrajectoryPlannerResult FrenetSamplingBasedPlanner::plan(const TrajectoryPlannerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  TrajectoryPlannerResult result;
  {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_normal", *time_keeper_);
    auto compiled = compile_constraint_list(input.context, input.normal_constraints);
    // 候補の可視化は rough_plan 用のマーカー配信に載せる (Node 側の配信経路を増やさない)
    result.normal_trajectory =
      plan_one_side(input.context, compiled, result.debug.rough_plan_result.debug.debug_markers);
    result.debug.compiled_constraints = std::move(compiled);
  }
  {
    autoware_utils_debug::ScopedTimeTrack side_st("plan_cautious", *time_keeper_);
    const auto compiled = compile_constraint_list(input.context, input.cautious_constraints);
    MarkerArray unused_markers;
    result.cautious_trajectory = plan_one_side(input.context, compiled, unused_markers);
  }
  return result;
}

std::optional<Trajectory> FrenetSamplingBasedPlanner::plan_one_side(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  MarkerArray & debug_markers) const
{
  const auto initial_state = compute_initial_state(context);
  const auto paths = generate_paths(context, initial_state);
  const auto profiles = generate_velocity_profiles(context, initial_state, compiled_constraints);

  std::vector<Candidate> candidates;
  candidates.reserve(paths.size() * profiles.size());
  for (const auto & path : paths) {
    for (const auto & profile : profiles) {
      candidates.push_back(combine(context, path, profile));
      evaluate(context, compiled_constraints, candidates.back());
    }
  }
  append_debug_markers(context, candidates, debug_markers);

  const Candidate * best = nullptr;
  for (const auto & candidate : candidates) {
    if (candidate.valid && (!best || candidate.cost < best->cost)) {
      best = &candidate;
    }
  }

  if (!best) {
    std::map<std::string, int> reasons;
    for (const auto & candidate : candidates) {
      const auto pos = candidate.tag.rfind('[');
      reasons[pos == std::string::npos ? "?" : candidate.tag.substr(pos)]++;
    }
    std::stringstream ss;
    for (const auto & [reason, count] : reasons) {
      ss << " " << reason << "x" << count;
    }
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("safety_planner"), steady_clock, 5000,
      "[frenet_sampler] no valid candidate (%zu sampled:%s). Falling back to the stop trajectory.",
      candidates.size(), ss.str().c_str());
    // 停止経路 = 現在の横位置を保つ直進 (経路候補の先頭の s 列を借りて l を l0 に固定する)
    PathCandidate straight = paths.front();
    std::fill(straight.l.begin(), straight.l.end(), initial_state.l);
    const auto stop =
      make_stop_profile(initial_state, collect_kinematic_limits(compiled_constraints));
    return to_trajectory_msg(context, combine(context, straight, stop));
  }
  return to_trajectory_msg(context, *best);
}

FrenetSamplingBasedPlanner::InitialState FrenetSamplingBasedPlanner::compute_initial_state(
  const PlannerContext & context) const
{
  const auto ego = compute_ego_frenet_state(context);
  const auto & path = context.reference_path;
  const double ego_yaw = autoware_utils_geometry::get_rpy(context.odometry.pose.pose).z;
  const double frenet_yaw = autoware_utils_math::normalize_radian(ego_yaw - path.azimuth(ego.s));
  const double v = context.odometry.twist.twist.linear.x;

  InitialState state;
  state.s = ego.s;
  state.l = ego.l;
  // 中心線に対して ±90° 近くの向きは tan が発散するので、勾配は ±60° 相当で打ち切る
  state.dl_ds = std::tan(std::clamp(frenet_yaw, -M_PI / 3.0, M_PI / 3.0));
  state.v = std::max(0.0, v * std::cos(frenet_yaw));
  state.a = context.acceleration.accel.accel.linear.x;
  return state;
}

std::vector<FrenetSamplingBasedPlanner::PathCandidate> FrenetSamplingBasedPlanner::generate_paths(
  const PlannerContext & context, const InitialState & initial_state) const
{
  using autoware::frenet_planner::Polynomial;

  const auto & p = params_.frenet_sampler;
  const auto & ref = context.reference_path;
  const double res = p.path_resolution_m;
  const double s0 = initial_state.s;
  const double s_max = ref.length();
  const double ego_yaw = autoware_utils_geometry::get_rpy(context.odometry.pose.pose).z;

  std::vector<PathCandidate> paths;
  for (const double length : p.target_lengths_m) {
    for (const double l_target : p.target_lateral_positions_m) {
      // l(s): 初期 (l0, l'0, 0) → 終端 (l_T, 0, 0) を弧長 L で結ぶ。L 以降は l_T を保つ。
      // 終端が経路終端より先なら経路終端で切る
      const double L = std::max(res, std::min(length, s_max - s0));
      const Polynomial lat(initial_state.l, initial_state.dl_ds, 0.0, l_target, 0.0, 0.0, L);

      PathCandidate path;
      for (double s = s0; s <= s_max + 1e-9; s += res) {
        path.s.push_back(s);
        path.l.push_back(s - s0 <= L ? lat.position(s - s0) : l_target);
      }
      if (path.s.size() < 2) {
        // ego が経路終端に居る (goal 到達)。1 点だけの経路は接線 heading・曲率 0 で埋める
        path.yaw.push_back(ref.azimuth(std::clamp(s0, 0.0, s_max)));
        path.kappa.push_back(0.0);
      } else {
        // heading は隣接点の位置差分、曲率は heading 差 / 距離 (等間隔なので低速ノイズが無い)。
        // 先頭の曲率は ego の実 heading からの変化で取り、ego と向きの違う出発を弾く
        std::vector<Point2d> xy;
        xy.reserve(path.s.size());
        for (std::size_t i = 0; i < path.s.size(); ++i) {
          xy.push_back(to_world_pose(ref, std::clamp(path.s[i], 0.0, s_max), path.l[i]).position);
        }
        for (std::size_t i = 0; i + 1 < xy.size(); ++i) {
          path.yaw.push_back(std::atan2(xy[i + 1].y() - xy[i].y(), xy[i + 1].x() - xy[i].x()));
        }
        path.yaw.push_back(path.yaw.back());
        double prev_yaw = ego_yaw;
        for (std::size_t i = 0; i < xy.size(); ++i) {
          const double dist = i + 1 < xy.size()
                                ? std::hypot(xy[i + 1].x() - xy[i].x(), xy[i + 1].y() - xy[i].y())
                                : res;
          path.kappa.push_back(
            autoware_utils_math::normalize_radian(path.yaw[i] - prev_yaw) / std::max(dist, 1e-3));
          prev_yaw = path.yaw[i];
        }
      }
      std::stringstream ss;
      ss << "L=" << L << " l=" << l_target;
      path.tag = ss.str();
      paths.push_back(std::move(path));
    }
  }
  return paths;
}

std::vector<FrenetSamplingBasedPlanner::VelocityProfile>
FrenetSamplingBasedPlanner::generate_velocity_profiles(
  const PlannerContext & context, const InitialState & initial_state,
  const CompiledConstraints & compiled_constraints) const
{
  using autoware::frenet_planner::Polynomial;

  const auto & p = params_.frenet_sampler;
  const double dt = p.time_step_s;
  const double horizon = p.horizon_s;
  const double s_max = context.reference_path.length();
  const auto limits = collect_kinematic_limits(compiled_constraints);
  const double v_limit = velocity_limit_at(compiled_constraints, limits, initial_state.s);

  std::vector<VelocityProfile> profiles;
  const auto sample = [&](const double duration, const double v_target, const double s_target) {
    const Polynomial lon(
      initial_state.s, initial_state.v, initial_state.a, s_target, v_target, 0.0, duration);
    VelocityProfile profile;
    for (double t = 0.0; t <= horizon + 1e-9; t += dt) {
      profile.t.push_back(t);
      if (t <= duration) {
        profile.s.push_back(lon.position(t));
        profile.v.push_back(lon.velocity(t));
        profile.a.push_back(lon.acceleration(t));
      } else {
        // 終端状態を保ってホライゾンまで延長 (等速)
        profile.s.push_back(s_target + v_target * (t - duration));
        profile.v.push_back(v_target);
        profile.a.push_back(0.0);
      }
    }
    std::stringstream ss;
    ss << "T=" << duration << " v=" << v_target;
    profile.tag = ss.str();
    profiles.push_back(std::move(profile));
  };

  for (const double duration : p.target_durations_s) {
    for (const double v_ratio : p.target_velocity_ratios) {
      const double v_target = v_ratio * v_limit;
      // 終端 s は平均速度で進んだ距離 (velocity-keeping 相当)。goal (経路終端) より先には出さない
      const double s_target =
        std::min(s_max, initial_state.s + 0.5 * (initial_state.v + v_target) * duration);
      sample(duration, v_target, s_target);
    }
  }

  // goal 停止プロファイル: 残距離を初速の平均で走り切る時間 T を距離から決める。固定の T 列
  // だけだと goal 手前で「T 内に残距離を進み切れない (逆走)」か「行き過ぎる」候補しか残らず、
  // 数 m 手前で全滅する
  {
    const double remaining = s_max - initial_state.s;
    const double duration = std::clamp(
      2.0 * remaining / std::max(initial_state.v, 0.1), p.target_durations_s.front(), horizon);
    sample(duration, 0.0, s_max);
  }
  return profiles;
}

FrenetSamplingBasedPlanner::VelocityProfile FrenetSamplingBasedPlanner::make_stop_profile(
  const InitialState & initial_state, const KinematicLimits & limits) const
{
  const auto & p = params_.frenet_sampler;
  const double dt = p.time_step_s;
  const double decel = std::abs(limits.a_hard_min);

  VelocityProfile profile;
  profile.tag = "stop";
  double s = initial_state.s;
  double v = initial_state.v;
  for (double t = 0.0; t <= p.horizon_s + 1e-9; t += dt) {
    profile.t.push_back(t);
    profile.s.push_back(s);
    profile.v.push_back(v);
    profile.a.push_back(v > 0.0 ? -decel : 0.0);
    const double v_next = std::max(0.0, v - decel * dt);
    s += 0.5 * (v + v_next) * dt;
    v = v_next;
  }
  return profile;
}

FrenetSamplingBasedPlanner::Candidate FrenetSamplingBasedPlanner::combine(
  const PlannerContext & context, const PathCandidate & path, const VelocityProfile & profile) const
{
  const auto & ref = context.reference_path;
  const double res = params_.frenet_sampler.path_resolution_m;
  const double s0 = path.s.front();
  const double s_max = ref.length();

  Candidate candidate;
  candidate.tag = path.tag + " " + profile.tag;
  candidate.s = profile.s;
  candidate.l.reserve(profile.s.size());
  candidate.points.reserve(profile.s.size());
  for (std::size_t k = 0; k < profile.t.size(); ++k) {
    const double s = profile.s[k];
    const double l = interpolate_uniform(path.l, s0, res, s);
    candidate.l.push_back(l);

    OptimizedTrajectoryPoint point;
    point.t = profile.t[k];
    point.pose = to_world_pose(ref, std::clamp(s, 0.0, s_max), l);
    point.pose.yaw = interpolate_uniform_angle(path.yaw, s0, res, s);
    point.kappa = interpolate_uniform(path.kappa, s0, res, s);
    point.v = profile.v[k];
    point.a = profile.a[k];
    candidate.points.push_back(point);
  }
  return candidate;
}

void FrenetSamplingBasedPlanner::evaluate(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  Candidate & candidate) const
{
  const auto & p = params_.frenet_sampler;
  const double s_max = context.reference_path.length();
  const double wheel_base_m = context.vehicle_info.wheel_base_m;
  const auto limits = collect_kinematic_limits(compiled_constraints);
  const auto bounds = collect_global_bounds(compiled_constraints);
  const double dt = p.time_step_s;
  const auto reject = [&](const char * reason) {
    candidate.valid = false;
    candidate.tag += std::string(" [") + reason + "]";
  };

  double cost = 0.0;
  for (std::size_t k = 0; k < candidate.points.size(); ++k) {
    const double s = candidate.s[k];
    const double l = candidate.l[k];
    const auto & point = candidate.points[k];

    // 経路終端 (goal) を越える候補・逆走する候補は不成立
    if (s > s_max + 1e-3) {
      return reject("beyond_goal");
    }
    if (point.v < -1e-3) {
      return reject("reverse");
    }
    // --- 車両運動 (VehicleKinematics の ScalarBound) ---
    const double v_max = velocity_limit_at(compiled_constraints, limits, s);
    if (point.v > v_max + 1e-6) {
      return reject("velocity");
    }
    if (point.a < limits.a_hard_min - 1e-6 || point.a > limits.a_hard_max + 1e-6) {
      return reject("lon_accel");
    }
    if (std::abs(point.kappa) > bounds.curvature) {
      return reject("curvature");
    }
    const double steer = std::atan(point.kappa * wheel_base_m);
    if (std::abs(steer) > bounds.steer_angle) {
      return reject("steer_angle");
    }
    if (std::abs(point.v * point.v * point.kappa) > bounds.lat_accel) {
      return reject("lat_accel");
    }
    if (k + 1 < candidate.points.size()) {
      const auto & next = candidate.points[k + 1];
      if (std::abs((next.a - point.a) / dt) > bounds.lon_jerk) {
        return reject("lon_jerk");
      }
      const double next_steer = std::atan(next.kappa * wheel_base_m);
      if (std::abs((next_steer - steer) / dt) > bounds.steer_rate) {
        return reject("steer_rate");
      }
    }

    // --- 幾何制約 (射影ビュー) ---
    const auto box = footprint_sl_box(context.vehicle_info, s, l);
    const double t0 = point.t;
    const double t1 = (k + 1 < candidate.points.size()) ? candidate.points[k + 1].t : t0;
    for (const auto & bound : compiled_constraints.lateral_bounds) {
      // SOFT 境界 (自レーン bound) は当面見ない。HARD (road_border) だけでふるう
      if (compiled_constraints.raw_constraints[bound.raw_index].hardness != Hardness::HARD) {
        continue;
      }
      if (violates_lateral_bound(bound, box)) {
        return reject("lateral_bound");
      }
    }
    for (const auto & occupancy : compiled_constraints.occupancies) {
      if (violates_occupancy(occupancy, compiled_constraints, box, t0, t1)) {
        return reject("occupancy");
      }
    }
    for (const auto & stop_bar : compiled_constraints.stop_bars) {
      if (violates_stop_bar(stop_bar, box, t0, t1)) {
        return reject("stop_bar");
      }
    }

    // soft コスト (時間積分)
    const double dv = v_max - point.v;
    cost += p.weights.lateral * l * l * dt;
    cost += p.weights.velocity * dv * dv * dt;
    cost += p.weights.curvature * point.kappa * point.kappa * dt;
    if (k + 1 < candidate.points.size()) {
      const double lon_jerk = (candidate.points[k + 1].a - point.a) / dt;
      cost += p.weights.lon_jerk * lon_jerk * lon_jerk * dt;
    }
  }
  candidate.cost = cost;
}

Trajectory FrenetSamplingBasedPlanner::to_trajectory_msg(
  const PlannerContext & context, const Candidate & candidate) const
{
  const double z = context.odometry.pose.pose.position.z;
  const double wheel_base_m = context.vehicle_info.wheel_base_m;

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = context.odometry.header.stamp;
  trajectory.points.reserve(candidate.points.size());
  for (const auto & point : candidate.points) {
    trajectory.points.push_back(to_trajectory_point(point, z, wheel_base_m));
  }
  const double engage_velocity_mps =
    params_.engage_velocity.enable ? params_.engage_velocity.velocity_hard_mps : 0.0;
  return set_engage_speed(trajectory, engage_velocity_mps);
}

void FrenetSamplingBasedPlanner::append_debug_markers(
  const PlannerContext & context, const std::vector<Candidate> & candidates,
  MarkerArray & debug_markers) const
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const double z = context.odometry.pose.pose.position.z;
  const auto stamp = context.odometry.header.stamp;

  auto valid_marker = create_default_marker(
    "map", stamp, "candidates_valid", 0, Marker::LINE_LIST, create_marker_scale(0.05, 0.0, 0.0),
    create_marker_color(0.0, 1.0, 0.0, 0.5));
  auto invalid_marker = create_default_marker(
    "map", stamp, "candidates_invalid", 0, Marker::LINE_LIST, create_marker_scale(0.03, 0.0, 0.0),
    create_marker_color(1.0, 0.0, 0.0, 0.2));

  for (const auto & candidate : candidates) {
    auto & marker = candidate.valid ? valid_marker : invalid_marker;
    for (std::size_t k = 0; k + 1 < candidate.points.size(); ++k) {
      for (const std::size_t i : {k, k + 1}) {
        geometry_msgs::msg::Point point;
        point.x = candidate.points[i].pose.position.x();
        point.y = candidate.points[i].pose.position.y();
        point.z = z;
        marker.points.push_back(point);
      }
    }
  }
  if (!valid_marker.points.empty()) {
    debug_markers.markers.push_back(valid_marker);
  }
  if (!invalid_marker.points.empty()) {
    debug_markers.markers.push_back(invalid_marker);
  }
}

}  // namespace autoware::safety_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::FrenetSamplingBasedPlanner,
  autoware::safety_planner::TrajectoryPlannerInterface)
