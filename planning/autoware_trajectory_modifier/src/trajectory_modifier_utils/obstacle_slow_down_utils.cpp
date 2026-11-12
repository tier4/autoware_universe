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

#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_slow_down_utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::utils::obstacle_slow_down
{
namespace
{
namespace bg = boost::geometry;
namespace mu = autoware::motion_utils;
namespace geom = autoware_utils_geometry;
using geom::Point2d;
using PredictedObject = autoware_perception_msgs::msg::PredictedObject;
using Shape = autoware_perception_msgs::msg::Shape;
using Polygon2d = geom::Polygon2d;
using UuidKey = std::array<std::uint8_t, 16>;

struct Obstacle
{
  ObjectId uuid{};
  geometry_msgs::msg::Pose pose{};
  std::uint8_t label{0};
  double lon_velocity{0.0};
  double lat_velocity{0.0};
  double clearance{0.0};
  double filtered_clearance{0.0};
  geometry_msgs::msg::Point front{};
  geometry_msgs::msg::Point back{};
  Side side{Side::Right};
};

struct PreviousOutput
{
  double target_velocity{0.0};
  double feasible_velocity{0.0};
  double clearance{0.0};
  std::optional<geometry_msgs::msg::Pose> start_pose;
  std::optional<geometry_msgs::msg::Pose> end_pose;
  Motion motion{Motion::Static};
};

using Box2d = bg::model::box<Point2d>;

struct Corridor
{
  std::vector<Polygon2d> polygons;
  std::vector<Box2d> boxes;
};

using PolygonCache = std::map<double, Corridor>;

std::vector<Box2d> bounding_boxes(const std::vector<Polygon2d> & polygons)
{
  std::vector<Box2d> boxes;
  boxes.reserve(polygons.size());
  for (const auto & polygon : polygons) boxes.push_back(bg::return_envelope<Box2d>(polygon));
  return boxes;
}

//! Lower bound of the distance between any shapes inside the boxes
double box_distance(const Box2d & a, const Box2d & b)
{
  const double dx = std::max(
    {0.0, a.min_corner().x() - b.max_corner().x(), b.min_corner().x() - a.max_corner().x()});
  const double dy = std::max(
    {0.0, a.min_corner().y() - b.max_corner().y(), b.min_corner().y() - a.max_corner().y()});
  return std::hypot(dx, dy);
}

UuidKey key(const ObjectId & id)
{
  return id.uuid;
}

Point2d to_2d(const geometry_msgs::msg::Point & point)
{
  return Point2d{point.x, point.y};
}

geometry_msgs::msg::Point to_msg(const Point2d & point)
{
  geometry_msgs::msg::Point p;
  p.x = point.x();
  p.y = point.y();
  return p;
}

bool is_lower_hysteresis(double value, bool previously_low, double high, double low)
{
  // Preserve the legacy comparison operators exactly.
  return previously_low ? !(high < value) : value < low;
}

double possible_object_radius(const Shape & shape)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  }
  if (shape.type == Shape::CYLINDER) return shape.dimensions.x / 2.0;
  if (shape.type == Shape::POLYGON) {
    double radius = 0.0;
    for (const auto & p : shape.footprint.points) {
      radius = std::max(radius, std::hypot(static_cast<double>(p.x), static_cast<double>(p.y)));
    }
    return radius;
  }
  return std::numeric_limits<double>::infinity();
}

std::pair<double, double> velocity_relative_to_trajectory(
  const PredictedObject & object, const TrajectoryPoints & trajectory)
{
  // Same transform and sign convention as legacy PlannerData::Object::calc_vel_relative_to_traj.
  const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  const auto & nearest = trajectory.at(mu::findNearestIndex(trajectory, pose.position)).pose;
  const double trajectory_yaw = tf2::getYaw(nearest.orientation);
  const double delta = tf2::getYaw(pose.orientation) - trajectory_yaw;
  const double lon = std::cos(delta) * twist.linear.x - std::sin(delta) * twist.linear.y;
  const double lat = std::sin(delta) * twist.linear.x + std::cos(delta) * twist.linear.y;
  const double dx = pose.position.x - nearest.position.x;
  const double dy = pose.position.y - nearest.position.y;
  const double cross = std::cos(trajectory_yaw) * dy - std::sin(trajectory_yaw) * dx;
  return {lon, cross > 0.0 ? -lat : lat};
}

geometry_msgs::msg::Pose predicted_current_pose(
  const PredictedObject & object, const rclcpp::Time & stamp, const rclcpp::Time & now)
{
  const auto & paths = object.kinematics.predicted_paths;
  const auto it = std::max_element(paths.begin(), paths.end(), [](const auto & a, const auto & b) {
    return a.confidence < b.confidence;
  });
  const double relative_time = (now - stamp).seconds();
  if (relative_time < 0.0 || it == paths.end()) {
    return object.kinematics.initial_pose_with_covariance.pose;
  }
  const auto pose = autoware::object_recognition_utils::calcInterpolatedPose(*it, relative_time);
  return pose ? *pose : object.kinematics.initial_pose_with_covariance.pose;
}

TrajectoryPoints decimate_trajectory(
  const TrajectoryPoints & trajectory, const geometry_msgs::msg::Pose & ego_pose,
  const TrajectoryPolygonParam & p, const double extension_length)
{
  const auto index = mu::findFirstNearestSegmentIndexWithSoftConstraints(
    trajectory, ego_pose, p.ego_nearest_dist_threshold, p.ego_nearest_yaw_threshold);
  TrajectoryPoints trimmed(trajectory.begin() + index, trajectory.end());
  if (trimmed.size() < 2) return trajectory;  // Same fallback as legacy utility.
  const auto msg = mu::convertToTrajectory(trimmed);
  auto points = mu::convertToTrajectoryPointArray(
    mu::resampleTrajectory(msg, p.decimate_trajectory_step_length));
  if (points.size() < 2) return trajectory;
  const bool forward = geom::is_driving_forward(trimmed.at(0), trimmed.at(1));
  mu::insertOrientationAsSpline(points, forward);
  if (extension_length >= 0.1) {
    const auto goal = points.back();
    for (double s = p.decimate_trajectory_step_length;
         s < extension_length - p.decimate_trajectory_step_length;
         s += p.decimate_trajectory_step_length) {
      auto point = goal;
      point.pose = geom::calc_offset_pose(goal.pose, forward ? s : -s, 0.0, 0.0);
      points.push_back(point);
    }
    auto point = goal;
    point.pose =
      geom::calc_offset_pose(goal.pose, forward ? extension_length : -extension_length, 0.0, 0.0);
    points.push_back(point);
  }
  return points;
}

std::vector<geometry_msgs::msg::Pose> error_poses(
  const TrajectoryPoints & trajectory, const geometry_msgs::msg::Pose & ego_pose,
  const double convergence_time)
{
  std::vector<geometry_msgs::msg::Pose> errors;
  const auto idx = mu::findNearestSegmentIndex(trajectory, ego_pose.position);
  const auto nearest_pose = trajectory.at(idx).pose;
  const auto relative = geom::inverse_transform_pose(ego_pose, nearest_pose);
  const double yaw_error = tf2::getYaw(relative.orientation);
  double elapsed = 0.0;
  for (size_t i = 0; i < trajectory.size() && elapsed < convergence_time; ++i) {
    const double ratio = (convergence_time - elapsed) / convergence_time;
    geometry_msgs::msg::Pose error;
    error.position.y = relative.position.y * ratio;
    error.orientation = geom::create_quaternion_from_yaw(yaw_error * ratio);
    errors.push_back(geom::transform_pose(error, trajectory.at(i).pose));
    if (i + 1 < trajectory.size() && trajectory.at(i).longitudinal_velocity_mps != 0.0f) {
      elapsed +=
        geom::calc_distance2d(trajectory.at(i).pose.position, trajectory.at(i + 1).pose.position) /
        std::abs(trajectory.at(i).longitudinal_velocity_mps);
    } else {
      break;
    }
  }
  return errors;
}

std::vector<double> wheel_off_tracking(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle)
{
  auto curvature = mu::calcCurvature(trajectory);
  if (mu::isDrivingForward(trajectory) == false) {
    for (auto & c : curvature) c = -c;
  }
  for (auto & c : curvature) {
    const double outer = c / (1.0 + std::abs(c) * vehicle.vehicle_width_m / 2.0);
    c = -vehicle.wheel_base_m * std::tan(0.5 * std::atan(outer * vehicle.wheel_base_m));
  }
  return curvature;
}

void append_footprint(
  Polygon2d & poly, const geometry_msgs::msg::Pose & pose, const VehicleInfo & vehicle,
  const double left_margin, const double right_margin, const bool first_forward)
{
  const double half_width = vehicle.vehicle_width_m / 2.0;
  const double rear_left = first_forward ? 0.0 : left_margin;
  const double rear_right = first_forward ? 0.0 : right_margin;
  for (const auto & p : std::array<geometry_msgs::msg::Point, 5>{
         geom::calc_offset_pose(
           pose, vehicle.max_longitudinal_offset_m, half_width + left_margin, 0.0)
           .position,
         geom::calc_offset_pose(
           pose, vehicle.max_longitudinal_offset_m, -half_width - right_margin, 0.0)
           .position,
         geom::calc_offset_pose(pose, -vehicle.rear_overhang_m, -half_width - rear_right, 0.0)
           .position,
         geom::calc_offset_pose(pose, -vehicle.rear_overhang_m, half_width + rear_left, 0.0)
           .position,
         geom::calc_offset_pose(
           pose, vehicle.max_longitudinal_offset_m, half_width + left_margin, 0.0)
           .position}) {
    bg::append(poly, to_2d(p));
  }
}

std::vector<Polygon2d> swept_polygons(
  const TrajectoryPoints & trajectory, const VehicleInfo & vehicle,
  const geometry_msgs::msg::Pose & ego_pose, const double margin, const TrajectoryPolygonParam & p,
  const double off_track_scale)
{
  // This is the local equivalent of legacy polygon_utils::create_one_step_polygons.
  const auto errors = p.enable_to_consider_current_pose
                        ? error_poses(trajectory, ego_pose, p.time_to_convergence)
                        : std::vector<geometry_msgs::msg::Pose>{};
  const auto off_track = off_track_scale > 0.0 ? wheel_off_tracking(trajectory, vehicle)
                                               : std::vector<double>(trajectory.size(), 0.0);
  std::vector<Polygon2d> result;
  result.reserve(trajectory.size());
  Polygon2d previous;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const double left = margin + off_track_scale * std::max(off_track.at(i), 0.0);
    const double right = margin + off_track_scale * std::max(-off_track.at(i), 0.0);
    Polygon2d current;
    const bool first_forward = i == 0 && trajectory.at(i).longitudinal_velocity_mps > 1e-3f;
    append_footprint(current, trajectory.at(i).pose, vehicle, left, right, first_forward);
    if (i < errors.size())
      append_footprint(current, errors.at(i), vehicle, left, right, first_forward);
    bg::append(previous, current.outer());
    Polygon2d hull;
    bg::convex_hull(previous, hull);
    bg::correct(hull);
    result.push_back(std::move(hull));
    previous = std::move(current);
  }
  return result;
}

double clearance_to_trajectory(
  const Polygon2d & object, const Box2d & object_box, const Corridor & corridor)
{
  // Skips polygons whose box distance cannot beat the best so far; the margin absorbs rounding
  constexpr double BOUND_MARGIN_M = 1e-6;
  const auto & polygons = corridor.polygons;
  if (polygons.empty()) return std::numeric_limits<double>::max();
  std::vector<double> bounds(polygons.size());
  for (size_t i = 0; i < polygons.size(); ++i) {
    bounds[i] = box_distance(corridor.boxes[i], object_box);
  }
  const auto seed = static_cast<size_t>(
    std::distance(bounds.begin(), std::min_element(bounds.begin(), bounds.end())));
  double distance = bg::distance(polygons[seed], object);
  for (size_t i = 0; i < polygons.size(); ++i) {
    if (i == seed || bounds[i] - BOUND_MARGIN_M >= distance) continue;
    distance = std::min(distance, bg::distance(polygons[i], object));
  }
  return distance;
}

using CollisionLimits = std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>;
std::optional<CollisionLimits> collision_limits(
  const Polygon2d & object, const Box2d & object_box, const TrajectoryPoints & decimated,
  const Corridor & expanded_corridor)
{
  // IMPORTANT: polygon i is built from *decimated* poses i-1 and i.
  // Never use polygon indices against the original, un-decimated trajectory.
  const auto & expanded = expanded_corridor.polygons;
  std::vector<Polygon2d> first, last;
  size_t first_segment = 0, last_segment = 0;
  for (size_t i = 0; i < expanded.size(); ++i) {
    std::vector<Polygon2d> intersections;
    if (!bg::disjoint(expanded_corridor.boxes.at(i), object_box)) {
      bg::intersection(expanded.at(i), object, intersections);
    }
    if (!intersections.empty()) {
      if (first.empty()) {
        first = intersections;
        first_segment = i == 0 ? 0 : i - 1;
      }
      last = std::move(intersections);
      last_segment = i == 0 ? 0 : i - 1;
    } else if (!last.empty()) {
      break;  // Legacy only tracks the first contiguous intersection span.
    }
  }
  if (first.empty() || last.empty() || decimated.size() < 2) return std::nullopt;
  double min_offset = std::numeric_limits<double>::max();
  double max_offset = std::numeric_limits<double>::lowest();
  geometry_msgs::msg::Point front, back;
  for (const auto & intersection : first) {
    for (const auto & point : intersection.outer()) {
      const auto p = to_msg(point);
      const double offset = mu::calcLongitudinalOffsetToSegment(decimated, first_segment, p);
      if (offset < min_offset) {
        min_offset = offset;
        front = p;
      }
    }
  }
  if (min_offset == std::numeric_limits<double>::max()) return std::nullopt;
  back = front;
  for (const auto & intersection : last) {
    for (const auto & point : intersection.outer()) {
      const auto p = to_msg(point);
      const double offset = mu::calcLongitudinalOffsetToSegment(decimated, last_segment, p);
      if (offset > max_offset) {
        max_offset = offset;
        back = p;
      }
    }
  }
  return CollisionLimits{front, back};
}

double reach_time(
  const double jerk, const double accel, const double velocity, const double distance, double lower,
  double upper)
{
  const auto travel = [&](double t) {
    return jerk * t * t * t / 6.0 + accel * t * t / 2.0 + velocity * t;
  };
  if (lower > upper || travel(lower) > distance || travel(upper) < distance) {
    throw std::logic_error("Obstacle SlowDown: invalid reach-time search range");
  }
  // The original implementation loops until a 1e-5 residual. Bound the iteration count.
  for (int i = 0; i < 100; ++i) {
    const double t = (lower + upper) / 2.0;
    const double error = travel(t) - distance;
    if (std::abs(error) < 1e-5) return t;
    if (error > 0.0)
      upper = t;
    else
      lower = t;
  }
  return (lower + upper) / 2.0;
}

double deceleration_velocity(
  const double jerk, const double min_acc, const double current_acc, const double velocity,
  const double distance)
{
  if (distance <= 0.0) return velocity;
  const auto travel = [](double t, double j, double a, double v) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t;
  };
  const auto vel = [](double t, double j, double a, double v) {
    return j * t * t / 2.0 + a * t + v;
  };
  const double t_jerk = (min_acc - current_acc) / jerk;
  const double remaining = distance - travel(t_jerk, jerk, current_acc, velocity);
  if (remaining < 0.0) {
    return vel(
      reach_time(jerk, current_acc, velocity, distance, 0.0, t_jerk), jerk, current_acc, velocity);
  }
  const double v1 = vel(t_jerk, jerk, current_acc, velocity);
  const double discriminant = v1 * v1 + 2.0 * min_acc * remaining;
  if (discriminant <= 0.0) return 0.0;
  return vel((-v1 + std::sqrt(discriminant)) / min_acc, 0.0, min_acc, v1);
}

Motion classify_motion(
  const Obstacle & obstacle, const PreviousOutput * previous, const SlowDownPlanningParam & p)
{
  const double speed = std::hypot(obstacle.lon_velocity, obstacle.lat_velocity);
  if (!previous) return speed > p.moving_object_speed_threshold ? Motion::Moving : Motion::Static;
  if (previous->motion == Motion::Moving) {
    return speed > p.moving_object_speed_threshold - p.moving_object_hysteresis_range
             ? Motion::Moving
             : Motion::Static;
  }
  return speed > p.moving_object_speed_threshold + p.moving_object_hysteresis_range
           ? Motion::Moving
           : Motion::Static;
}

double target_velocity(
  Obstacle & obstacle, const PreviousOutput * previous, const SlowDownPlanningParam & p,
  const Motion motion)
{
  const auto & cls = p.object_type_specific_params.at(obstacle.label);
  const auto & limits =
    cls.velocity_params.at(static_cast<size_t>(obstacle.side)).at(static_cast<size_t>(motion));
  obstacle.filtered_clearance =
    previous ? autoware::signal_processing::lowpassFilter(
                 obstacle.clearance, previous->clearance, p.lpf_gain_lateral_distance)
             : obstacle.clearance;
  const double ratio = std::clamp(
    (std::abs(obstacle.filtered_clearance) - limits.min_lat_margin) /
      (limits.max_lat_margin - limits.min_lat_margin),
    0.0, 1.0);
  return limits.min_ego_velocity + ratio * (limits.max_ego_velocity - limits.min_ego_velocity);
}

struct SlowdownCandidate
{
  double start_s;
  double end_s;
  double feasible_velocity;
};

std::optional<SlowdownCandidate> slowdown_candidate(
  Obstacle & object, const PreviousOutput * previous, Motion motion,
  const TrajectoryPoints & trajectory, const geometry_msgs::msg::Pose & ego_pose,
  double ego_velocity, double ego_acceleration, const VehicleInfo & vehicle,
  const SlowDownPlanningParam & p)
{
  const bool is_forward = mu::isDrivingForward(trajectory) != false;
  const double bumper = is_forward ? std::abs(vehicle.max_longitudinal_offset_m)
                                   : std::abs(vehicle.min_longitudinal_offset_m);
  const double ego_s = mu::calcSignedArcLength(trajectory, 0UL, ego_pose.position);
  const double nominal = target_velocity(object, previous, p, motion);
  const double front = mu::calcSignedArcLength(trajectory, 0UL, object.front);
  const double back = mu::calcSignedArcLength(trajectory, 0UL, object.back);

  double offset = 0.0;
  if (front >= ego_s + bumper) {
    const double assumed =
      object.lon_velocity > 0.0 ? std::max(ego_velocity, nominal) : std::min(ego_velocity, nominal);
    const double relative = std::max(1.0, assumed - object.lon_velocity);
    offset = object.lon_velocity * std::max(0.0, (front - ego_s - bumper) / relative);
  }
  const double prepare = std::max(3.0, nominal * p.time_margin_on_target_velocity);
  const double start = front + offset - bumper - prepare;
  const double end = back + offset - bumper;
  if (start > 100.0) return std::nullopt;

  const auto smooth_arc = [&](double current, const std::optional<geometry_msgs::msg::Pose> & old) {
    if (!old) return current;
    const auto segment = mu::findNearestSegmentIndex(trajectory, old->position);
    const double old_s = mu::calcSignedArcLength(trajectory, 0UL, old->position, segment);
    return autoware::signal_processing::lowpassFilter(current, old_s, p.lpf_gain_dist_to_slow_down);
  };
  const double smooth_start = smooth_arc(start, previous ? previous->start_pose : std::nullopt);
  const double smooth_end = smooth_arc(end, previous ? previous->end_pose : std::nullopt);
  const double distance_to_start = smooth_start - ego_s;

  double feasible = nominal;
  if (distance_to_start < 0.0) {
    feasible = previous ? previous->target_velocity : std::max(ego_velocity, nominal);
  } else if (ego_velocity >= nominal) {
    double one_shot;
    if (ego_acceleration < p.slow_down_min_acc) {
      const double squared =
        ego_velocity * ego_velocity + 2.0 * distance_to_start * p.slow_down_min_acc;
      one_shot = squared < 0.0 ? nominal : std::max(std::sqrt(squared), nominal);
    } else {
      one_shot = deceleration_velocity(
        p.slow_down_min_jerk, p.slow_down_min_acc, ego_acceleration, ego_velocity,
        distance_to_start);
    }
    if (previous && previous->start_pose) {
      const double previous_start =
        mu::calcSignedArcLength(trajectory, 0UL, previous->start_pose->position);
      const double diff = smooth_start - previous_start;
      const double previous_feasible = std::sqrt(
        std::max(
          0.0, previous->feasible_velocity * previous->feasible_velocity +
                 2.0 * p.slow_down_min_acc * diff));
      feasible = std::max(nominal, std::min(one_shot, previous_feasible));
    } else {
      feasible = std::max(nominal, one_shot);
    }
  }
  return SlowdownCandidate{smooth_start, smooth_end, feasible};
}

}  // namespace

void validate_parameters(const SlowDownParameters & p)
{
  const auto & f = p.obstacle_filtering;
  const auto & s = p.slow_down_planning;
  const auto & t = p.trajectory_polygon;
  const auto in_unit_interval = [](double v) { return std::isfinite(v) && v >= 0 && v <= 1; };
  const auto is_valid_interpolation = [](const VelocityInterpolationParam & v) {
    return v.min_lat_margin < v.max_lat_margin && v.min_ego_velocity <= v.max_ego_velocity;
  };
  if (
    f.object_types.empty() || f.max_lat_margin <= f.min_lat_margin || f.lat_hysteresis_margin < 0 ||
    f.max_lat_velocity <= 0 || f.successive_num_to_entry_slow_down_condition <= 0 ||
    f.successive_num_to_exit_slow_down_condition <= 0 || s.slow_down_min_acc >= 0 ||
    s.slow_down_min_jerk >= 0 || s.time_margin_on_target_velocity < 0 ||
    s.moving_object_speed_threshold < 0 || s.moving_object_hysteresis_range < 0 ||
    !in_unit_interval(s.lpf_gain_slow_down_vel) || !in_unit_interval(s.lpf_gain_lateral_distance) ||
    !in_unit_interval(s.lpf_gain_dist_to_slow_down) || t.decimate_trajectory_step_length <= 0 ||
    t.ego_nearest_dist_threshold < 0 || t.ego_nearest_yaw_threshold < 0 ||
    (t.enable_to_consider_current_pose && t.time_to_convergence <= 0)) {
    throw std::invalid_argument("Obstacle SlowDown: missing or invalid YAML parameters");
  }
  for (const auto label : f.object_types) {
    const auto it = s.object_type_specific_params.find(label);
    if (it == s.object_type_specific_params.end()) {
      throw std::invalid_argument("Obstacle SlowDown: missing velocity parameters for class");
    }
    for (const auto & row : it->second.velocity_params) {
      if (!std::all_of(row.begin(), row.end(), is_valid_interpolation)) {
        throw std::invalid_argument("Obstacle SlowDown: invalid velocity interpolation bounds");
      }
    }
  }
}

struct SlowDownPlanner::Impl
{
  std::map<UuidKey, int> counters;
  std::set<UuidKey> previous_obstacles;
  std::map<UuidKey, PreviousOutput> previous_outputs;
};

SlowDownPlanner::SlowDownPlanner() : impl_(std::make_unique<Impl>())
{
}
SlowDownPlanner::~SlowDownPlanner() = default;
SlowDownPlanner::SlowDownPlanner(SlowDownPlanner &&) noexcept = default;
SlowDownPlanner & SlowDownPlanner::operator=(SlowDownPlanner &&) noexcept = default;

void SlowDownPlanner::reset()
{
  impl_->counters.clear();
  impl_->previous_obstacles.clear();
  impl_->previous_outputs.clear();
}

std::vector<SlowDownInterval> SlowDownPlanner::plan(
  const TrajectoryPoints & trajectory, const PredictedObjects & objects,
  const geometry_msgs::msg::Pose & ego_pose, const double ego_velocity,
  const double ego_acceleration, const VehicleInfo & vehicle_info, const rclcpp::Time & now,
  const SlowDownParameters & params)
{
  std::vector<SlowDownInterval> result;
  if (trajectory.size() < 2) {
    reset();
    return result;
  }
  const auto & filtering = params.obstacle_filtering;
  const auto & geometry = params.trajectory_polygon;
  const auto & planning = params.slow_down_planning;
  const rclcpp::Time stamp(objects.header.stamp);

  const auto clearance_trajectory =
    decimate_trajectory(trajectory, ego_pose, geometry, geometry.goal_extended_trajectory_length);
  const auto detection_trajectory = decimate_trajectory(trajectory, ego_pose, geometry, 0.0);
  Corridor expanded;
  expanded.polygons = swept_polygons(
    detection_trajectory, vehicle_info, ego_pose,
    filtering.max_lat_margin + filtering.lat_hysteresis_margin, geometry, 0.0);
  expanded.boxes = bounding_boxes(expanded.polygons);
  const double ego_radius =
    std::hypot(vehicle_info.max_longitudinal_offset_m, vehicle_info.vehicle_width_m / 2.0);
  PolygonCache clearance_cache;
  std::set<UuidKey> observed;
  std::set<UuidKey> accepted_ids;
  std::vector<Obstacle> accepted;

  // The resampled centerline is common to all objects and needs only one computation.
  std::vector<geometry_msgs::msg::Point> centerline_points;
  centerline_points.reserve(trajectory.size());
  for (const auto & p : trajectory) centerline_points.push_back(p.pose.position);
  const auto centerline_resampled = mu::resamplePointVector(centerline_points, 2.0);
  bg::model::linestring<Point2d> centerline;
  for (const auto & p : centerline_resampled) centerline.push_back(to_2d(p));

  for (const auto & object : objects.objects) {
    if (object.classification.empty()) continue;
    if (
      object.shape.type != Shape::BOUNDING_BOX && object.shape.type != Shape::CYLINDER &&
      object.shape.type != Shape::POLYGON)
      continue;
    const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
    if (mu::calcSignedArcLength(trajectory, ego_pose.position, pos) < 0.0) continue;
    const double lower_bound = std::abs(mu::calcLateralOffset(trajectory, pos)) - ego_radius -
                               possible_object_radius(object.shape);
    if (lower_bound > filtering.max_lat_margin) continue;

    const auto label = object.classification.front().label;
    // Check class before retrieving type-specific parameters; legacy had an .at() crash here.
    if (
      std::find(filtering.object_types.begin(), filtering.object_types.end(), label) ==
      filtering.object_types.end())
      continue;
    const auto & class_param = planning.object_type_specific_params.at(label);
    const double scale = class_param.wheel_off_track_scale;
    if (clearance_cache.count(scale) == 0) {
      Corridor corridor;
      corridor.polygons =
        swept_polygons(clearance_trajectory, vehicle_info, ego_pose, 0.0, geometry, scale);
      corridor.boxes = bounding_boxes(corridor.polygons);
      clearance_cache.emplace(scale, std::move(corridor));
    }
    const auto polygon =
      geom::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape);
    const auto polygon_box = bg::return_envelope<Box2d>(polygon);
    const double clearance =
      clearance_to_trajectory(polygon, polygon_box, clearance_cache.at(scale));
    const auto id = key(object.object_id);
    observed.insert(id);
    if (clearance <= filtering.min_lat_margin || bg::intersects(polygon, centerline)) continue;

    const auto [lon_velocity, lat_velocity] = velocity_relative_to_trajectory(object, trajectory);
    const bool was_active = impl_->previous_obstacles.count(id) != 0;
    const bool near = is_lower_hysteresis(
      clearance, was_active, filtering.max_lat_margin + filtering.lat_hysteresis_margin / 2.0,
      filtering.max_lat_margin - filtering.lat_hysteresis_margin / 2.0);
    const bool condition = near && std::abs(lat_velocity) < filtering.max_lat_velocity;

    bool enabled = false;
    if (was_active) {
      if (condition) {
        enabled = true;
      } else {
        int & count = impl_->counters[id];
        count = std::min(-1, count - 1);
        if (count <= -filtering.successive_num_to_exit_slow_down_condition) {
          count = 0;  // Fix legacy map.emplace() reset (would not overwrite existing key).
        } else {
          enabled = true;
        }
      }
    } else if (condition) {
      int & count = impl_->counters[id];
      count = std::max(1, count + 1);
      if (count >= filtering.successive_num_to_entry_slow_down_condition) {
        count = 0;
        enabled = true;
      }
    }
    if (!enabled) continue;
    const auto limits = collision_limits(polygon, polygon_box, detection_trajectory, expanded);
    if (!limits) continue;

    Obstacle o;
    o.uuid = object.object_id;
    o.label = label;
    o.pose = predicted_current_pose(object, stamp, now);
    o.lon_velocity = lon_velocity;
    o.lat_velocity = lat_velocity;
    o.clearance = clearance;
    o.front = limits->first;
    o.back = limits->second;
    o.side = mu::calcLateralOffset(trajectory, o.pose.position) > 0.0 ? Side::Left : Side::Right;
    accepted_ids.insert(id);
    accepted.push_back(std::move(o));
  }

  for (auto it = impl_->counters.begin(); it != impl_->counters.end();) {
    if (observed.count(it->first) == 0)
      it = impl_->counters.erase(it);
    else
      ++it;
  }
  impl_->previous_obstacles = std::move(accepted_ids);

  auto working = trajectory;
  std::map<UuidKey, PreviousOutput> next_outputs;
  for (auto & object : accepted) {
    const auto id = key(object.uuid);
    const auto old_it = impl_->previous_outputs.find(id);
    const PreviousOutput * old =
      old_it == impl_->previous_outputs.end() ? nullptr : &old_it->second;
    const auto motion = classify_motion(object, old, planning);
    const auto candidate = slowdown_candidate(
      object, old, motion, working, ego_pose, ego_velocity, ego_acceleration, vehicle_info,
      planning);
    if (!candidate) continue;

    // Keep legacy's insertTargetPoint + original speed check; no trajectory velocity is modified.
    const auto insert = [&](double s) -> std::optional<size_t> {
      const auto idx = mu::insertTargetPoint(0UL, s, working);
      if (!idx) return std::nullopt;
      if (*idx + 1 < working.size()) {
        working.at(*idx).longitudinal_velocity_mps = working.at(*idx + 1).longitudinal_velocity_mps;
      }
      return *idx;
    };
    const auto start_idx = insert(candidate->start_s);
    const auto end_idx =
      candidate->start_s < candidate->end_s ? insert(candidate->end_s) : std::nullopt;
    if (!end_idx) continue;
    const size_t begin = start_idx.value_or(0);
    if (begin >= *end_idx || *end_idx >= working.size()) continue;
    const double stable =
      old ? autoware::signal_processing::lowpassFilter(
              candidate->feasible_velocity, old->target_velocity, planning.lpf_gain_slow_down_vel)
          : candidate->feasible_velocity;
    const bool effective = std::any_of(
      working.begin() + begin, working.begin() + *end_idx,
      [&](const auto & p) { return stable < p.longitudinal_velocity_mps; });
    if (!effective) continue;

    const auto & start_pose = working.at(begin).pose;
    const auto & end_pose = working.at(*end_idx).pose;
    result.push_back(
      SlowDownInterval{
        object.uuid, mu::calcSignedArcLength(trajectory, 0UL, start_pose.position),
        mu::calcSignedArcLength(trajectory, 0UL, end_pose.position), stable, start_pose.position,
        end_pose.position});
    next_outputs[id] = PreviousOutput{
      stable,
      candidate->feasible_velocity,
      object.filtered_clearance,
      start_idx ? std::optional<geometry_msgs::msg::Pose>{start_pose} : std::nullopt,
      end_pose,
      motion};
  }
  impl_->previous_outputs = std::move(next_outputs);
  return result;
}

}  // namespace autoware::trajectory_modifier::utils::obstacle_slow_down
