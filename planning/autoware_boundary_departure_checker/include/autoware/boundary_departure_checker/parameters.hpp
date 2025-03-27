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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::lane_departure_checker
{
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_utils::LinearRing2d;
using autoware_utils::Point2d;
using autoware_utils::Segment2d;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

struct Projection
{
  Point2d orig;
  Point2d proj;
  double dist{std::numeric_limits<double>::max()};
};

template <typename T>
struct Side
{
  T left;
  T right;
};

template <typename T>
struct SideExt : Side<T>
{
  Pose pose;
  double dist_from_start{0.0};
};

struct ProjectionWithSegment
{
  Projection projection;
  Segment2d nearest_segment;
  size_t idx_from_ego_sides_from_footprints{0};
  ProjectionWithSegment() = default;
  ProjectionWithSegment(Projection proj, Segment2d seg, size_t idx)
  : projection(std::move(proj)),
    nearest_segment(std::move(seg)),
    idx_from_ego_sides_from_footprints(idx)
  {
  }
};

using SideProjOpt = Side<std::optional<Projection>>;
using BoundarySide = Side<std::vector<Segment2d>>;
using SideToBoundPojections = Side<std::vector<ProjectionWithSegment>>;
using EgoSide = SideExt<Segment2d>;
using EgoSides = std::vector<EgoSide>;

struct Param
{
  double footprint_margin_scale{};
  double footprint_extra_margin{};
  double resample_interval{};
  double max_deceleration{};
  double delay_time{};
  double min_braking_distance{};
  // nearest search to ego
  double ego_nearest_dist_threshold{};
  double ego_nearest_yaw_threshold{};
};

struct Input
{
  nav_msgs::msg::Odometry::ConstSharedPtr current_odom;
  lanelet::LaneletMapPtr lanelet_map;
  LaneletRoute::ConstSharedPtr route;
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets shoulder_lanelets;
  Trajectory::ConstSharedPtr reference_trajectory;
  Trajectory::ConstSharedPtr predicted_trajectory;
  std::vector<std::string> boundary_types_to_detect;
};

struct Output
{
  std::map<std::string, double> processing_time_map;
  bool will_leave_lane{};
  bool is_out_of_lane{};
  bool will_cross_boundary{};
  lanelet::ConstLanelets candidate_lanelets;
  TrajectoryPoints resampled_trajectory;
  std::vector<LinearRing2d> vehicle_footprints;
  std::vector<LinearRing2d> vehicle_passing_areas;
  EgoSides ego_sides_from_footprints;
  SideToBoundPojections side_to_bound_projections;
};
}  // namespace autoware::lane_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__PARAMETERS_HPP_
