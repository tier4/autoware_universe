#ifndef TIER4_AUTOWARE_UTILS__TEST_FIRST_NEAREST_INPUTS_HPP_
#define TIER4_AUTOWARE_UTILS__TEST_FIRST_NEAREST_INPUTS_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace first_nearest_inputs
{
using TrajectoryPoints = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;

constexpr double distance_thresh_default = 9.0;
constexpr double distance_thresh_small = 0.5;  // overlap2_n が折返し前も窓に入る下限
constexpr double max_dist = 9.0;
constexpr double max_yaw = tier4_autoware_utils::deg2rad(60.0);

inline geometry_msgs::msg::Pose makePose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position = tier4_autoware_utils::createPoint(x, y, 0.0);
  pose.orientation = tier4_autoware_utils::createQuaternionFromRPY(0.0, 0.0, yaw);
  return pose;
}

inline geometry_msgs::msg::Point makePoint(const double x, const double y)
{
  return tier4_autoware_utils::createPoint(x, y, 0.0);
}

inline autoware_auto_planning_msgs::msg::TrajectoryPoint makeTrajPoint(
  const double x, const double y, const double yaw)
{
  autoware_auto_planning_msgs::msg::TrajectoryPoint p;
  p.pose = makePose(x, y, yaw);
  return p;
}

// --- trajectories ---

// 折線 S: (0,0)->(10,0)->(10,10)->(20,10)、点間隔 0.1 m
inline TrajectoryPoints traj_plain()
{
  TrajectoryPoints points;
  points.reserve(301);
  constexpr double yaw_x = 0.0;
  constexpr double yaw_y = tier4_autoware_utils::deg2rad(90.0);

  for (int i = 0; i < 100; ++i) {
    points.push_back(makeTrajPoint(0.1 * i, 0.0, yaw_x));
  }
  points.push_back(makeTrajPoint(10.0, 0.0, yaw_y));
  for (int i = 1; i < 100; ++i) {
    points.push_back(makeTrajPoint(10.0, 0.1 * i, yaw_y));
  }
  points.push_back(makeTrajPoint(10.0, 10.0, yaw_x));
  for (int i = 1; i <= 100; ++i) {
    points.push_back(makeTrajPoint(10.0 + 0.1 * i, 10.0, yaw_x));
  }
  return points;
}

inline TrajectoryPoints traj_overlap()
{
  // 15+2+10+3、全て直角右折: (0,0)->(15,0)->(15,-2)->(5,-2)->(5,1)、点間隔 0.1 m
  TrajectoryPoints points;
  points.reserve(301);
  constexpr double yaw_px = 0.0;
  constexpr double yaw_my = tier4_autoware_utils::deg2rad(-90.0);
  constexpr double yaw_mx = tier4_autoware_utils::deg2rad(180.0);
  constexpr double yaw_py = tier4_autoware_utils::deg2rad(90.0);

  for (int i = 0; i < 150; ++i) {
    points.push_back(makeTrajPoint(0.1 * i, 0.0, yaw_px));
  }
  points.push_back(makeTrajPoint(15.0, 0.0, yaw_my));
  for (int i = 1; i < 20; ++i) {
    points.push_back(makeTrajPoint(15.0, -0.1 * i, yaw_my));
  }
  points.push_back(makeTrajPoint(15.0, -2.0, yaw_mx));
  for (int i = 1; i < 100; ++i) {
    points.push_back(makeTrajPoint(15.0 - 0.1 * i, -2.0, yaw_mx));
  }
  points.push_back(makeTrajPoint(5.0, -2.0, yaw_py));
  for (int i = 1; i <= 30; ++i) {
    points.push_back(makeTrajPoint(5.0, -2.0 + 0.1 * i, yaw_py));
  }
  return points;
}

inline geometry_msgs::msg::Pose plain_n_pose()
{
  return makePose(2.0, 0.05, 0.0);
}

inline geometry_msgs::msg::Point plain_n_point()
{
  return makePoint(5.0, 0.05);
}

inline geometry_msgs::msg::Pose plain_f_pose()
{
  return makePose(3.5, 0.6, 0.0);
}

inline geometry_msgs::msg::Point plain_f_point()
{
  return makePoint(6.5, 0.6);
}

inline geometry_msgs::msg::Pose plain_y_pose()
{
  return makePose(7.5, 0.05, tier4_autoware_utils::deg2rad(70.0));
}

inline geometry_msgs::msg::Pose overlap1_n_pose()
{
  return makePose(4.7, 0.05, 0.0);
}

inline geometry_msgs::msg::Point overlap1_n_point()
{
  return makePoint(5.3, 0.05);
}

inline geometry_msgs::msg::Pose overlap2_n_pose()
{
  return makePose(4.94, -0.4, tier4_autoware_utils::deg2rad(45.0));
}

inline geometry_msgs::msg::Point overlap2_n_point()
{
  return makePoint(5.04, 0.3);
}

inline geometry_msgs::msg::Pose overlap2_f_pose()
{
  return makePose(4.4, -0.7, tier4_autoware_utils::deg2rad(45.0));
}

inline geometry_msgs::msg::Point overlap2_f_point()
{
  return makePoint(4.4, 0.7);
}

}  // namespace first_nearest_inputs

#endif  // TIER4_AUTOWARE_UTILS__TEST_FIRST_NEAREST_INPUTS_HPP_
