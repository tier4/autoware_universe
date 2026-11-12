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

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace
{
using autoware::trajectory_modifier::utils::obstacle_slow_down::Motion;
using autoware::trajectory_modifier::utils::obstacle_slow_down::ObjectTypeSpecificParams;
using autoware::trajectory_modifier::utils::obstacle_slow_down::Side;
using autoware::trajectory_modifier::utils::obstacle_slow_down::SlowDownInterval;
using autoware::trajectory_modifier::utils::obstacle_slow_down::SlowDownParameters;
using autoware::trajectory_modifier::utils::obstacle_slow_down::SlowDownPlanner;
using autoware::trajectory_modifier::utils::obstacle_slow_down::TrajectoryPoints;
using autoware::trajectory_modifier::utils::obstacle_slow_down::validate_parameters;
using autoware::trajectory_modifier::utils::obstacle_slow_down::VelocityInterpolationParam;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::TrajectoryPoint;

constexpr double EGO_VELOCITY = 10.0;
constexpr double OBJECT_X = 40.0;
// The car is 2 m wide and the ego 1.8 m, so the clearance to the ego band is |y| - 1.9
constexpr double NEAR_Y = 2.4;  // clearance 0.5 m: inside the entry clearance 0.9 m
constexpr int ENTRY_NUM = 5;
constexpr int EXIT_NUM = 5;

// The legacy defaults of the X2 configuration, for CAR and UNKNOWN
SlowDownParameters make_params()
{
  SlowDownParameters p;
  p.obstacle_filtering.object_types = {ObjectClassification::CAR};
  p.obstacle_filtering.min_lat_margin = -10.0;
  p.obstacle_filtering.max_lat_margin = 1.0;
  p.obstacle_filtering.lat_hysteresis_margin = 0.2;
  p.obstacle_filtering.max_lat_velocity = 3.0;
  p.obstacle_filtering.successive_num_to_entry_slow_down_condition = ENTRY_NUM;
  p.obstacle_filtering.successive_num_to_exit_slow_down_condition = EXIT_NUM;

  auto & s = p.slow_down_planning;
  s.slow_down_min_acc = -1.0;
  s.slow_down_min_jerk = -1.0;
  s.lpf_gain_slow_down_vel = 0.0;
  s.lpf_gain_lateral_distance = 0.8;
  s.lpf_gain_dist_to_slow_down = 0.8;
  s.time_margin_on_target_velocity = 1.5;
  s.moving_object_speed_threshold = 0.5;
  s.moving_object_hysteresis_range = 0.1;
  ObjectTypeSpecificParams class_params;
  for (const auto side : {Side::Left, Side::Right}) {
    auto & row = class_params.velocity_params[static_cast<std::size_t>(side)];
    row[static_cast<std::size_t>(Motion::Moving)] = VelocityInterpolationParam{0.2, 1.5, 2.0, 12.0};
    row[static_cast<std::size_t>(Motion::Static)] = VelocityInterpolationParam{0.2, 1.8, 4.0, 12.0};
  }
  s.object_type_specific_params[ObjectClassification::CAR] = class_params;
  s.object_type_specific_params[ObjectClassification::UNKNOWN] = class_params;

  auto & t = p.trajectory_polygon;
  t.decimate_trajectory_step_length = 2.0;
  t.goal_extended_trajectory_length = 6.0;
  t.ego_nearest_dist_threshold = 3.0;
  t.ego_nearest_yaw_threshold = 1.046;
  t.enable_to_consider_current_pose = true;
  t.time_to_convergence = 1.5;
  return p;
}

VehicleInfo make_vehicle_info()
{
  // width 1.8 m, front of the ego 3.8 m ahead of the rear axle
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.4, 0.2, 2.8, 1.6, 1.0, 1.0, 0.1, 0.1, 1.5, 0.6);
}

TrajectoryPoints make_straight_trajectory()
{
  TrajectoryPoints trajectory;
  for (double x = 0.0; x <= 100.0 + 1e-6; x += 1.0) {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = static_cast<float>(EGO_VELOCITY);
    trajectory.push_back(point);
  }
  return trajectory;
}

// 4 m x 2 m car standing still at (x, y), heading along the trajectory
PredictedObject make_car(
  const double x, const double y, const std::uint8_t id,
  const std::uint8_t label = ObjectClassification::CAR)
{
  PredictedObject object;
  object.object_id.uuid.fill(id);
  auto & pose = object.kinematics.initial_pose_with_covariance.pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;
  ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  return object;
}

class ObstacleSlowDownUtilsTest : public ::testing::Test
{
protected:
  std::vector<SlowDownInterval> plan(const std::vector<PredictedObject> & objects)
  {
    PredictedObjects msg;
    msg.objects = objects;
    geometry_msgs::msg::Pose ego_pose;
    ego_pose.orientation.w = 1.0;
    return planner_.plan(
      trajectory_, msg, ego_pose, EGO_VELOCITY, 0.0, vehicle_info_,
      rclcpp::Time(0, 0, RCL_ROS_TIME), params_);  // the clock of a stamp in a message
  }

  //! Runs the cycles needed to enter the slow down condition and returns the last output
  std::vector<SlowDownInterval> plan_until_entry(const std::vector<PredictedObject> & objects)
  {
    std::vector<SlowDownInterval> intervals;
    for (int i = 0; i < ENTRY_NUM; ++i) {
      intervals = plan(objects);
    }
    return intervals;
  }

  const VehicleInfo & vehicle_info() const { return vehicle_info_; }

private:
  SlowDownParameters params_{make_params()};
  VehicleInfo vehicle_info_{make_vehicle_info()};
  TrajectoryPoints trajectory_{make_straight_trajectory()};
  SlowDownPlanner planner_;
};

}  // namespace

TEST(ObstacleSlowDownValidateParameters, AcceptsLegacyDefaults)
{
  EXPECT_NO_THROW(validate_parameters(make_params()));
}

TEST(ObstacleSlowDownValidateParameters, RejectsInconsistentParameters)
{
  auto no_types = make_params();
  no_types.obstacle_filtering.object_types.clear();
  EXPECT_THROW(validate_parameters(no_types), std::invalid_argument);

  auto missing_class = make_params();
  missing_class.obstacle_filtering.object_types.push_back(ObjectClassification::PEDESTRIAN);
  EXPECT_THROW(validate_parameters(missing_class), std::invalid_argument);

  auto inverted_margin = make_params();
  inverted_margin.slow_down_planning.object_type_specific_params[ObjectClassification::CAR]
    .velocity_params[0][0]
    .max_lat_margin = 0.1;
  EXPECT_THROW(validate_parameters(inverted_margin), std::invalid_argument);
}

TEST_F(ObstacleSlowDownUtilsTest, SlowsDownAfterSuccessiveEntryCycles)
{
  const std::vector objects{make_car(OBJECT_X, NEAR_Y, 1)};
  for (int i = 0; i < ENTRY_NUM - 1; ++i) {
    EXPECT_TRUE(plan(objects).empty()) << "cycle " << i;
  }
  const auto intervals = plan(objects);
  ASSERT_EQ(intervals.size(), 1U);
  const auto & interval = intervals.front();
  EXPECT_EQ(interval.uuid, objects.front().object_id);
  // Nominal 5.5 m/s from the clearance 0.5 m; the jerk limited braking reaches only ~7.6 m/s
  // by the start of the interval
  EXPECT_GT(interval.velocity, 5.5);
  EXPECT_LT(interval.velocity, EGO_VELOCITY);
  // The ego front reaches the velocity before the car (rear end at x = 38) and keeps it until
  // the car (front end at x = 42) is passed
  EXPECT_LT(interval.start_s, OBJECT_X - 2.0 - vehicle_info().max_longitudinal_offset_m);
  EXPECT_GT(interval.end_s, interval.start_s);
  EXPECT_LE(interval.end_s, OBJECT_X + 2.0);
}

TEST_F(ObstacleSlowDownUtilsTest, IgnoresObjectsOutOfScope)
{
  // far beside the path, on the centerline (left to obstacle_stop), behind the ego, disabled class
  EXPECT_TRUE(plan_until_entry({make_car(OBJECT_X, 6.0, 1)}).empty());
  EXPECT_TRUE(plan_until_entry({make_car(OBJECT_X, 0.0, 2)}).empty());
  EXPECT_TRUE(plan_until_entry({make_car(-20.0, NEAR_Y, 3)}).empty());
  EXPECT_TRUE(
    plan_until_entry({make_car(OBJECT_X, NEAR_Y, 4, ObjectClassification::UNKNOWN)}).empty());
}

TEST_F(ObstacleSlowDownUtilsTest, ReleasesAfterSuccessiveExitCycles)
{
  ASSERT_EQ(plan_until_entry({make_car(OBJECT_X, NEAR_Y, 1)}).size(), 1U);

  // clearance 1.15 m: beyond the retention clearance 1.1 m, but still within the expanded band
  const std::vector away{make_car(OBJECT_X, 1.15 + 1.9, 1)};
  for (int i = 0; i < EXIT_NUM - 1; ++i) {
    EXPECT_EQ(plan(away).size(), 1U) << "cycle " << i;
  }
  EXPECT_TRUE(plan(away).empty());
}

TEST_F(ObstacleSlowDownUtilsTest, KeepsSlowingDownInsideHysteresis)
{
  ASSERT_EQ(plan_until_entry({make_car(OBJECT_X, NEAR_Y, 1)}).size(), 1U);

  // clearance 1.0 m: above the entry clearance 0.9 m, below the retention clearance 1.1 m
  const std::vector between{make_car(OBJECT_X, 1.0 + 1.9, 1)};
  for (int i = 0; i < EXIT_NUM + 1; ++i) {
    EXPECT_EQ(plan(between).size(), 1U) << "cycle " << i;
  }
}
