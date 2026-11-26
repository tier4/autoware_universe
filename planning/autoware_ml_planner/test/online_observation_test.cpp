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

#include "autoware/ml_planner/ml_planner_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>

namespace autoware::ml_planner
{
class OnlineObservationTest : public ::testing::Test
{
protected:
  static constexpr int64_t second = 1'000'000'000;

  static rclcpp::NodeOptions node_options()
  {
    const auto params = ament_index_cpp::get_package_share_directory("autoware_test_utils") +
                        "/config/test_vehicle_info.param.yaml";
    return rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", params});
  }

  void SetUp() override
  {
    node_ = std::make_shared<MLPlanner>(node_options());
    ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
  }

  void receive_odometry(int receipt, int stamp, double speed)
  {
    ASSERT_EQ(
      rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), receipt * second),
      RCL_RET_OK);
    auto msg = std::make_shared<Odometry>();
    msg->header.frame_id = "map";
    msg->header.stamp.sec = stamp;
    msg->child_frame_id = "base_link";
    msg->pose.pose.orientation.w = 1.0;
    msg->twist.twist.linear.x = speed;
    std::shared_ptr<void> data = msg;
    node_->sub_current_odometry_->handle_message(data, rclcpp::MessageInfo{});
  }

  std::optional<double> speed_at(int measurement, int receipt_cutoff) const
  {
    const auto anchor = rclcpp::Time(node_->odometry_->header.stamp).nanoseconds();
    const auto window =
      node_->scene_builder_->build_window(anchor, {"test"}, receipt_cutoff * second);
    const auto offset = (anchor - measurement * second) / 100'000'000;
    return window.ego->velocities_3d->get(3 * (window.current_index - offset));
  }

  int anchor() const { return node_->odometry_->header.stamp.sec; }

  void receive_objects(int receipt, int stamp, uint32_t nanosecond)
  {
    ASSERT_EQ(
      rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), receipt * second),
      RCL_RET_OK);
    auto msg = std::make_shared<TrackedObjects>();
    msg->header.frame_id = "map";
    msg->header.stamp.sec = stamp;
    msg->header.stamp.nanosec = nanosecond;
    std::shared_ptr<void> data = msg;
    node_->sub_tracked_objects_->handle_message(data, rclcpp::MessageInfo{});
  }

  void prune_history() { node_->discard_scene_history(); }

  std::optional<int64_t> oldest_tracking_stamp(int receipt_cutoff) const
  {
    const auto scene = node_->scene_builder_->build_window(
      anchor() * second, {"tracking-test"}, receipt_cutoff * second);
    return scene.annotations->timestamps_ns->get(0);
  }

  void receive_route(int receipt, int stamp, int64_t lane_id)
  {
    ASSERT_EQ(
      rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), receipt * second),
      RCL_RET_OK);
    auto msg = std::make_shared<LaneletRoute>();
    msg->header.frame_id = "map";
    msg->header.stamp.sec = stamp;
    msg->goal_pose.orientation.w = 1.0;
    msg->goal_pose.position.x = static_cast<double>(lane_id);
    auto & segment = msg->segments.emplace_back();
    segment.preferred_primitive.id = lane_id;
    segment.primitives.push_back(segment.preferred_primitive);
    std::shared_ptr<void> data = msg;
    node_->sub_route_->handle_message(data, rclcpp::MessageInfo{});
  }

  auto route_window(int receipt_cutoff) const
  {
    return node_->scene_builder_->build_window_with_route(
      anchor() * second, {"route-test"}, receipt_cutoff * second);
  }

  void receive_acceleration(int receipt, int stamp, double longitudinal, double lateral = 0.0)
  {
    ASSERT_EQ(
      rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), receipt * second),
      RCL_RET_OK);
    auto msg = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
    msg->header.stamp.sec = stamp;
    msg->accel.accel.linear.x = longitudinal;
    msg->accel.accel.linear.y = lateral;
    std::shared_ptr<void> data = msg;
    node_->sub_acceleration_->handle_message(data, rclcpp::MessageInfo{});
  }

  std::optional<double> longitudinal_acceleration() const
  {
    return node_->longitudinal_acceleration_;
  }

  bool has_scene_acceleration(int receipt_cutoff) const
  {
    const auto scene = node_->scene_builder_->build_window(
      anchor() * second, {"acceleration-test"}, receipt_cutoff * second);
    return scene.ego->accelerations_3d.has_value();
  }

  std::shared_ptr<MLPlanner> node_;
};

TEST_F(OnlineObservationTest, BackfillsMissingMeasurementsAtTheirReceiptCutoff)
{
  receive_odometry(20, 10, 10.0);
  receive_odometry(21, 9, 9.0);
  receive_odometry(22, 9, 99.0);
  EXPECT_EQ(anchor(), 10);
  EXPECT_FALSE(speed_at(9, 20).has_value());
  EXPECT_EQ(speed_at(9, 21), 9.0);
  EXPECT_EQ(speed_at(9, 22), 99.0);
  EXPECT_EQ(speed_at(10, 22), 10.0);

  receive_odometry(18, 8, 8.0);
  EXPECT_FALSE(speed_at(8, 22).has_value());
  receive_odometry(23, 11, 11.0);
  EXPECT_EQ(anchor(), 11);
  EXPECT_EQ(speed_at(9, 23), 99.0);
}

TEST_F(OnlineObservationTest, ReappliesTheMainRouteWithItsOriginalStamp)
{
  receive_odometry(20, 20, 2.0);
  receive_route(20, 10, 101);
  receive_route(21, 16, 202);
  receive_route(22, 10, 101);

  const auto earlier = route_window(21);
  ASSERT_TRUE(earlier.active_route);
  EXPECT_EQ(earlier.active_route->goal_pose.position.x, 202.0);

  const auto restored = route_window(22);
  ASSERT_TRUE(restored.active_route);
  EXPECT_EQ(restored.active_route->header.stamp.sec, 10);
  EXPECT_EQ(restored.active_route->goal_pose.position.x, 101.0);
  const auto & routes = restored.scene.routes.value();
  const auto & current = routes.rows.at(routes.indices.at(restored.scene.current_index));
  EXPECT_EQ(current.segments.at(0).preferred_lane_id, 101);
}

TEST_F(OnlineObservationTest, PreservesTrackingAtTheEarliestHistoryRow)
{
  receive_odometry(20, 10, 2.0);
  // The first row is at 6.4 s. A 5.4 s measurement is still eligible there.
  receive_objects(21, 5, 400'000'000);
  EXPECT_EQ(oldest_tracking_stamp(21), 5'400'000'000);
  prune_history();
  EXPECT_EQ(oldest_tracking_stamp(21), 5'400'000'000);
}

TEST_F(OnlineObservationTest, RetainsAccelerationOnlyForTheOptimizer)
{
  receive_acceleration(19, 9, 7.0);
  EXPECT_FALSE(longitudinal_acceleration().has_value());

  receive_odometry(20, 10, 2.0);
  receive_acceleration(21, 5, 6.0);
  EXPECT_FALSE(longitudinal_acceleration().has_value());

  receive_acceleration(21, 9, 2.0);
  receive_acceleration(22, 9, 7.0);
  receive_acceleration(22, 10, 8.0, std::numeric_limits<double>::infinity());
  EXPECT_EQ(longitudinal_acceleration(), 7.0);

  receive_acceleration(23, 10, 3.0);
  EXPECT_EQ(longitudinal_acceleration(), 3.0);
  EXPECT_FALSE(has_scene_acceleration(23));
}

TEST_F(OnlineObservationTest, RejectsInvalidTrafficLightTimeoutAtStartupAndRuntime)
{
  constexpr auto name = "traffic_light_group_msg_timeout_seconds";
  for (const double invalid :
       {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    auto options = node_options();
    options.append_parameter_override(name, invalid);
    EXPECT_THROW(std::make_shared<MLPlanner>(options), std::invalid_argument);
    EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter(name, invalid)).successful);
    EXPECT_DOUBLE_EQ(node_->get_parameter(name).as_double(), 0.2);
  }
  EXPECT_TRUE(node_->set_parameter(rclcpp::Parameter(name, 0.0)).successful);
  EXPECT_DOUBLE_EQ(node_->get_parameter(name).as_double(), 0.0);
}
}  // namespace autoware::ml_planner
