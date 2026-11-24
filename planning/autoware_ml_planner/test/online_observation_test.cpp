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
#include <memory>
#include <optional>

namespace autoware::ml_planner
{
class OnlineObservationTest : public ::testing::Test
{
protected:
  static constexpr int64_t second = 1'000'000'000;

  void SetUp() override
  {
    const auto params = ament_index_cpp::get_package_share_directory("autoware_test_utils") +
                        "/config/test_vehicle_info.param.yaml";
    node_ = std::make_shared<MLPlanner>(
      rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", params}));
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
  EXPECT_EQ(speed_at(9, 22), 9.0);
  EXPECT_EQ(speed_at(10, 22), 10.0);

  receive_odometry(18, 8, 8.0);
  EXPECT_FALSE(speed_at(8, 22).has_value());
  receive_odometry(23, 11, 11.0);
  EXPECT_EQ(anchor(), 11);
  EXPECT_EQ(speed_at(9, 23), 9.0);
}
}  // namespace autoware::ml_planner
