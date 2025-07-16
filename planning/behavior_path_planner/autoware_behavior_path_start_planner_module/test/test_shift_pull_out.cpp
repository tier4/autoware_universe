// Copyright 2024 TIER IV, Inc.
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

#include "start_planner_test_helper.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_path_start_planner_module/shift_pull_out.hpp>
#include <autoware/behavior_path_start_planner_module/start_planner_module.hpp>
#include <autoware/behavior_path_start_planner_module/util.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>
#include <matplotlibcpp17/pyplot.h>
#include <pybind11/pytypes.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using autoware::behavior_path_planner::ShiftPullOut;
using autoware::behavior_path_planner::StartPlannerParameters;
using autoware::test_utils::get_absolute_path_to_config;
using autoware_planning_msgs::msg::LaneletRoute;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
using autoware::behavior_path_planner::testing::StartPlannerTestHelper;
using autoware_planning_test_manager::utils::makeBehaviorRouteFromLaneId;

namespace autoware::behavior_path_planner
{

// Declaration of plot_footprint
void plot_footprint(
  matplotlibcpp17::axes::Axes & axes, const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & color,
  const double alpha);

void plot_path_with_lane_id(
  matplotlibcpp17::axes::Axes & axes,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const std::string & color = "red", const std::string & label = "", const double linewidth = 1.0,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info =
    autoware::vehicle_info_utils::VehicleInfo(),
  const bool draw_footprint = false)
{
  std::vector<double> xs, ys;
  std::vector<double> yaw_cos, yaw_sin;
  for (const auto & point : path.points) {
    xs.push_back(point.point.pose.position.x);
    ys.push_back(point.point.pose.position.y);
    const double yaw = autoware_utils::get_rpy(point.point.pose).z;
    yaw_cos.push_back(std::cos(yaw));
    yaw_sin.push_back(std::sin(yaw));
    axes.scatter(
      Args(xs.back(), ys.back()), Kwargs("marker"_a = "o", "color"_a = "blue", "s"_a = 10));

    // Draw footprint
    if (draw_footprint) {
      plot_footprint(axes, point.point.pose, vehicle_info, "blue", 0.1);
    }
  }
  axes.quiver(
    Args(xs, ys, yaw_cos, yaw_sin),
    Kwargs("angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 2.0));

  if (label == "") {
    axes.plot(Args(xs, ys), Kwargs("color"_a = color, "linewidth"_a = linewidth));
  } else {
    axes.plot(
      Args(xs, ys), Kwargs("color"_a = color, "linewidth"_a = linewidth, "label"_a = label));
  }
}

void plot_lanelet(
  matplotlibcpp17::axes::Axes & axes, lanelet::ConstLanelet lanelet,
  const std::string & color = "blue", const double linewidth = 0.5)
{
  const auto lefts = lanelet.leftBound();
  const auto rights = lanelet.rightBound();
  std::vector<double> xs_left, ys_left;
  for (const auto & point : lefts) {
    xs_left.push_back(point.x());
    ys_left.push_back(point.y());
  }

  std::vector<double> xs_right, ys_right;
  for (const auto & point : rights) {
    xs_right.push_back(point.x());
    ys_right.push_back(point.y());
  }

  std::vector<double> xs_center, ys_center;
  for (const auto & point : lanelet.centerline()) {
    xs_center.push_back(point.x());
    ys_center.push_back(point.y());
  }

  axes.plot(Args(xs_left, ys_left), Kwargs("color"_a = color, "linewidth"_a = linewidth));
  axes.plot(Args(xs_right, ys_right), Kwargs("color"_a = color, "linewidth"_a = linewidth));
  axes.plot(
    Args(xs_center, ys_center),
    Kwargs("color"_a = "black", "linewidth"_a = linewidth, "linestyle"_a = "dashed"));
}

void plot_footprint(
  matplotlibcpp17::axes::Axes & axes, const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & color,
  const double alpha)
{
  // Calculate vehicle footprint
  const double base_to_front = vehicle_info.front_overhang_m + vehicle_info.wheel_base_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;
  const double width = vehicle_info.vehicle_width_m;
  const double half_width = width / 2.0;

  // Relative coordinates of the four corners of the footprint
  std::vector<std::pair<double, double>> relative_points = {
    {base_to_front, half_width},   // Front right
    {base_to_front, -half_width},  // Front left
    {-base_to_rear, -half_width},  // Rear left
    {-base_to_rear, half_width},   // Rear right
  };

  // Calculate rotation matrix
  const double yaw = autoware_utils::get_rpy(pose).z;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  // Transform footprint points
  std::vector<double> xs, ys;
  for (const auto & point : relative_points) {
    // Rotation
    const double rotated_x = point.first * cos_yaw - point.second * sin_yaw;
    const double rotated_y = point.first * sin_yaw + point.second * cos_yaw;
    // Translation
    xs.push_back(rotated_x + pose.position.x);
    ys.push_back(rotated_y + pose.position.y);
  }
  // Add the first point at the end to close the polygon
  xs.push_back(xs.front());
  ys.push_back(ys.front());

  // Draw footprint
  axes.fill(Args(xs, ys), Kwargs("color"_a = color, "alpha"_a = alpha));
}

class TestShiftPullOut : public ::testing::Test
{
public:
  std::optional<PullOutPath> call_plan(
    const Pose & start_pose, const Pose & goal_pose,
    const std::shared_ptr<const PlannerData> & planner_data, PlannerDebugData & planner_debug_data)
  {
    return shift_pull_out_->plan(start_pose, goal_pose, planner_data, planner_debug_data);
  }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ =
      rclcpp::Node::make_shared("shift_pull_out", StartPlannerTestHelper::make_node_options());

    initialize_shift_pull_out_planner();
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Member variables
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<ShiftPullOut> shift_pull_out_;

private:
  void initialize_shift_pull_out_planner()
  {
    auto parameters = StartPlannerParameters::init(*node_);

    shift_pull_out_ = std::make_shared<ShiftPullOut>(*node_, parameters);
  }
};

TEST_F(TestShiftPullOut, GenerateValidShiftPullOutPath)
{
  const auto start_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(362.181).y(362.164).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.709650).w(
          0.704554));

  const auto goal_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(365.658).y(507.253).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.705897).w(
          0.708314));

  auto planner_data = std::make_shared<PlannerData>();
  planner_data->init_parameters(*node_);
  StartPlannerTestHelper::set_odometry(planner_data, start_pose);
  StartPlannerTestHelper::set_route(planner_data, 4619, 4635);
  // Plan the pull out path
  PlannerDebugData debug_data;
  auto result = call_plan(start_pose, goal_pose, planner_data, debug_data);

  // Assert that a valid shift pull out path is generated
  ASSERT_TRUE(result.has_value()) << "shift pull out path generation failed.";
  EXPECT_EQ(result->partial_paths.size(), 1UL)
    << "Generated shift pull out path does not have the expected number of partial paths.";
  EXPECT_EQ(debug_data.conditions_evaluation.back(), "success")
    << "shift pull out path planning did not succeed.";

  // Plot the generated path
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 1);
  auto & ax = axes[0];

  // Plot lanelets
  const auto & lanelets = planner_data->route_handler->getLaneletMapPtr()->laneletLayer;
  for (const auto & lanelet : lanelets) {
    plot_lanelet(ax, lanelet);
  }

  // Plot start and goal poses
  ax.plot(
    Args(start_pose.position.x, start_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "start", "markersize"_a = 20, "color"_a = "green"));
  ax.plot(
    Args(goal_pose.position.x, goal_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "goal", "markersize"_a = 20, "color"_a = "red"));

  // Plot footprints
  plot_footprint(ax, start_pose, planner_data->parameters.vehicle_info, "green", 0.3);
  plot_footprint(ax, goal_pose, planner_data->parameters.vehicle_info, "red", 0.3);

  // Plot generated path
  for (const auto & path : result->partial_paths) {
    plot_path_with_lane_id(
      ax, path, "blue", "generated path", 2.0, planner_data->parameters.vehicle_info, true);
  }

  // Set plot limits
  const double margin = 10.0;  // 10 meters margin
  const double x_min = std::min(start_pose.position.x, goal_pose.position.x) - margin;
  const double x_max = std::max(start_pose.position.x, goal_pose.position.x) + margin;
  const double y_min = std::min(start_pose.position.y, goal_pose.position.y) - margin;
  const double y_max = std::max(start_pose.position.y, goal_pose.position.y) + margin;
  ax.set_xlim(Args(x_min, x_max));
  ax.set_ylim(Args(y_min, y_max));

  ax.set_aspect(Args("equal"));
  ax.legend();
  plt.show(Args(), Kwargs("block"_a = true));
}

}  // namespace autoware::behavior_path_planner
