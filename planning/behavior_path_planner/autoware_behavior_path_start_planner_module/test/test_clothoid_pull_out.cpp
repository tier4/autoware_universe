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
#include <autoware/behavior_path_start_planner_module/clothoid_pull_out.hpp>
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
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using autoware::behavior_path_planner::ClothoidPullOut;
using autoware::behavior_path_planner::combinePathWithCenterline;
using autoware::behavior_path_planner::createPathWithLaneIdFromClothoidPaths;
using autoware::behavior_path_planner::StartPlannerParameters;
using autoware::test_utils::get_absolute_path_to_config;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
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

void plot_velocity_acceleration(
  matplotlibcpp17::axes::Axes & axes,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  if (path.points.empty()) {
    axes.text(
      Args(0.5, 0.5, "No path data available"), Kwargs("ha"_a = "center", "va"_a = "center"));
    axes.set_title(Args("Velocity & Acceleration (No Data)"));
    return;
  }

  std::vector<double> arc_lengths;
  std::vector<double> velocities;

  // 弧長と速度を計算
  arc_lengths.push_back(0.0);
  double cumulative_length = 0.0;

  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & point = path.points[i];
    velocities.push_back(point.point.longitudinal_velocity_mps);

    if (i > 0) {
      // 弧長を計算
      double dx = point.point.pose.position.x - path.points[i - 1].point.pose.position.x;
      double dy = point.point.pose.position.y - path.points[i - 1].point.pose.position.y;
      cumulative_length += std::sqrt(dx * dx + dy * dy);
      arc_lengths.push_back(cumulative_length);
    }
  }

  // 速度を点でプロット
  axes.scatter(
    Args(arc_lengths, velocities),
    Kwargs("color"_a = "blue", "s"_a = 20, "label"_a = "Velocity [m/s]", "alpha"_a = 0.7));

  axes.set_xlabel(Args("Arc Length [m]"));
  axes.set_ylabel(Args("Velocity [m/s]"));
  axes.set_title(Args("Velocity Profile"));
  axes.grid(Args(true), Kwargs("alpha"_a = 0.3));
  axes.legend();
}

class TestClothoidPullOut : public ::testing::Test
{
public:
  std::optional<PullOutPath> call_plan(
    const Pose & start_pose, const Pose & goal_pose,
    const std::shared_ptr<const PlannerData> & planner_data, PlannerDebugData & planner_debug_data)
  {
    return clothoid_pull_out_->plan(start_pose, goal_pose, planner_data, planner_debug_data);
  }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ =
      rclcpp::Node::make_shared("clothoid_pull_out", StartPlannerTestHelper::make_node_options());

    initialize_clothoid_pull_out_planner();
  }

  void TearDown() override { rclcpp::shutdown(); }

  // Member variables
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<ClothoidPullOut> clothoid_pull_out_;

private:
  void initialize_clothoid_pull_out_planner()
  {
    auto parameters = StartPlannerParameters::init(*node_);

    clothoid_pull_out_ = std::make_shared<ClothoidPullOut>(*node_, parameters);
  }
};

TEST_F(TestClothoidPullOut, DISABLED_GenerateValidClothoidPullOutPath)
{
  const auto start_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(362.181).y(362.164).z(100.000))
      .orientation(
        geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.709650).w(
          0.704554));
  // const auto start_pose =
  //   geometry_msgs::build<geometry_msgs::msg::Pose>()
  //     .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(299.462).y(354.701).z(100.000))
  //     .orientation(
  //       geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(-0.769919).w(
  //         0.638141));
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
  // StartPlannerTestHelper::set_route(planner_data, 675, 720);
  // Plan the pull out path
  PlannerDebugData debug_data;
  std::cerr << "Planning clothoid pull out path..." << std::endl;
  auto result = call_plan(start_pose, goal_pose, planner_data, debug_data);

  // Assert that a valid clothoid pull out path is generated
  // ASSERT_TRUE(result.has_value()) << "clothoid pull out path generation failed.";
  // EXPECT_EQ(result->partial_paths.size(), 1UL)
  //   << "Generated clothoid pull out path does not have the expected number of partial paths.";
  // EXPECT_EQ(debug_data.conditions_evaluation.back(), "success")
  //   << "clothoid pull out path planning did not succeed.";

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

  // タイトルに座標範囲情報を追加
  std::string title = "Circular Path vs Clothoid Path Comparison\n";
  title += "X: [" + std::to_string(static_cast<int>(x_min)) + ", " +
           std::to_string(static_cast<int>(x_max)) + "] m, ";
  title += "Y: [" + std::to_string(static_cast<int>(y_min)) + ", " +
           std::to_string(static_cast<int>(y_max)) + "] m";
  ax.set_title(Args(title));
  ax.set_xlabel(Args("X [m]"));
  ax.set_ylabel(Args("Y [m]"));
  ax.legend();

  // グリッドを追加して視認性を向上
  ax.grid(Args(true), Kwargs("alpha"_a = 0.3));

  plt.show(Args(), Kwargs("block"_a = true));

  std::cerr << "=======================================" << std::endl;
}

/**
 * @brief 複数セグメント対応の改良版クロソイド変換関数
 */
std::vector<std::vector<geometry_msgs::msg::Point>> convertMultipleArcsToClothoidWithCorrection(
  const std::vector<ArcSegment> & arc_segments, const geometry_msgs::msg::Pose & initial_start_pose,
  const std::vector<double> & A_values, const std::vector<double> & L_values, double velocity,
  double wheel_base, double max_steer_angle_rate, double point_interval)
{
  std::vector<std::vector<geometry_msgs::msg::Point>> corrected_clothoid_paths;

  // セグメント間の連続性を保つための姿勢管理
  geometry_msgs::msg::Pose current_segment_pose = initial_start_pose;

  for (size_t i = 0; i < arc_segments.size(); ++i) {
    const auto & segment = arc_segments[i];

    std::cerr << "\n--- Converting Arc Segment " << (i + 1) << "/" << arc_segments.size()
              << " with Correction ---" << std::endl;

    // パラメータの取得
    double A_min = (i < A_values.size()) ? A_values[i] : 50.0;
    double L_min = (i < L_values.size()) ? L_values[i] : 10.0;

    // 補正付きクロソイド変換を実行
    auto corrected_clothoid_points = convertArcToClothoidWithCorrection(
      segment, current_segment_pose, velocity, wheel_base, max_steer_angle_rate, point_interval);

    if (!corrected_clothoid_points.empty()) {
      corrected_clothoid_paths.push_back(corrected_clothoid_points);

      // 次のセグメントのために終点姿勢を更新
      if (i < arc_segments.size() - 1) {
        const auto & last_point = corrected_clothoid_points.back();

        // 終点での進行方向を計算（最後の2点から）
        if (corrected_clothoid_points.size() >= 2) {
          const auto & second_last =
            corrected_clothoid_points[corrected_clothoid_points.size() - 2];
          double dx = last_point.x - second_last.x;
          double dy = last_point.y - second_last.y;
          double heading = std::atan2(dy, dx);

          current_segment_pose.position = last_point;
          current_segment_pose.orientation =
            tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));

          std::cerr << "Updated pose for next segment:" << std::endl;
          std::cerr << "  Position: (" << current_segment_pose.position.x << ", "
                    << current_segment_pose.position.y << ")" << std::endl;
          std::cerr << "  Heading: " << heading << " rad (" << heading * 180.0 / M_PI << " deg)"
                    << std::endl;
        }
      }
    } else {
      std::cerr << "Failed to convert segment " << (i + 1) << " to clothoid with correction"
                << std::endl;
    }
  }

  return corrected_clothoid_paths;
}

TEST_F(TestClothoidPullOut, DISABLED_PlotCircularPathGeneration)
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

  const auto & route_handler = planner_data->route_handler;
  const auto & common_parameters = planner_data->parameters;

  const double backward_path_length =
    planner_data->parameters.backward_path_length + 10.0;  // max_back_distance = 10.0と仮定
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  // Generate centerline path from road_lanes
  const auto centerline_path = utils::getCenterLinePath(
    *route_handler, road_lanes, start_pose, backward_path_length,
    std::numeric_limits<double>::max(), common_parameters);

  // Calculate lateral offset
  const double lateral_offset =
    centerline_path.points.empty()
      ? 0.0
      : autoware::motion_utils::calcLateralOffset(centerline_path.points, start_pose.position);

  const double max_steer_angle_deg = 20.0;
  const double max_steer_angle = max_steer_angle_deg * M_PI / 180.0;
  const double max_steer_angle_rate_deg_per_sec = 10.0;
  const double max_steer_angle_rate = max_steer_angle_rate_deg_per_sec * M_PI / 180.0;
  const double velocity = 1.0;
  const double wheel_base = planner_data->parameters.vehicle_info.wheel_base_m;
  // const double minimum_radius = 13.46;
  const double minimum_radius = wheel_base / std::tan(max_steer_angle);

  // longitudinal necessary distance for pull out
  const double longitudinal_distance =
    start_planner_utils::calc_necessary_longitudinal_distance(-lateral_offset, minimum_radius);

  const Pose target_pose = start_planner_utils::findTargetPoseAlongPath(
    centerline_path, start_pose, longitudinal_distance);

  const auto relative_pose_info =
    start_planner_utils::calculateRelativePoseInVehicleCoordinate(start_pose, target_pose);

  const auto circular_path = start_planner_utils::calc_circular_path(
    start_pose, relative_pose_info.longitudinal_distance_vehicle,
    relative_pose_info.lateral_distance_vehicle, relative_pose_info.angle_diff, minimum_radius);

  ASSERT_FALSE(circular_path.segments.empty()) << "Circular path generation failed.";

  // 経路点を生成
  std::vector<std::pair<double, double>> path_points;
  const int points_per_segment = 50;

  // 角度変化の計算
  double total_angle_change = 0.0;

  for (const auto & segment : circular_path.segments) {
    const double circular_steer_angle = std::atan(wheel_base / segment.radius);
    const double circular_steer_angle_deg = circular_steer_angle * 180.0 / M_PI;
    std::cerr << "circular_steer_angle_deg: " << circular_steer_angle_deg << std::endl;
    const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
    const double L_min = velocity * minimum_steer_time;
    const double A_min = std::sqrt(segment.radius * L_min);
    const double alpha_clothoid = (L_min * L_min) / (2.0 * A_min * A_min);
    std::cerr << "L_min: " << L_min << std::endl;
    std::cerr << "A_min: " << A_min << std::endl;
    std::cerr << "alpha_clothoid: " << alpha_clothoid << std::endl;

    // 各セグメントの角度変化を計算
    double start_angle = segment.getStartAngle();
    double end_angle = segment.getEndAngle();
    double segment_angle_change;

    if (segment.is_clockwise) {
      segment_angle_change = end_angle - start_angle;
      if (segment_angle_change > 0) {
        segment_angle_change -= 2 * M_PI;
      }
    } else {
      segment_angle_change = end_angle - start_angle;
      if (segment_angle_change < 0) {
        segment_angle_change += 2 * M_PI;
      }
    }

    total_angle_change += segment_angle_change;

    for (int i = 0; i < points_per_segment; ++i) {
      if (!path_points.empty() && i == 0) {
        continue;
      }

      double progress = static_cast<double>(i) / (points_per_segment - 1);

      double current_angle;

      if (segment.is_clockwise) {
        double angle_diff_seg = end_angle - start_angle;
        if (angle_diff_seg > 0) {
          angle_diff_seg -= 2 * M_PI;
        }
        current_angle = start_angle + angle_diff_seg * progress;
      } else {
        double angle_diff_seg = end_angle - start_angle;
        if (angle_diff_seg < 0) {
          angle_diff_seg += 2 * M_PI;
        }
        current_angle = start_angle + angle_diff_seg * progress;
      }

      auto point = segment.getPointAtAngle(current_angle);
      path_points.push_back(std::make_pair(point.x, point.y));
    }
  }

  const auto trajectory = start_planner_utils::convertCircularPathToTrajectory(circular_path);
  const auto curvatures = start_planner_utils::calcCurvatureFromTrajectory(trajectory);

  std::vector<std::vector<geometry_msgs::msg::Point>> clothoid_paths;

  // セグメント間の連続性を保つための姿勢管理
  geometry_msgs::msg::Pose current_segment_pose = start_pose;

  for (size_t i = 0; i < circular_path.segments.size(); ++i) {
    const auto & segment = circular_path.segments[i];

    // 車両パラメータから最適なクロソイドパラメータを計算
    const double circular_steer_angle = std::atan(wheel_base / segment.radius);
    const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
    const double L_min = velocity * minimum_steer_time;
    const double A_min = std::sqrt(segment.radius * L_min);

    // クロソイド変換を実行
    auto clothoid_points = convertArcToClothoidWithCorrection(
      segment, current_segment_pose, velocity, wheel_base, max_steer_angle_rate,
      0.5);  // point_interval

    if (!clothoid_points.empty()) {
      clothoid_paths.push_back(clothoid_points);

      // 次のセグメントのために終点姿勢を更新
      if (i < circular_path.segments.size() - 1) {
        const auto & last_point = clothoid_points.back();

        // 終点での進行方向を計算（最後の2点から）
        if (clothoid_points.size() >= 2) {
          const auto & second_last = clothoid_points[clothoid_points.size() - 2];
          double dx = last_point.x - second_last.x;
          double dy = last_point.y - second_last.y;
          double heading = std::atan2(dy, dx);

          current_segment_pose.position = last_point;
          current_segment_pose.orientation =
            tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));

          std::cerr << "Updated pose for next segment:" << std::endl;
          std::cerr << "  Position: (" << current_segment_pose.position.x << ", "
                    << current_segment_pose.position.y << ")" << std::endl;
          std::cerr << "  Heading: " << heading << " rad (" << heading * 180.0 / M_PI << " deg)"
                    << std::endl;
        }
      }
    } else {
      std::cerr << "Failed to convert segment " << (i + 1) << " to clothoid" << std::endl;
    }
  }

  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  std::vector<geometry_msgs::msg::Point> combined_clothoid_path;
  for (size_t i = 0; i < clothoid_paths.size(); ++i) {
    const auto & clothoid_path = clothoid_paths[i];

    // 最初のセグメント以外は開始点を除いて結合（重複回避）
    size_t start_idx = (i == 0) ? 0 : 1;
    for (size_t j = start_idx; j < clothoid_path.size(); ++j) {
      combined_clothoid_path.push_back(clothoid_path[j]);
    }
  }

  // createPathWithLaneIdFromClothoidPaths関数を呼び出してPathWithLaneIdを生成
  auto parameters = StartPlannerParameters::init(*node_);
  PathWithLaneId path_with_lane_id = createPathWithLaneIdFromClothoidPaths(
    clothoid_paths, target_pose, velocity, velocity, 0.0, road_lanes, route_handler);

  auto combined_path = combinePathWithCenterline(path_with_lane_id, centerline_path, target_pose);

  // 曲率計算データの準備
  std::vector<double> arc_lengths;
  std::vector<double> curvature_values;
  std::vector<double> curvature_changes;
  std::vector<double> arc_lengths_changes;
  bool has_curvature_data = false;

  // 結合されたクロソイド経路の曲率を計算
  auto combined_curvatures =
    autoware::behavior_path_planner::start_planner_utils::calcCurvatureFromPoints(
      combined_clothoid_path);

  // 各点の曲率をデバッグプリント
  for (size_t i = 0; i < combined_curvatures.size(); ++i) {
    const auto & point = combined_clothoid_path[i];
    const double curvature = combined_curvatures[i];

    std::cerr << "Point[" << i << "]: "
              << "pos=(" << std::fixed << std::setprecision(3) << point.x << ", " << point.y
              << "), "
              << "curvature=" << std::setprecision(6) << curvature << " (1/m)" << std::endl;
  }

  // 弧長を計算
  arc_lengths.push_back(0.0);
  double cumulative_length = 0.0;
  for (size_t i = 1; i < combined_clothoid_path.size(); ++i) {
    double dx = combined_clothoid_path[i].x - combined_clothoid_path[i - 1].x;
    double dy = combined_clothoid_path[i].y - combined_clothoid_path[i - 1].y;
    cumulative_length += std::sqrt(dx * dx + dy * dy);
    arc_lengths.push_back(cumulative_length);
  }

  // 曲率値をコピー
  for (size_t i = 0; i < combined_curvatures.size(); ++i) {
    curvature_values.push_back(combined_curvatures[i]);
  }

  has_curvature_data = true;

  // 横並びプロット作成: 左に経路、中央に曲率分析、右に速度・加速度
  auto [fig, axes] = plt.subplots(1, 3, Kwargs("figsize"_a = std::make_tuple(24, 6)));
  auto & ax_path = axes[0];
  auto & ax_curvature = axes[1];
  auto & ax_velocity = axes[2];

  // ============================================================================
  // 左側: 経路プロット
  // ============================================================================

  // レーンレットをプロット
  const auto & lanelets = planner_data->route_handler->getLaneletMapPtr()->laneletLayer;
  for (const auto & lanelet : lanelets) {
    plot_lanelet(ax_path, lanelet);
  }

  // 開始姿勢と目標姿勢をプロット
  ax_path.plot(
    Args(start_pose.position.x, start_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "start", "markersize"_a = 20, "color"_a = "green"));
  ax_path.plot(
    Args(target_pose.position.x, target_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "target", "markersize"_a = 20, "color"_a = "red"));

  // 元の円弧経路の点をプロット
  std::vector<double> xs, ys;
  for (const auto & point : path_points) {
    xs.push_back(point.first);
    ys.push_back(point.second);
  }

  ax_path.scatter(Args(xs, ys), Kwargs("color"_a = "blue", "s"_a = 10, "alpha"_a = 0.6));

  // クロソイド経路をプロット
  for (size_t i = 0; i < clothoid_paths.size(); ++i) {
    const auto & clothoid_path = clothoid_paths[i];

    std::vector<double> clothoid_xs, clothoid_ys;
    for (const auto & point : clothoid_path) {
      clothoid_xs.push_back(point.x);
      clothoid_ys.push_back(point.y);
    }

    // クロソイド経路を異なる色で描画（線分を削除して点のみ表示）
    std::string color = (i % 2 == 0) ? "red" : "purple";
    std::string label = (i == 0) ? "clothoid path" : "";

    // クロソイド経路の点をscatterで描画
    ax_path.scatter(
      Args(clothoid_xs, clothoid_ys),
      Kwargs("color"_a = color, "s"_a = 15, "label"_a = label, "alpha"_a = 0.8));

    // クロソイド経路の開始点と終了点をマーク
    if (!clothoid_path.empty()) {
      ax_path.plot(
        Args(clothoid_path.front().x, clothoid_path.front().y),
        Kwargs("marker"_a = "o", "color"_a = color, "markersize"_a = 8, "alpha"_a = 0.9));
      ax_path.plot(
        Args(clothoid_path.back().x, clothoid_path.back().y),
        Kwargs("marker"_a = "s", "color"_a = color, "markersize"_a = 8, "alpha"_a = 0.9));
    }
  }

  // PathWithLaneIdをプロット
  if (!path_with_lane_id.points.empty()) {
    plot_path_with_lane_id(ax_path, combined_path, "green", "PathWithLaneId", 3.0);
  }

  // 円弧セグメントの中心点をプロット
  for (size_t i = 0; i < circular_path.segments.size(); ++i) {
    const auto & segment = circular_path.segments[i];
    ax_path.plot(
      Args(segment.center.x, segment.center.y),
      Kwargs(
        "marker"_a = "o", "color"_a = "orange", "markersize"_a = 8,
        "label"_a = (i == 0 ? "arc centers" : "")));
  }

  // プロット範囲を設定
  const double margin = 5.0;

  // PathWithLaneIdの点から範囲を計算
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  // PathWithLaneIdの点のみをチェック
  if (!combined_path.points.empty()) {
    for (const auto & point : combined_path.points) {
      x_min = std::min(x_min, point.point.pose.position.x);
      x_max = std::max(x_max, point.point.pose.position.x);
      y_min = std::min(y_min, point.point.pose.position.y);
      y_max = std::max(y_max, point.point.pose.position.y);
    }
  }

  // 余裕を追加
  x_min -= margin;
  x_max += margin;
  y_min -= margin;
  y_max += margin;

  ax_path.set_xlim(Args(x_min, x_max));
  ax_path.set_ylim(Args(y_min, y_max));
  ax_path.set_aspect(Args("equal"));
  ax_path.grid(Args(true), Kwargs("alpha"_a = 0.3));
  ax_path.set_title(Args("Path Comparison"));
  ax_path.set_xlabel(Args("X [m]"));
  ax_path.set_ylabel(Args("Y [m]"));
  ax_path.legend();

  // ============================================================================
  // 中央: 曲率分析プロット
  // ============================================================================

  if (has_curvature_data && !curvature_values.empty()) {
    // 曲率 vs 弧長をプロット
    ax_curvature.plot(
      Args(arc_lengths, curvature_values),
      Kwargs("color"_a = "blue", "linewidth"_a = 2.0, "label"_a = "Curvature"));

    ax_curvature.set_xlabel(Args("Arc Length [m]"));
    ax_curvature.set_ylabel(Args("Curvature [1/m]"));
    ax_curvature.set_title(Args("Clothoid Path Curvature"));
    ax_curvature.grid(Args(true), Kwargs("alpha"_a = 0.3));
    ax_curvature.legend();

  } else {
    // 曲率データがない場合のメッセージ表示
    ax_curvature.text(
      Args(0.5, 0.5, "Insufficient points for\ncurvature calculation"),
      Kwargs("ha"_a = "center", "va"_a = "center"));
    ax_curvature.set_title(Args("Curvature Analysis (No Data)"));
  }

  // ============================================================================
  // 右側: 速度・加速度分析プロット
  // ============================================================================

  if (!combined_path.points.empty()) {
    plot_velocity_acceleration(ax_velocity, combined_path);
    std::cerr << "Velocity and acceleration plotted for PathWithLaneId" << std::endl;
  } else {
    ax_velocity.text(
      Args(0.5, 0.5, "No PathWithLaneId data\navailable for velocity analysis"),
      Kwargs("ha"_a = "center", "va"_a = "center"));
    ax_velocity.set_title(Args("Velocity & Acceleration (No Data)"));
  }

  // レイアウトを調整して表示
  fig.tight_layout();
  plt.show(Args(), Kwargs("block"_a = true));
}

TEST_F(TestClothoidPullOut, PlotPathInShiojiri)
{
  const auto start_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                            .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                                        .x(65397.40625)
                                        .y(684.406005859375)
                                        .z(758.714))
                            .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                                           .x(0.0)
                                           .y(0.0)
                                           .z(-0.0018264627034187376)
                                           .w(0.9999983320156054));

  const auto goal_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                           .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                                       .x(65486.0078125)
                                       .y(680.9390258789062)
                                       .z(757.8801237939126))
                           .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                                          .x(0.0)
                                          .y(0.0)
                                          .z(0.04829264604882148)
                                          .w(0.9988332294920925));

  auto planner_data = std::make_shared<PlannerData>();
  planner_data->init_parameters(*node_);
  StartPlannerTestHelper::set_odometry(planner_data, start_pose);
  StartPlannerTestHelper::set_route(planner_data, 94791, 23);

  const auto & route_handler = planner_data->route_handler;
  const auto & common_parameters = planner_data->parameters;

  const double backward_path_length =
    planner_data->parameters.backward_path_length + 10.0;  // max_back_distance = 10.0と仮定
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  // Generate centerline path from road_lanes
  const auto centerline_path = utils::getCenterLinePath(
    *route_handler, road_lanes, start_pose, backward_path_length,
    std::numeric_limits<double>::max(), common_parameters);

  // Calculate lateral offset
  const double lateral_offset =
    centerline_path.points.empty()
      ? 0.0
      : autoware::motion_utils::calcLateralOffset(centerline_path.points, start_pose.position);

  const double max_steer_angle_deg = 20.0;
  const double max_steer_angle = max_steer_angle_deg * M_PI / 180.0;
  const double max_steer_angle_rate_deg_per_sec = 10.0;
  const double max_steer_angle_rate = max_steer_angle_rate_deg_per_sec * M_PI / 180.0;
  const double velocity = 1.0;
  const double wheel_base = planner_data->parameters.vehicle_info.wheel_base_m;
  // const double minimum_radius = 13.46;
  const double minimum_radius = wheel_base / std::tan(max_steer_angle);

  // longitudinal necessary distance for pull out
  const double longitudinal_distance =
    start_planner_utils::calc_necessary_longitudinal_distance(-lateral_offset, minimum_radius);

  const Pose target_pose = start_planner_utils::findTargetPoseAlongPath(
    centerline_path, start_pose, longitudinal_distance);

  const auto relative_pose_info =
    start_planner_utils::calculateRelativePoseInVehicleCoordinate(start_pose, target_pose);

  const auto circular_path = start_planner_utils::calc_circular_path(
    start_pose, relative_pose_info.longitudinal_distance_vehicle,
    relative_pose_info.lateral_distance_vehicle, relative_pose_info.angle_diff, minimum_radius);

  ASSERT_FALSE(circular_path.segments.empty()) << "Circular path generation failed.";

  // 経路点を生成
  std::vector<std::pair<double, double>> path_points;
  const int points_per_segment = 50;

  // 角度変化の計算
  double total_angle_change = 0.0;

  for (const auto & segment : circular_path.segments) {
    const double circular_steer_angle = std::atan(wheel_base / segment.radius);
    const double circular_steer_angle_deg = circular_steer_angle * 180.0 / M_PI;
    std::cerr << "circular_steer_angle_deg: " << circular_steer_angle_deg << std::endl;
    const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
    const double L_min = velocity * minimum_steer_time;
    const double A_min = std::sqrt(segment.radius * L_min);
    const double alpha_clothoid = (L_min * L_min) / (2.0 * A_min * A_min);
    std::cerr << "L_min: " << L_min << std::endl;
    std::cerr << "A_min: " << A_min << std::endl;
    std::cerr << "alpha_clothoid: " << alpha_clothoid << std::endl;

    // 各セグメントの角度変化を計算
    double start_angle = segment.getStartAngle();
    double end_angle = segment.getEndAngle();
    double segment_angle_change;

    if (segment.is_clockwise) {
      segment_angle_change = end_angle - start_angle;
      if (segment_angle_change > 0) {
        segment_angle_change -= 2 * M_PI;
      }
    } else {
      segment_angle_change = end_angle - start_angle;
      if (segment_angle_change < 0) {
        segment_angle_change += 2 * M_PI;
      }
    }

    total_angle_change += segment_angle_change;

    for (int i = 0; i < points_per_segment; ++i) {
      if (!path_points.empty() && i == 0) {
        continue;
      }

      double progress = static_cast<double>(i) / (points_per_segment - 1);

      double current_angle;

      if (segment.is_clockwise) {
        double angle_diff_seg = end_angle - start_angle;
        if (angle_diff_seg > 0) {
          angle_diff_seg -= 2 * M_PI;
        }
        current_angle = start_angle + angle_diff_seg * progress;
      } else {
        double angle_diff_seg = end_angle - start_angle;
        if (angle_diff_seg < 0) {
          angle_diff_seg += 2 * M_PI;
        }
        current_angle = start_angle + angle_diff_seg * progress;
      }

      auto point = segment.getPointAtAngle(current_angle);
      path_points.push_back(std::make_pair(point.x, point.y));
    }
  }

  const auto trajectory = start_planner_utils::convertCircularPathToTrajectory(circular_path);
  const auto curvatures = start_planner_utils::calcCurvatureFromTrajectory(trajectory);

  std::vector<std::vector<geometry_msgs::msg::Point>> clothoid_paths;

  // セグメント間の連続性を保つための姿勢管理
  geometry_msgs::msg::Pose current_segment_pose = start_pose;

  for (size_t i = 0; i < circular_path.segments.size(); ++i) {
    const auto & segment = circular_path.segments[i];

    // 車両パラメータから最適なクロソイドパラメータを計算
    const double circular_steer_angle = std::atan(wheel_base / segment.radius);
    const double minimum_steer_time = circular_steer_angle / max_steer_angle_rate;
    const double L_min = velocity * minimum_steer_time;
    const double A_min = std::sqrt(segment.radius * L_min);

    // クロソイド変換を実行
    auto clothoid_points = convertArcToClothoidWithCorrection(
      segment, current_segment_pose, velocity, wheel_base, max_steer_angle_rate,
      0.5);  // point_interval

    if (!clothoid_points.empty()) {
      clothoid_paths.push_back(clothoid_points);

      // 次のセグメントのために終点姿勢を更新
      if (i < circular_path.segments.size() - 1) {
        const auto & last_point = clothoid_points.back();

        // 終点での進行方向を計算（最後の2点から）
        if (clothoid_points.size() >= 2) {
          const auto & second_last = clothoid_points[clothoid_points.size() - 2];
          double dx = last_point.x - second_last.x;
          double dy = last_point.y - second_last.y;
          double heading = std::atan2(dy, dx);

          current_segment_pose.position = last_point;
          current_segment_pose.orientation =
            tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
        }
      }
    } else {
      std::cerr << "Failed to convert segment " << (i + 1) << " to clothoid" << std::endl;
    }
  }

  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  std::vector<geometry_msgs::msg::Point> combined_clothoid_path;
  for (size_t i = 0; i < clothoid_paths.size(); ++i) {
    const auto & clothoid_path = clothoid_paths[i];

    // 最初のセグメント以外は開始点を除いて結合（重複回避）
    size_t start_idx = (i == 0) ? 0 : 1;
    for (size_t j = start_idx; j < clothoid_path.size(); ++j) {
      combined_clothoid_path.push_back(clothoid_path[j]);
    }
  }

  double target_velocity = velocity;  // デフォルト値
  if (!centerline_path.points.empty()) {
    const auto target_idx =
      autoware::motion_utils::findNearestIndex(centerline_path.points, target_pose.position);
    if (target_idx < centerline_path.points.size()) {
      target_velocity = centerline_path.points[target_idx].point.longitudinal_velocity_mps;
    }
  }

  // createPathWithLaneIdFromClothoidPaths関数を呼び出してPathWithLaneIdを生成
  auto parameters = StartPlannerParameters::init(*node_);
  PathWithLaneId path_with_lane_id = createPathWithLaneIdFromClothoidPaths(
    clothoid_paths, target_pose, velocity, target_velocity, 0.0, road_lanes, route_handler);

  auto combined_path = combinePathWithCenterline(path_with_lane_id, centerline_path, target_pose);

  // 曲率計算データの準備
  std::vector<double> arc_lengths;
  std::vector<double> curvature_values;
  std::vector<double> curvature_changes;
  std::vector<double> arc_lengths_changes;
  bool has_curvature_data = false;

  // 結合されたクロソイド経路の曲率を計算
  auto combined_curvatures =
    autoware::behavior_path_planner::start_planner_utils::calcCurvatureFromPoints(
      combined_clothoid_path);

  // 各点の曲率をデバッグプリント
  for (size_t i = 0; i < combined_curvatures.size(); ++i) {
    const auto & point = combined_clothoid_path[i];
    const double curvature = combined_curvatures[i];

    std::cerr << "Point[" << i << "]: "
              << "pos=(" << std::fixed << std::setprecision(3) << point.x << ", " << point.y
              << "), "
              << "curvature=" << std::setprecision(6) << curvature << " (1/m)" << std::endl;
  }

  // 弧長を計算
  arc_lengths.push_back(0.0);
  double cumulative_length = 0.0;
  for (size_t i = 1; i < combined_clothoid_path.size(); ++i) {
    double dx = combined_clothoid_path[i].x - combined_clothoid_path[i - 1].x;
    double dy = combined_clothoid_path[i].y - combined_clothoid_path[i - 1].y;
    cumulative_length += std::sqrt(dx * dx + dy * dy);
    arc_lengths.push_back(cumulative_length);
  }

  // 曲率値をコピー
  for (size_t i = 0; i < combined_curvatures.size(); ++i) {
    curvature_values.push_back(combined_curvatures[i]);
  }

  has_curvature_data = true;

  // 横並びプロット作成: 左に経路、中央に曲率分析、右に速度・加速度
  auto [fig, axes] = plt.subplots(1, 3, Kwargs("figsize"_a = std::make_tuple(24, 6)));
  auto & ax_path = axes[0];
  auto & ax_curvature = axes[1];
  auto & ax_velocity = axes[2];

  // ============================================================================
  // 左側: 経路プロット
  // ============================================================================

  // レーンレットをプロット
  const auto & lanelets = planner_data->route_handler->getLaneletMapPtr()->laneletLayer;
  for (const auto & lanelet : lanelets) {
    plot_lanelet(ax_path, lanelet);
  }

  // 開始姿勢と目標姿勢をプロット
  ax_path.plot(
    Args(start_pose.position.x, start_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "start", "markersize"_a = 20, "color"_a = "green"));
  ax_path.plot(
    Args(target_pose.position.x, target_pose.position.y),
    Kwargs("marker"_a = "x", "label"_a = "target", "markersize"_a = 20, "color"_a = "red"));

  // 元の円弧経路の点をプロット
  std::vector<double> xs, ys;
  for (const auto & point : path_points) {
    xs.push_back(point.first);
    ys.push_back(point.second);
  }

  ax_path.scatter(Args(xs, ys), Kwargs("color"_a = "blue", "s"_a = 10, "alpha"_a = 0.6));

  // クロソイド経路をプロット
  for (size_t i = 0; i < clothoid_paths.size(); ++i) {
    const auto & clothoid_path = clothoid_paths[i];

    std::vector<double> clothoid_xs, clothoid_ys;
    for (const auto & point : clothoid_path) {
      clothoid_xs.push_back(point.x);
      clothoid_ys.push_back(point.y);
    }

    // クロソイド経路を異なる色で描画（線分を削除して点のみ表示）
    std::string color = (i % 2 == 0) ? "red" : "purple";
    std::string label = (i == 0) ? "clothoid path" : "";

    // クロソイド経路の点をscatterで描画
    ax_path.scatter(
      Args(clothoid_xs, clothoid_ys),
      Kwargs("color"_a = color, "s"_a = 15, "label"_a = label, "alpha"_a = 0.8));

    // クロソイド経路の開始点と終了点をマーク
    if (!clothoid_path.empty()) {
      ax_path.plot(
        Args(clothoid_path.front().x, clothoid_path.front().y),
        Kwargs("marker"_a = "o", "color"_a = color, "markersize"_a = 8, "alpha"_a = 0.9));
      ax_path.plot(
        Args(clothoid_path.back().x, clothoid_path.back().y),
        Kwargs("marker"_a = "s", "color"_a = color, "markersize"_a = 8, "alpha"_a = 0.9));
    }
  }

  // PathWithLaneIdをプロット
  if (!path_with_lane_id.points.empty()) {
    plot_path_with_lane_id(ax_path, combined_path, "green", "PathWithLaneId", 3.0);
  }

  // 円弧セグメントの中心点をプロット
  for (size_t i = 0; i < circular_path.segments.size(); ++i) {
    const auto & segment = circular_path.segments[i];
    ax_path.plot(
      Args(segment.center.x, segment.center.y),
      Kwargs(
        "marker"_a = "o", "color"_a = "orange", "markersize"_a = 8,
        "label"_a = (i == 0 ? "arc centers" : "")));
  }

  // プロット範囲を設定
  const double margin = 5.0;

  // PathWithLaneIdの点から範囲を計算
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  // PathWithLaneIdの点のみをチェック
  if (!combined_path.points.empty()) {
    for (const auto & point : combined_path.points) {
      x_min = std::min(x_min, point.point.pose.position.x);
      x_max = std::max(x_max, point.point.pose.position.x);
      y_min = std::min(y_min, point.point.pose.position.y);
      y_max = std::max(y_max, point.point.pose.position.y);
    }
  }

  // 余裕を追加
  x_min -= margin;
  x_max += margin;
  y_min -= margin;
  y_max += margin;

  ax_path.set_xlim(Args(x_min, x_max));
  ax_path.set_ylim(Args(y_min, y_max));
  ax_path.set_aspect(Args("equal"));
  ax_path.grid(Args(true), Kwargs("alpha"_a = 0.3));
  ax_path.set_title(Args("Path Comparison"));
  ax_path.set_xlabel(Args("X [m]"));
  ax_path.set_ylabel(Args("Y [m]"));
  ax_path.legend();

  // ============================================================================
  // 中央: 曲率分析プロット
  // ============================================================================

  if (has_curvature_data && !curvature_values.empty()) {
    // 曲率 vs 弧長をプロット
    ax_curvature.plot(
      Args(arc_lengths, curvature_values),
      Kwargs("color"_a = "blue", "linewidth"_a = 2.0, "label"_a = "Curvature"));

    ax_curvature.set_xlabel(Args("Arc Length [m]"));
    ax_curvature.set_ylabel(Args("Curvature [1/m]"));
    ax_curvature.set_title(Args("Clothoid Path Curvature"));
    ax_curvature.grid(Args(true), Kwargs("alpha"_a = 0.3));
    ax_curvature.legend();

  } else {
    // 曲率データがない場合のメッセージ表示
    ax_curvature.text(
      Args(0.5, 0.5, "Insufficient points for\ncurvature calculation"),
      Kwargs("ha"_a = "center", "va"_a = "center"));
    ax_curvature.set_title(Args("Curvature Analysis (No Data)"));
  }

  // ============================================================================
  // 右側: 速度・加速度分析プロット
  // ============================================================================

  if (!combined_path.points.empty()) {
    plot_velocity_acceleration(ax_velocity, combined_path);
    std::cerr << "Velocity and acceleration plotted for PathWithLaneId" << std::endl;
  } else {
    ax_velocity.text(
      Args(0.5, 0.5, "No PathWithLaneId data\navailable for velocity analysis"),
      Kwargs("ha"_a = "center", "va"_a = "center"));
    ax_velocity.set_title(Args("Velocity & Acceleration (No Data)"));
  }

  // レイアウトを調整して表示
  fig.tight_layout();
  plt.show(Args(), Kwargs("block"_a = true));
}

}  // namespace autoware::behavior_path_planner
