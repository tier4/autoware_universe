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

#include "autoware/trajectory_modifier/trajectory_modifier.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/virtual_traffic_light_stop.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_core/utility/Utilities.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using autoware::trajectory_modifier::TrajectoryModifierContext;
using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware::trajectory_modifier::plugin::VirtualTrafficLightStop;
using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using tier4_v2x_msgs::msg::InfrastructureCommandArray;
using tier4_v2x_msgs::msg::VirtualTrafficLightState;
using tier4_v2x_msgs::msg::VirtualTrafficLightStateArray;
using visualization_msgs::msg::MarkerArray;

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.compare(0, prefix.size(), prefix) == 0;
}

bool ends_with(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void assign_time_from_start(TrajectoryPoints & trajectory)
{
  for (size_t i = 0; i < trajectory.size(); ++i) {
    trajectory.at(i).time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * 0.2);
  }
}

TrajectoryPoints make_trajectory()
{
  TrajectoryPoints trajectory;
  for (size_t i = 0; i < 3; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 5.0;
    trajectory.push_back(point);
  }
  assign_time_from_start(trajectory);
  return trajectory;
}

TrajectoryPoints make_straight_trajectory(const double end_x)
{
  TrajectoryPoints trajectory;
  for (size_t i = 0; i <= static_cast<size_t>(std::ceil(end_x)); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = std::min(static_cast<double>(i), end_x);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 5.0;
    trajectory.push_back(point);
  }
  assign_time_from_start(trajectory);
  return trajectory;
}

TrajectoryPoints make_straight_trajectory(const double start_x, const double end_x)
{
  TrajectoryPoints trajectory;
  for (double x = start_x; x <= end_x; x += 1.0) {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 5.0;
    trajectory.push_back(point);
  }
  if (trajectory.empty() || trajectory.back().pose.position.x < end_x) {
    TrajectoryPoint point;
    point.pose.position.x = end_x;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 5.0;
    trajectory.push_back(point);
  }
  assign_time_from_start(trajectory);
  return trajectory;
}

Odometry::ConstSharedPtr make_odometry(
  const double x, const double velocity, const rclcpp::Time & stamp)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.header.stamp = stamp;
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = velocity;
  return std::make_shared<const Odometry>(odometry);
}

Odometry::ConstSharedPtr make_odometry(
  const double x, const double y, const double yaw, const double velocity,
  const rclcpp::Time & stamp)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.header.stamp = stamp;
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  odometry.pose.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  odometry.twist.twist.linear.x = velocity;
  return std::make_shared<const Odometry>(odometry);
}

AccelWithCovarianceStamped::ConstSharedPtr make_acceleration(const double ax)
{
  auto acceleration = std::make_shared<AccelWithCovarianceStamped>();
  acceleration->accel.accel.linear.x = ax;
  return acceleration;
}

void set_zero_velocity_at_x(TrajectoryPoints & trajectory, const double x)
{
  for (auto & point : trajectory) {
    if (std::abs(point.pose.position.x - x) < 1e-6) {
      point.longitudinal_velocity_mps = 0.0F;
      return;
    }
  }
}

VirtualTrafficLightStateArray::ConstSharedPtr make_vtl_states(
  const lanelet::Id instrument_id, const bool approval, const bool is_finalized,
  const rclcpp::Time & stamp)
{
  VirtualTrafficLightState state;
  state.stamp = stamp;
  state.type = "virtual_traffic_light";
  state.id = std::to_string(instrument_id);
  state.approval = approval;
  state.is_finalized = is_finalized;

  auto states = std::make_shared<VirtualTrafficLightStateArray>();
  states->stamp = stamp;
  states->states.push_back(state);
  return states;
}

LaneletRoute::ConstSharedPtr make_route(const lanelet::Id lanelet_id)
{
  auto route = std::make_shared<LaneletRoute>();
  LaneletSegment segment;
  segment.preferred_primitive.id = lanelet_id;
  route->segments.push_back(segment);
  return route;
}

lanelet::LineString3d make_cross_line(const lanelet::Id id, const double x)
{
  lanelet::Point3d left(lanelet::utils::getId(), x, -5.0, 0.0);
  lanelet::Point3d right(lanelet::utils::getId(), x, 5.0, 0.0);
  return lanelet::LineString3d(id, {left, right});
}

lanelet::LineString3d make_cross_line(const lanelet::Id id, const double x, const double center_y)
{
  lanelet::Point3d left(lanelet::utils::getId(), x, center_y - 1.0, 0.0);
  lanelet::Point3d right(lanelet::utils::getId(), x, center_y + 1.0, 0.0);
  return lanelet::LineString3d(id, {left, right});
}

std::shared_ptr<lanelet::LaneletMap> make_vtl_map(
  const lanelet::Id instrument_id, const double start_x, const double stop_x,
  const std::vector<std::tuple<lanelet::Id, double, double>> & end_lines,
  const double lane_end_x = 100.0)
{
  auto start_line = make_cross_line(lanelet::utils::getId(), start_x);
  auto stop_line = make_cross_line(lanelet::utils::getId(), stop_x);
  auto instrument = make_cross_line(instrument_id, stop_x + 1.0);
  instrument.attributes()["type"] = "intersection_coordination";

  lanelet::RuleParameterMap parameters;
  parameters[lanelet::RoleNameString::Refers].push_back(instrument);
  parameters[lanelet::RoleNameString::RefLine].push_back(stop_line);
  parameters["start_line"].push_back(start_line);
  for (const auto & [id, x, center_y] : end_lines) {
    parameters["end_line"].push_back(make_cross_line(id, x, center_y));
  }
  auto vtl = lanelet::RegulatoryElementFactory::create(
    lanelet::autoware::VirtualTrafficLight::RuleName, lanelet::utils::getId(), parameters);

  lanelet::Point3d l1(lanelet::utils::getId(), 0.0, -5.0, 0.0);
  lanelet::Point3d l2(lanelet::utils::getId(), lane_end_x, -5.0, 0.0);
  lanelet::Point3d r1(lanelet::utils::getId(), 0.0, 5.0, 0.0);
  lanelet::Point3d r2(lanelet::utils::getId(), lane_end_x, 5.0, 0.0);
  lanelet::LineString3d left(lanelet::utils::getId(), {l1, l2});
  lanelet::LineString3d right(lanelet::utils::getId(), {r1, r2});
  lanelet::Lanelet lane(lanelet::utils::getId(), left, right);
  lane.addRegulatoryElement(vtl);

  return lanelet::utils::createMap({lane});
}

std::shared_ptr<lanelet::LaneletMap> make_vtl_map(
  const lanelet::Id instrument_id, const double start_x, const double stop_x, const double end_x)
{
  return make_vtl_map(instrument_id, start_x, stop_x, {{lanelet::utils::getId(), end_x, 0.0}});
}

std::shared_ptr<lanelet::LaneletMap> make_curved_vtl_map(
  const lanelet::Id instrument_id, const lanelet::Id end_line_id)
{
  const std::vector<lanelet::Point3d> left_points{
    lanelet::Point3d(lanelet::utils::getId(), 0.0, -2.0, 0.0),
    lanelet::Point3d(lanelet::utils::getId(), 5.0, -2.0, 0.0),
    lanelet::Point3d(lanelet::utils::getId(), 10.0, -1.0, 0.0),
    lanelet::Point3d(lanelet::utils::getId(), 14.0, 3.0, 0.0),
    lanelet::Point3d(lanelet::utils::getId(), 16.0, 7.0, 0.0),
    lanelet::Point3d(lanelet::utils::getId(), 17.0, 13.0, 0.0)};
  const std::vector<lanelet::Point3d> right_points{
    lanelet::Point3d(lanelet::utils::getId(), 0.0, 2.0, 0.0),
    lanelet::Point3d(lanelet::utils::getId(), 5.0, 2.0, 0.0),
    lanelet::Point3d(lanelet::utils::getId(), 8.0, 3.0, 0.0),
    lanelet::Point3d(lanelet::utils::getId(), 10.0, 5.0, 0.0),
    lanelet::Point3d(lanelet::utils::getId(), 12.0, 9.0, 0.0),
    lanelet::Point3d(lanelet::utils::getId(), 13.0, 13.0, 0.0)};
  lanelet::LineString3d left(lanelet::utils::getId(), left_points);
  lanelet::LineString3d right(lanelet::utils::getId(), right_points);
  lanelet::Lanelet lane(lanelet::utils::getId(), left, right);
  lane.setCenterline(lanelet::LineString3d(
    lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 0.0, 0.0, 0.0),
                              lanelet::Point3d(lanelet::utils::getId(), 5.0, 0.0, 0.0),
                              lanelet::Point3d(lanelet::utils::getId(), 9.0, 1.0, 0.0),
                              lanelet::Point3d(lanelet::utils::getId(), 12.0, 4.0, 0.0),
                              lanelet::Point3d(lanelet::utils::getId(), 14.0, 8.0, 0.0),
                              lanelet::Point3d(lanelet::utils::getId(), 15.0, 13.0, 0.0)}));

  lanelet::LineString3d start_line(
    lanelet::utils::getId(), {left_points.at(0), right_points.at(0)});
  lanelet::LineString3d stop_line(lanelet::utils::getId(), {left_points.at(1), right_points.at(1)});
  lanelet::LineString3d end_line(end_line_id, {left_points.at(4), right_points.at(4)});
  lanelet::LineString3d instrument(instrument_id, {left_points.at(2), right_points.at(2)});
  instrument.attributes()["type"] = "intersection_coordination";

  lanelet::RuleParameterMap parameters;
  parameters[lanelet::RoleNameString::Refers].push_back(instrument);
  parameters[lanelet::RoleNameString::RefLine].push_back(stop_line);
  parameters["start_line"].push_back(start_line);
  parameters["end_line"].push_back(end_line);
  const auto vtl = lanelet::RegulatoryElementFactory::create(
    lanelet::autoware::VirtualTrafficLight::RuleName, lanelet::utils::getId(), parameters);
  lane.addRegulatoryElement(vtl);
  return lanelet::utils::createMap({lane});
}

TrajectoryPoints make_curved_short_trajectory()
{
  TrajectoryPoints trajectory;
  for (const auto & [x, y] :
       std::vector<std::pair<double, double>>{{10.5, 2.5}, {11.25, 3.25}, {12.0, 4.0}}) {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.orientation = autoware_utils::create_quaternion_from_yaw(M_PI_4);
    point.longitudinal_velocity_mps = 5.0F;
    trajectory.push_back(point);
  }
  assign_time_from_start(trajectory);
  return trajectory;
}

TrajectoryModifierData make_vtl_input(
  Odometry::ConstSharedPtr odometry, std::shared_ptr<lanelet::LaneletMap> lanelet_map,
  LaneletRoute::ConstSharedPtr route, VirtualTrafficLightStateArray::ConstSharedPtr states)
{
  TrajectoryModifierData input;
  input.current_odometry = std::move(odometry);
  input.lanelet_map = std::move(lanelet_map);
  input.route = std::move(route);
  input.virtual_traffic_light_states = std::move(states);
  return input;
}

void expect_same_trajectory(const TrajectoryPoints & actual, const TrajectoryPoints & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < actual.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual[i].pose.position.x, expected[i].pose.position.x);
    EXPECT_DOUBLE_EQ(actual[i].pose.position.y, expected[i].pose.position.y);
    EXPECT_FLOAT_EQ(actual[i].longitudinal_velocity_mps, expected[i].longitudinal_velocity_mps);
  }
}

void expect_truncated_stop_at_x(
  const TrajectoryPoints & trajectory, const double x, const double tolerance = 0.3)
{
  ASSERT_FALSE(trajectory.empty());
  EXPECT_NEAR(trajectory.back().pose.position.x, x, tolerance);
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
}

void expect_three_point_stop_at_ego(
  const TrajectoryPoints & trajectory, const double ego_x, const double time_step)
{
  ASSERT_EQ(trajectory.size(), 3U);
  constexpr double point_interval = 1e-3;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto & point = trajectory.at(i);
    EXPECT_NEAR(point.pose.position.x, ego_x + static_cast<double>(i) * point_interval, 1e-9);
    EXPECT_NEAR(point.pose.position.y, 0.0, 1e-9);
    EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(point.lateral_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(point.acceleration_mps2, 0.0F);
    EXPECT_FLOAT_EQ(point.heading_rate_rps, 0.0F);
    EXPECT_DOUBLE_EQ(
      rclcpp::Duration(point.time_from_start).seconds(), static_cast<double>(i) * time_step);
  }
}
}  // namespace

class VirtualTrafficLightStopIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions node_options;
    const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      node_options, {test_utils_dir + "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<rclcpp::Node>("test_virtual_traffic_light_stop_node", node_options);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    processing_time_pub_ = node_->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail", 1);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(processing_time_pub_);
    context_ = std::make_shared<TrajectoryModifierContext>(node_.get());

    marker_sub_ = node_->create_subscription<MarkerArray>(
      "~/virtual_traffic_light_stop/debug/marker", 10,
      [this](const MarkerArray::ConstSharedPtr msg) { marker_messages_.push_back(*msg); });
    text_sub_ = node_->create_subscription<StringStamped>(
      "~/virtual_traffic_light_stop/debug/text", 10,
      [this](const StringStamped::ConstSharedPtr msg) { text_messages_.push_back(*msg); });
    processing_time_sub_ = node_->create_subscription<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail", 20,
      [this](const autoware_utils_debug::ProcessingTimeDetail::ConstSharedPtr msg) {
        processing_time_messages_.push_back(*msg);
      });
    command_sub_ = node_->create_subscription<InfrastructureCommandArray>(
      "~/output/infrastructure_commands", 10,
      [this](const InfrastructureCommandArray::ConstSharedPtr msg) {
        command_messages_.push_back(*msg);
      });
    executor_->add_node(node_);

    params_.use_virtual_traffic_light_stop = true;
    params_.virtual_traffic_light.max_delay_sec = 3.0;
    params_.virtual_traffic_light.near_line_distance = 1.0;
    params_.virtual_traffic_light.dead_line_margin = 1.0;
    params_.virtual_traffic_light.max_yaw_deviation_deg = 90.0;
    params_.virtual_traffic_light.check_timeout_after_stop_line = true;
    params_.virtual_traffic_light.min_hold_trajectory_length = 5.0;
    params_.stopping_constraints.nominal_deceleration = 1.0;
    params_.stopping_constraints.maximum_deceleration = 4.0;
    params_.stopping_constraints.jerk_limit = 3.0;
    params_.stopping_constraints.arrived_distance_threshold = 0.5;

    plugin_ = std::make_unique<VirtualTrafficLightStop>();
    plugin_->initialize(
      "test_virtual_traffic_light_stop", node_.get(), time_keeper_, context_, params_);
  }

  void TearDown() override
  {
    executor_->remove_node(node_);
    plugin_.reset();
    context_.reset();
    time_keeper_.reset();
    node_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  void spin_until(const std::function<bool()> & condition)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!condition() && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_->spin_some();
  }

  void expect_single_stop_factor(const std::string & detail)
  {
    const auto factors = plugin_->get_planning_factors();
    ASSERT_EQ(factors.size(), 1U);
    EXPECT_EQ(factors.front().behavior, PlanningFactor::STOP);
    EXPECT_EQ(factors.front().detail, detail);
    EXPECT_EQ(factors.front().module, "modifier_virtual_traffic_light_stop");
  }

  double geometric_stop_x(const double line_x) const
  {
    return line_x - context_->vehicle_info.max_longitudinal_offset_m;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr processing_time_pub_;
  rclcpp::Subscription<MarkerArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<StringStamped>::SharedPtr text_sub_;
  rclcpp::Subscription<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr processing_time_sub_;
  rclcpp::Subscription<InfrastructureCommandArray>::SharedPtr command_sub_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<TrajectoryModifierContext> context_;
  std::unique_ptr<VirtualTrafficLightStop> plugin_;
  trajectory_modifier_params::Params params_;
  std::vector<MarkerArray> marker_messages_;
  std::vector<StringStamped> text_messages_;
  std::vector<autoware_utils_debug::ProcessingTimeDetail> processing_time_messages_;
  std::vector<InfrastructureCommandArray> command_messages_;
};

TEST_F(VirtualTrafficLightStopIntegrationTest, DisabledPluginDoesNotModify)
{
  params_.use_virtual_traffic_light_stop = false;
  plugin_->update_params(params_);

  auto trajectory = make_trajectory();
  const auto original = trajectory;
  TrajectoryModifierData input;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, MissingMapAndRouteAreSafe)
{
  auto trajectory = make_trajectory();
  const auto original = trajectory;
  TrajectoryModifierData input;
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  input.current_odometry = std::make_shared<nav_msgs::msg::Odometry>(odometry);
  input.lanelet_map = nullptr;
  input.route = nullptr;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, DeniedStateStopsWhenTrajectoryDoesNotReachEndLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto states = make_vtl_states(instrument_id, false, false, node_->now());
  auto input = make_vtl_input(make_odometry(10.0, 5.0, node_->now()), lanelet_map, route, states);
  auto trajectory = make_straight_trajectory(25.0);

  plugin_->begin_cycle(input);
  const bool modified = plugin_->modify_trajectory(trajectory, input);
  plugin_->end_cycle();

  ASSERT_TRUE(modified);
  expect_single_stop_factor("VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE");
  expect_truncated_stop_at_x(trajectory, geometric_stop_x(stop_x));
}

TEST_F(VirtualTrafficLightStopIntegrationTest, DeniedFinalizedStateStillStopsAtStopLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto states = make_vtl_states(instrument_id, false, true, node_->now());
  auto input = make_vtl_input(make_odometry(10.0, 5.0, node_->now()), lanelet_map, route, states);
  auto trajectory = make_straight_trajectory(25.0);

  plugin_->begin_cycle(input);
  const bool modified = plugin_->modify_trajectory(trajectory, input);
  plugin_->end_cycle();

  ASSERT_TRUE(modified);
  expect_single_stop_factor("VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE");
  expect_truncated_stop_at_x(trajectory, geometric_stop_x(stop_x));
}

TEST_F(VirtualTrafficLightStopIntegrationTest, MissingStateStopsAtStopLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto input = make_vtl_input(
    make_odometry(10.0, 5.0, node_->now()), lanelet_map, route,
    VirtualTrafficLightStateArray::ConstSharedPtr{});
  auto trajectory = make_straight_trajectory(25.0);

  plugin_->begin_cycle(input);
  const bool modified = plugin_->modify_trajectory(trajectory, input);
  plugin_->end_cycle();

  ASSERT_TRUE(modified);
  expect_single_stop_factor("VTL 12345: NO_STATE -> STOP_LINE");
  expect_truncated_stop_at_x(trajectory, geometric_stop_x(stop_x));
}

TEST_F(VirtualTrafficLightStopIntegrationTest, MissingStateDoesNotStopBeforeDistantStopLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 80.0;
  constexpr double end_x = 90.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto input = make_vtl_input(
    make_odometry(10.0, 5.0, node_->now()), lanelet_map, route,
    VirtualTrafficLightStateArray::ConstSharedPtr{});
  auto trajectory = make_straight_trajectory(10.0, 15.0);
  const auto original = trajectory;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, StaleApprovedStateStopsAtStopLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  const auto stale_stamp = node_->now() - rclcpp::Duration::from_seconds(4.0);
  auto states = make_vtl_states(instrument_id, true, false, stale_stamp);
  auto input = make_vtl_input(make_odometry(10.0, 5.0, node_->now()), lanelet_map, route, states);
  auto trajectory = make_straight_trajectory(25.0);

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  expect_single_stop_factor("VTL 12345: STATE_TIMEOUT_BEFORE_STOP_LINE -> STOP_LINE");
  expect_truncated_stop_at_x(trajectory, geometric_stop_x(stop_x));
}

TEST_F(VirtualTrafficLightStopIntegrationTest, StaleApprovedFinalizedStateStillStopsAtStopLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  const auto stale_stamp = node_->now() - rclcpp::Duration::from_seconds(4.0);
  auto states = make_vtl_states(instrument_id, true, true, stale_stamp);
  auto input = make_vtl_input(make_odometry(10.0, 5.0, node_->now()), lanelet_map, route, states);
  auto trajectory = make_straight_trajectory(25.0);

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  expect_single_stop_factor("VTL 12345: STATE_TIMEOUT_BEFORE_STOP_LINE -> STOP_LINE");
  expect_truncated_stop_at_x(trajectory, geometric_stop_x(stop_x));
}

TEST_F(VirtualTrafficLightStopIntegrationTest, StaleApprovedStateAfterStopLineReportsExactReason)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  const auto stale_stamp = node_->now() - rclcpp::Duration::from_seconds(4.0);
  auto input = make_vtl_input(
    make_odometry(25.0, 5.0, node_->now()), lanelet_map, make_route(lanelet_id),
    make_vtl_states(instrument_id, true, false, stale_stamp));
  auto trajectory = make_straight_trajectory(10.0, 45.0);

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  expect_single_stop_factor("VTL 12345: STATE_TIMEOUT_AFTER_STOP_LINE -> STOP_LINE");
}

TEST_F(VirtualTrafficLightStopIntegrationTest, WaitingForFinalizationReportsEndLineStop)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto input = make_vtl_input(
    make_odometry(25.0, 5.0, node_->now()), lanelet_map, make_route(lanelet_id),
    make_vtl_states(instrument_id, true, false, node_->now()));
  auto trajectory = make_straight_trajectory(10.0, 45.0);

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  expect_single_stop_factor("VTL 12345: WAITING_FINALIZATION -> END_LINE");
  expect_truncated_stop_at_x(trajectory, geometric_stop_x(40.0));
}

TEST_F(VirtualTrafficLightStopIntegrationTest, MovingAtEndLineUsesSharedControlStopTrajectory)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double end_x = 40.0;
  const auto ego_x = geometric_stop_x(end_x);
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto input = make_vtl_input(
    make_odometry(ego_x, 1.0, node_->now()), lanelet_map, make_route(lanelet_id),
    make_vtl_states(instrument_id, true, false, node_->now()));
  auto trajectory = make_straight_trajectory(10.0, 45.0);

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  expect_three_point_stop_at_ego(trajectory, ego_x, params_.trajectory_time_step);
  expect_single_stop_factor("VTL 12345: WAITING_FINALIZATION -> END_LINE");
}

TEST_F(VirtualTrafficLightStopIntegrationTest, PublishesProductionDebugMarkersTextAndTiming)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto input = make_vtl_input(
    make_odometry(10.0, 5.0, node_->now()), lanelet_map, make_route(lanelet_id),
    VirtualTrafficLightStateArray::ConstSharedPtr{});
  auto trajectory = make_straight_trajectory(0.0, 45.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->publish_debug_data("trajectory_0");
  plugin_->end_cycle();
  spin_until([this]() {
    return !marker_messages_.empty() && !text_messages_.empty() &&
           processing_time_messages_.size() >= 4U;
  });

  ASSERT_FALSE(marker_messages_.empty());
  const auto & markers = marker_messages_.back().markers;
  std::set<std::pair<std::string, int32_t>> marker_keys;
  bool has_start = false;
  bool has_stop = false;
  bool has_instrument = false;
  bool has_end = false;
  bool has_start_label = false;
  bool has_stop_label = false;
  bool has_instrument_label = false;
  bool has_end_label = false;
  for (const auto & marker : markers) {
    EXPECT_EQ(marker.header.frame_id, "map");
    EXPECT_NE(marker.ns.find("trajectory_0/vtl_"), std::string::npos);
    EXPECT_TRUE(marker_keys.emplace(marker.ns, marker.id).second);
    if (ends_with(marker.ns, "/start_line/line")) {
      has_start = true;
      EXPECT_FLOAT_EQ(marker.color.r, 0.1F);
      EXPECT_FLOAT_EQ(marker.color.g, 0.4F);
      EXPECT_FLOAT_EQ(marker.color.b, 1.0F);
      EXPECT_DOUBLE_EQ(marker.scale.x, 0.35);
    } else if (ends_with(marker.ns, "/stop_line/line")) {
      has_stop = true;
      EXPECT_FLOAT_EQ(marker.color.r, 1.0F);
      EXPECT_FLOAT_EQ(marker.color.g, 0.0F);
      EXPECT_FLOAT_EQ(marker.color.b, 0.0F);
    } else if (ends_with(marker.ns, "/instrument_line/line")) {
      has_instrument = true;
      EXPECT_FLOAT_EQ(marker.color.r, 1.0F);
      EXPECT_FLOAT_EQ(marker.color.g, 0.8F);
      EXPECT_FLOAT_EQ(marker.color.b, 0.0F);
    } else if (marker.ns.find("/active_end_line/line") != std::string::npos) {
      has_end = true;
      EXPECT_FLOAT_EQ(marker.color.r, 0.0F);
      EXPECT_FLOAT_EQ(marker.color.g, 1.0F);
      EXPECT_FLOAT_EQ(marker.color.b, 0.2F);
    }
    has_start_label = has_start_label || marker.text == "start_line";
    has_stop_label = has_stop_label || marker.text == "stop_line";
    has_instrument_label = has_instrument_label || marker.text == "instrument_line";
    has_end_label = has_end_label || starts_with(marker.text, "active_end_line[");
  }
  EXPECT_TRUE(has_start);
  EXPECT_TRUE(has_stop);
  EXPECT_TRUE(has_instrument);
  EXPECT_TRUE(has_end);
  EXPECT_TRUE(has_start_label);
  EXPECT_TRUE(has_stop_label);
  EXPECT_TRUE(has_instrument_label);
  EXPECT_TRUE(has_end_label);

  ASSERT_FALSE(text_messages_.empty());
  const auto & text = text_messages_.back().data;
  for (const auto & expected :
       {"CANDIDATE: trajectory_0", "VTL[12345]",
        "LANELET_ID:", "REGULATORY_ELEMENT_ID:", "STATE: REQUESTING", "V2X: received=false",
        "DECISION: STOP target=STOP_LINE reason=NO_STATE modified=true",
        "ACTIVE_END_LINE_ID:", "DISTANCE_PATH_M:", "DISTANCE_CENTERLINE_M:", "ego_in_lane=true",
        "stop_line_relevant=true", "end_hold_active=false"}) {
    EXPECT_NE(text.find(expected), std::string::npos) << expected;
  }

  std::set<std::string> timing_names;
  for (const auto & tree : processing_time_messages_) {
    for (const auto & node : tree.nodes) {
      timing_names.insert(node.name);
    }
  }
  for (const auto & expected :
       {"VirtualTrafficLightStop::begin_cycle", "VirtualTrafficLightStop::rebuild_modules",
        "VirtualTrafficLightStop::update_module_states",
        "VirtualTrafficLightStop::update_module_lifecycle",
        "VirtualTrafficLightStop::modify_trajectory", "VirtualTrafficLightStop::process_trajectory",
        "VirtualTrafficLightStop::process_module", "VirtualTrafficLightStop::insert_stop_velocity",
        "VirtualTrafficLightStop::publish_debug_data", "VirtualTrafficLightStop::end_cycle"}) {
    EXPECT_NE(timing_names.find(expected), timing_names.end()) << expected;
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, CandidateMarkerNamespacesDoNotOverlap)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto input = make_vtl_input(
    make_odometry(10.0, 5.0, node_->now()), lanelet_map, make_route(lanelet_id),
    VirtualTrafficLightStateArray::ConstSharedPtr{});
  auto trajectory = make_straight_trajectory(0.0, 45.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->publish_debug_data("trajectory_0");
  plugin_->publish_debug_data("trajectory_1");
  plugin_->end_cycle();
  spin_until([this]() { return marker_messages_.size() >= 2U; });

  ASSERT_GE(marker_messages_.size(), 2U);
  std::set<std::string> first_namespaces;
  std::set<std::string> second_namespaces;
  for (const auto & marker : marker_messages_.at(marker_messages_.size() - 2).markers) {
    first_namespaces.insert(marker.ns);
  }
  for (const auto & marker : marker_messages_.back().markers) {
    second_namespaces.insert(marker.ns);
  }
  EXPECT_TRUE(std::all_of(first_namespaces.begin(), first_namespaces.end(), [](const auto & ns) {
    return starts_with(ns, "trajectory_0/vtl_");
  }));
  EXPECT_TRUE(std::all_of(second_namespaces.begin(), second_namespaces.end(), [](const auto & ns) {
    return starts_with(ns, "trajectory_1/vtl_");
  }));
  std::vector<std::string> overlap;
  std::set_intersection(
    first_namespaces.begin(), first_namespaces.end(), second_namespaces.begin(),
    second_namespaces.end(), std::back_inserter(overlap));
  EXPECT_TRUE(overlap.empty());
}

TEST_F(VirtualTrafficLightStopIntegrationTest, CandidateOrderDoesNotChangeStateCommandOrOutput)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto states = make_vtl_states(instrument_id, false, false, node_->now());

  const auto run_cycle = [&](const bool reverse_order) {
    auto input = make_vtl_input(make_odometry(10.0, 5.0, node_->now()), lanelet_map, route, states);
    auto candidate_a = make_straight_trajectory(0.0, 45.0);
    auto candidate_b = make_straight_trajectory(2.0, 50.0);
    plugin_->begin_cycle(input);
    if (reverse_order) {
      EXPECT_TRUE(plugin_->is_trajectory_modification_required(candidate_b, input));
      EXPECT_TRUE(plugin_->modify_trajectory(candidate_b, input));
      EXPECT_TRUE(plugin_->is_trajectory_modification_required(candidate_a, input));
      EXPECT_TRUE(plugin_->modify_trajectory(candidate_a, input));
    } else {
      EXPECT_TRUE(plugin_->is_trajectory_modification_required(candidate_a, input));
      EXPECT_TRUE(plugin_->modify_trajectory(candidate_a, input));
      EXPECT_TRUE(plugin_->is_trajectory_modification_required(candidate_b, input));
      EXPECT_TRUE(plugin_->modify_trajectory(candidate_b, input));
    }
    const auto previous_command_count = command_messages_.size();
    plugin_->end_cycle();
    spin_until([&]() { return command_messages_.size() > previous_command_count; });
    EXPECT_GT(command_messages_.size(), previous_command_count);
    return std::pair{candidate_a, candidate_b};
  };

  const auto first = run_cycle(false);
  ASSERT_FALSE(command_messages_.empty());
  const auto first_command = command_messages_.back();
  const auto second = run_cycle(true);
  ASSERT_GE(command_messages_.size(), 2U);
  const auto second_command = command_messages_.back();

  expect_same_trajectory(first.first, second.first);
  expect_same_trajectory(first.second, second.second);
  ASSERT_EQ(first_command.commands.size(), 1U);
  ASSERT_EQ(second_command.commands.size(), 1U);
  EXPECT_EQ(first_command.commands.front().id, second_command.commands.front().id);
  EXPECT_EQ(first_command.commands.front().state, second_command.commands.front().state);
  EXPECT_EQ(
    first_command.commands.front().custom_tags, second_command.commands.front().custom_tags);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, LifecycleStateAdvancesWithoutCandidateProcessing)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);

  const auto run_cycle = [&](const double ego_x, const double velocity) {
    auto states = make_vtl_states(instrument_id, true, false, node_->now());
    auto input =
      make_vtl_input(make_odometry(ego_x, velocity, node_->now()), lanelet_map, route, states);
    const auto previous_command_count = command_messages_.size();
    plugin_->begin_cycle(input);
    plugin_->end_cycle();
    spin_until([&]() { return command_messages_.size() > previous_command_count; });
    EXPECT_GT(command_messages_.size(), previous_command_count);
    EXPECT_EQ(command_messages_.back().commands.size(), 1U);
    return command_messages_.back().commands.front().state;
  };

  EXPECT_EQ(run_cycle(0.0, 5.0), static_cast<uint8_t>(VirtualTrafficLightStop::ModuleState::NONE));
  EXPECT_EQ(
    run_cycle(10.0, 5.0), static_cast<uint8_t>(VirtualTrafficLightStop::ModuleState::REQUESTING));
  EXPECT_EQ(
    run_cycle(25.0, 5.0), static_cast<uint8_t>(VirtualTrafficLightStop::ModuleState::PASSING));
  EXPECT_EQ(
    run_cycle(36.0, 0.0), static_cast<uint8_t>(VirtualTrafficLightStop::ModuleState::FINALIZING));
  EXPECT_EQ(
    run_cycle(42.0, 5.0), static_cast<uint8_t>(VirtualTrafficLightStop::ModuleState::FINALIZED));
}

TEST_F(VirtualTrafficLightStopIntegrationTest, DeniedStateHoldsWhenStopLineIsNotInTrajectory)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto states = make_vtl_states(instrument_id, false, false, node_->now());

  {
    auto input = make_vtl_input(make_odometry(10.0, 5.0, node_->now()), lanelet_map, route, states);
    auto trajectory = make_straight_trajectory(10.0, 25.0);
    plugin_->begin_cycle(input);
    ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
    plugin_->end_cycle();
  }

  auto input = make_vtl_input(make_odometry(22.0, 1.0, node_->now()), lanelet_map, route, states);
  auto trajectory = make_straight_trajectory(22.0, 30.0);

  plugin_->begin_cycle(input);
  EXPECT_TRUE(plugin_->is_trajectory_modification_required(trajectory, input));
  const bool modified = plugin_->modify_trajectory(trajectory, input);
  plugin_->end_cycle();

  ASSERT_TRUE(modified);
  EXPECT_TRUE(std::all_of(trajectory.begin(), trajectory.end(), [](const auto & point) {
    return point.longitudinal_velocity_mps == 0.0F;
  }));
}

TEST_F(
  VirtualTrafficLightStopIntegrationTest, ApprovedStateDoesNotStopBeforeStopLineWithShortTrajectory)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto states = make_vtl_states(instrument_id, true, false, node_->now());
  auto input = make_vtl_input(make_odometry(10.0, 0.0, node_->now()), lanelet_map, route, states);
  auto trajectory = make_straight_trajectory(10.0, 15.0);
  const auto original = trajectory;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  const bool modified = plugin_->modify_trajectory(trajectory, input);
  plugin_->end_cycle();

  EXPECT_FALSE(modified);
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ApprovedShortTrajectoryIsLeftToUpstream)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto states = make_vtl_states(instrument_id, true, false, node_->now());
  auto input = make_vtl_input(
    make_odometry(10.0, 0.0, node_->now()), lanelet_map, make_route(lanelet_id), states);
  auto trajectory = make_straight_trajectory(10.0, 10.1);
  for (auto & point : trajectory) {
    point.longitudinal_velocity_mps = 0.02F;
  }

  const auto original = trajectory;
  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ApprovedStateFinalizesAfterPassingEndLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto states = make_vtl_states(instrument_id, true, false, node_->now());
  auto input = make_vtl_input(make_odometry(42.0, 1.0, node_->now()), lanelet_map, route, states);
  auto trajectory = make_straight_trajectory(35.0, 50.0);
  const auto original = trajectory;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  const bool modified = plugin_->modify_trajectory(trajectory, input);
  plugin_->end_cycle();

  EXPECT_FALSE(modified);
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ApprovedFinalizedStateDoesNotStopAtEndLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto states = make_vtl_states(instrument_id, true, true, node_->now());
  auto input = make_vtl_input(make_odometry(25.0, 5.0, node_->now()), lanelet_map, route, states);
  auto trajectory = make_straight_trajectory(25.0, 45.0);
  const auto original = trajectory;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  const bool modified = plugin_->modify_trajectory(trajectory, input);
  plugin_->end_cycle();

  EXPECT_FALSE(modified);
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ApprovedStateIgnoresShortTrajectoryFarFromEndLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto states = make_vtl_states(instrument_id, true, false, node_->now());
  auto input = make_vtl_input(make_odometry(25.0, 5.0, node_->now()), lanelet_map, route, states);
  auto trajectory = make_straight_trajectory(25.0, 30.0);
  const auto original = trajectory;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  const bool modified = plugin_->modify_trajectory(trajectory, input);
  plugin_->end_cycle();

  EXPECT_FALSE(modified);
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ApprovedStateStopsNearEndLineWithReplaceOrTruncate)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double start_x = 5.0;
  constexpr double stop_x = 20.0;
  constexpr double end_x = 40.0;
  const auto ego_x = 36.0;

  auto lanelet_map = make_vtl_map(instrument_id, start_x, stop_x, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto route = make_route(lanelet_id);
  auto states = make_vtl_states(instrument_id, true, false, node_->now());
  auto input = make_vtl_input(make_odometry(ego_x, 0.0, node_->now()), lanelet_map, route, states);
  auto trajectory = make_straight_trajectory(ego_x, 36.5);

  plugin_->begin_cycle(input);
  EXPECT_TRUE(plugin_->is_trajectory_modification_required(trajectory, input));
  const bool modified = plugin_->modify_trajectory(trajectory, input);
  plugin_->end_cycle();

  ASSERT_TRUE(modified);
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_LE(trajectory.back().pose.position.x, geometric_stop_x(end_x) + 0.3);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, SelectsFirstDownstreamEndLineIndependentOfMapOrder)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr lanelet::Id first_end_line_id = 400;
  constexpr lanelet::Id later_end_line_id = 500;
  auto lanelet_map = make_vtl_map(
    instrument_id, 5.0, 20.0,
    {{later_end_line_id, 50.0, 0.0}, {600, 40.0, 0.0}, {first_end_line_id, 40.0, 0.0}});
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto states = make_vtl_states(instrument_id, true, false, node_->now());
  auto input = make_vtl_input(
    make_odometry(36.0, 0.0, node_->now()), lanelet_map, make_route(lanelet_id), states);
  auto trajectory = make_straight_trajectory(36.0, 36.5);

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->publish_debug_data("trajectory_0");
  plugin_->end_cycle();
  spin_until([this]() { return !marker_messages_.empty() && !text_messages_.empty(); });

  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  ASSERT_FALSE(text_messages_.empty());
  EXPECT_NE(text_messages_.back().data.find("ACTIVE_END_LINE_ID: 400"), std::string::npos);
  const auto marker_it = std::find_if(
    marker_messages_.back().markers.begin(), marker_messages_.back().markers.end(),
    [](const auto & marker) { return marker.text == "active_end_line[400]"; });
  ASSERT_NE(marker_it, marker_messages_.back().markers.end());
  EXPECT_DOUBLE_EQ(marker_it->pose.position.x, 40.0);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, IgnoresEndLineOnAnotherBranch)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, {{300, 30.0, 20.0}, {400, 40.0, 0.0}});
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto states = make_vtl_states(instrument_id, true, false, node_->now());
  auto input = make_vtl_input(
    make_odometry(36.0, 0.0, node_->now()), lanelet_map, make_route(lanelet_id), states);
  auto trajectory = make_straight_trajectory(36.0, 36.5);

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->publish_debug_data("trajectory_0");
  plugin_->end_cycle();
  spin_until([this]() { return !text_messages_.empty(); });

  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  ASSERT_FALSE(text_messages_.empty());
  EXPECT_NE(text_messages_.back().data.find("ACTIVE_END_LINE_ID: 400"), std::string::npos);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, InvalidEndLineKeepsRequestingAndStopsFailSafe)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, {{400, 15.0, 0.0}});
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto states = make_vtl_states(instrument_id, true, false, node_->now());
  auto input = make_vtl_input(
    make_odometry(10.0, 5.0, node_->now()), lanelet_map, make_route(lanelet_id), states);
  auto trajectory = make_straight_trajectory(0.0, 45.0);

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->publish_debug_data("trajectory_0");
  plugin_->end_cycle();
  spin_until([this]() { return !text_messages_.empty() && !command_messages_.empty(); });

  ASSERT_FALSE(command_messages_.empty());
  ASSERT_EQ(command_messages_.back().commands.size(), 1U);
  EXPECT_EQ(
    command_messages_.back().commands.front().state,
    static_cast<uint8_t>(VirtualTrafficLightStop::ModuleState::REQUESTING));
  ASSERT_FALSE(text_messages_.empty());
  EXPECT_NE(text_messages_.back().data.find("ACTIVE_END_LINE_ID: INVALID"), std::string::npos);
  EXPECT_NE(text_messages_.back().data.find("reason=INVALID_END_LINE"), std::string::npos);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, CurvedShortPathReplacesAtEgoNearEndLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr lanelet::Id end_line_id = 400;
  auto lanelet_map = make_curved_vtl_map(instrument_id, end_line_id);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto states = make_vtl_states(instrument_id, true, false, node_->now());
  auto input = make_vtl_input(
    make_odometry(12.0, 4.0, M_PI_4, 0.0, node_->now()), lanelet_map, make_route(lanelet_id),
    states);
  auto trajectory = make_curved_short_trajectory();

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->publish_debug_data("trajectory_0");
  plugin_->end_cycle();
  spin_until([this]() { return !text_messages_.empty(); });

  ASSERT_EQ(trajectory.size(), 3U);
  EXPECT_NEAR(trajectory.front().pose.position.x, 12.0, 1e-6);
  EXPECT_NEAR(trajectory.front().pose.position.y, 4.0, 1e-6);
  EXPECT_TRUE(std::all_of(trajectory.begin(), trajectory.end(), [](const auto & point) {
    return point.longitudinal_velocity_mps == 0.0F && point.acceleration_mps2 == 0.0F;
  }));
  ASSERT_FALSE(text_messages_.empty());
  EXPECT_NE(text_messages_.back().data.find("target=END_LINE"), std::string::npos);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ShortPathNearEndLineDoesNotKeepMovingTail)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, {{400, 40.0, 0.0}}, 40.5);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto states = make_vtl_states(instrument_id, true, false, node_->now());
  auto input = make_vtl_input(
    make_odometry(36.0, 0.0, node_->now()), lanelet_map, make_route(lanelet_id), states);
  auto trajectory = make_straight_trajectory(36.0, 36.5);

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_LE(trajectory.back().pose.position.x, geometric_stop_x(40.0) + 0.3);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, PrecedingZeroSpeedKeepsStopLineTrajectoryUnchanged)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto input = make_vtl_input(
    make_odometry(10.0, 5.0, node_->now()), lanelet_map, make_route(lanelet_id),
    make_vtl_states(instrument_id, false, false, node_->now()));
  auto trajectory = make_straight_trajectory(0.0, 45.0);
  set_zero_velocity_at_x(trajectory, 12.0);
  const auto original = trajectory;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  expect_same_trajectory(trajectory, original);
  EXPECT_TRUE(plugin_->get_planning_factors().empty());
}

TEST_F(VirtualTrafficLightStopIntegrationTest, DuplicateStopAtStopLineDoesNotRewriteCreep)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto input = make_vtl_input(
    make_odometry(10.0, 5.0, node_->now()), lanelet_map, make_route(lanelet_id),
    VirtualTrafficLightStateArray::ConstSharedPtr{});
  auto trajectory = make_straight_trajectory(0.0, 45.0);
  const auto geometric_stop_x = 20.0 - context_->vehicle_info.max_longitudinal_offset_m;
  set_zero_velocity_at_x(trajectory, std::round(geometric_stop_x));
  const auto original = trajectory;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  expect_same_trajectory(trajectory, original);
  EXPECT_TRUE(plugin_->get_planning_factors().empty());
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 5.0F);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, PrecedingZeroSpeedDoesNotExtendOrCropEndLine)
{
  constexpr lanelet::Id instrument_id = 12345;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto input = make_vtl_input(
    make_odometry(25.0, 5.0, node_->now()), lanelet_map, make_route(lanelet_id),
    make_vtl_states(instrument_id, true, false, node_->now()));
  auto trajectory = make_straight_trajectory(10.0, 45.0);
  set_zero_velocity_at_x(trajectory, 28.0);
  const auto original = trajectory;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  expect_same_trajectory(trajectory, original);
  EXPECT_TRUE(plugin_->get_planning_factors().empty());
  EXPECT_DOUBLE_EQ(trajectory.back().pose.position.x, 45.0);
  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 5.0F);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, PrecedingZeroSkipsArrivedEndLineReplace)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double end_x = 40.0;
  const auto ego_x = geometric_stop_x(end_x);
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto input = make_vtl_input(
    make_odometry(ego_x, 1.0, node_->now()), lanelet_map, make_route(lanelet_id),
    make_vtl_states(instrument_id, true, false, node_->now()));
  auto trajectory = make_straight_trajectory(10.0, 45.0);
  set_zero_velocity_at_x(trajectory, 15.0);
  const auto original = trajectory;

  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  expect_same_trajectory(trajectory, original);
  EXPECT_TRUE(plugin_->get_planning_factors().empty());
}

TEST_F(VirtualTrafficLightStopIntegrationTest, PrecedingZeroDoesNotSkipArrivedEndLineReplace)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double ego_x = 36.0;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, 40.0);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  auto input = make_vtl_input(
    make_odometry(ego_x, 0.0, node_->now()), lanelet_map, make_route(lanelet_id),
    make_vtl_states(instrument_id, true, false, node_->now()));
  auto trajectory = make_straight_trajectory(ego_x, 36.5);
  trajectory.back().longitudinal_velocity_mps = 0.0F;

  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();

  EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_LE(trajectory.back().pose.position.x, geometric_stop_x(40.0) + 0.3);
  expect_single_stop_factor("VTL 12345: WAITING_FINALIZATION -> END_LINE");
}

TEST_F(VirtualTrafficLightStopIntegrationTest, EndLineStopClampsForwardWhenUnableToBrake)
{
  constexpr lanelet::Id instrument_id = 12345;
  constexpr double end_x = 40.0;
  auto lanelet_map = make_vtl_map(instrument_id, 5.0, 20.0, end_x);
  const auto lanelet_id = lanelet_map->laneletLayer.begin()->id();
  const auto stop_x = geometric_stop_x(end_x);
  constexpr double ego_x = 25.0;

  auto run = [&](const double velocity, const double ax) {
    auto input = make_vtl_input(
      make_odometry(ego_x, velocity, node_->now()), lanelet_map, make_route(lanelet_id),
      make_vtl_states(instrument_id, true, false, node_->now()));
    input.current_acceleration = make_acceleration(ax);
    auto trajectory = make_straight_trajectory(ego_x, 45.0);
    plugin_->begin_cycle(input);
    EXPECT_TRUE(plugin_->modify_trajectory(trajectory, input));
    plugin_->end_cycle();
    return trajectory;
  };

  const auto coasting = run(5.0, 0.0);
  const auto unable_to_brake = run(15.0, 2.0);
  expect_truncated_stop_at_x(coasting, stop_x);
  ASSERT_FALSE(unable_to_brake.empty());
  EXPECT_FLOAT_EQ(unable_to_brake.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_GT(unable_to_brake.back().pose.position.x, coasting.back().pose.position.x + 0.5);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ApprovalMustNotReleaseUnownedZeroStop)
{
  const auto map = make_vtl_map(12345, 5.0, 20.0, 40.0);
  auto input = make_vtl_input(
    make_odometry(10.0, 0.0, node_->now()), map, make_route(map->laneletLayer.begin()->id()),
    make_vtl_states(12345, true, false, node_->now()));
  auto traj = make_straight_trajectory(10.0, 10.002);
  for (auto & p : traj) p.longitudinal_velocity_mps = 0.0F;
  const auto original = traj;
  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->modify_trajectory(traj, input));
  expect_same_trajectory(traj, original);
}
TEST_F(VirtualTrafficLightStopIntegrationTest, DeniedShortHorizonMustRespectFrontOffset)
{
  const auto map = make_vtl_map(12345, 5.0, 20.0, 40.0);
  auto input = make_vtl_input(
    make_odometry(10.0, 1.0, node_->now()), map, make_route(map->laneletLayer.begin()->id()),
    make_vtl_states(12345, false, false, node_->now()));
  auto traj = make_straight_trajectory(10.0, 18.0);
  plugin_->begin_cycle(input);
  EXPECT_TRUE(plugin_->modify_trajectory(traj, input));
  EXPECT_LE(traj.back().pose.position.x, geometric_stop_x(20.0) + 0.1);
  EXPECT_FLOAT_EQ(traj.back().longitudinal_velocity_mps, 0.0F);
}
TEST_F(VirtualTrafficLightStopIntegrationTest, ApprovedModuleMustNotUndoDeniedModule)
{
  auto map = make_vtl_map(12345, 5.0, 20.0, 40.0);
  auto other = make_vtl_map(54321, 5.0, 25.0, 45.0);
  auto lane = *map->laneletLayer.begin();
  for (const auto & reg :
       other->laneletLayer.begin()->regulatoryElementsAs<lanelet::autoware::VirtualTrafficLight>())
    lane.addRegulatoryElement(reg);
  auto states = std::make_shared<VirtualTrafficLightStateArray>();
  states->states = make_vtl_states(12345, false, false, node_->now())->states;
  states->states.push_back(make_vtl_states(54321, true, false, node_->now())->states.front());
  const auto ego_x = geometric_stop_x(20.0);
  auto input =
    make_vtl_input(make_odometry(ego_x, 0.0, node_->now()), map, make_route(lane.id()), states);
  auto traj = make_straight_trajectory(ego_x, 50.0);
  plugin_->begin_cycle(input);
  EXPECT_TRUE(plugin_->modify_trajectory(traj, input));
  EXPECT_FLOAT_EQ(traj.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_LE(traj.back().pose.position.x, ego_x + 0.1);
}
TEST_F(VirtualTrafficLightStopIntegrationTest, ShortHorizonBeforeVehicleFrontReachesLineIsUnchanged)
{
  const auto map = make_vtl_map(12345, 5.0, 20.0, 40.0);
  auto input = make_vtl_input(
    make_odometry(10.0, 1.0, node_->now()), map, make_route(map->laneletLayer.begin()->id()),
    make_vtl_states(12345, false, false, node_->now()));
  auto trajectory = make_straight_trajectory(10.0, 15.0);
  const auto original = trajectory;
  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ApprovalAllowsNewMovingCandidateAfterDeniedStop)
{
  const auto map = make_vtl_map(12345, 5.0, 20.0, 40.0);
  auto input = make_vtl_input(
    make_odometry(10.0, 1.0, node_->now()), map, make_route(map->laneletLayer.begin()->id()),
    make_vtl_states(12345, false, false, node_->now()));
  auto trajectory = make_straight_trajectory(10.0, 30.0);
  plugin_->begin_cycle(input);
  ASSERT_TRUE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();
  input.virtual_traffic_light_states = make_vtl_states(12345, true, false, node_->now());
  trajectory = make_straight_trajectory(10.0, 30.0);
  const auto original = trajectory;
  plugin_->begin_cycle(input);
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, input));
  EXPECT_FALSE(plugin_->modify_trajectory(trajectory, input));
  plugin_->end_cycle();
  expect_same_trajectory(trajectory, original);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, NewFrameworkPublishesOneCommandPerCandidateCycle)
{
  const auto map = make_vtl_map(12345, 5.0, 20.0, 40.0);
  auto snapshot = make_vtl_input(
    make_odometry(10.0, 1.0, node_->now()), map, make_route(map->laneletLayer.begin()->id()),
    make_vtl_states(12345, false, false, node_->now()));
  plugin_->begin_cycle(snapshot);
  for (size_t index = 0; index < 2; ++index) {
    auto data = snapshot;
    data.candidate_index = index;
    data.candidate_count = 2;
    auto trajectory = make_straight_trajectory(10.0, 30.0);
    EXPECT_EQ(
      plugin_->process(trajectory, data),
      autoware::trajectory_modifier::plugin::ProcessingResult::Modified);
    EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
  }
  executor_->spin_some();
  EXPECT_TRUE(command_messages_.empty());
  plugin_->end_cycle();
  spin_until([this]() { return !command_messages_.empty(); });
  ASSERT_EQ(command_messages_.size(), 1U);
  ASSERT_EQ(command_messages_.front().commands.size(), 1U);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, NodeEndsEmptyAndUnchangedCandidateCyclesOnce)
{
  rclcpp::NodeOptions options;
  const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
  autoware::test_utils::updateNodeOptions(
    options, {test_utils_dir + "/config/test_vehicle_info.param.yaml"});
  options.append_parameter_override(
    "plugin_names",
    std::vector<std::string>{"autoware::trajectory_modifier::plugin::VirtualTrafficLightStop"});
  options.append_parameter_override("use_virtual_traffic_light_stop", true);
  auto modifier = std::make_shared<autoware::trajectory_modifier::TrajectoryModifier>(options);
  executor_->add_node(modifier);
  size_t command_count = 0;
  auto commands = node_->create_subscription<InfrastructureCommandArray>(
    "/trajectory_modifier/output/infrastructure_commands", 10,
    [&command_count](const InfrastructureCommandArray::ConstSharedPtr) { ++command_count; });
  auto odometry =
    node_->create_publisher<nav_msgs::msg::Odometry>("/trajectory_modifier/input/odometry", 1);
  auto acceleration = node_->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "/trajectory_modifier/input/acceleration", 1);
  auto candidates =
    node_->create_publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
      "/trajectory_modifier/input/trajectories", 1);
  spin_until([&]() {
    return candidates->get_subscription_count() > 0 && odometry->get_subscription_count() > 0 &&
           acceleration->get_subscription_count() > 0 && commands->get_publisher_count() > 0;
  });
  ASSERT_GT(commands->get_publisher_count(), 0U);
  for (size_t cycle = 0; cycle < 2; ++cycle) {
    odometry->publish(*make_odometry(0.0, 1.0, node_->now()));
    acceleration->publish(geometry_msgs::msg::AccelWithCovarianceStamped{});
    for (size_t i = 0; i < 5; ++i) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    autoware_internal_planning_msgs::msg::CandidateTrajectories batch;
    if (cycle == 1) {
      batch.candidate_trajectories.resize(2);
      for (auto & candidate : batch.candidate_trajectories) {
        candidate.points = make_straight_trajectory(0.0, 30.0);
      }
    }
    candidates->publish(batch);
    spin_until([&]() { return command_count >= cycle + 1; });
    EXPECT_EQ(command_count, cycle + 1);
  }
  executor_->remove_node(modifier);
}
