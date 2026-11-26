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
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
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
#include <rcl/time.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
void PrintTo(const VirtualTrafficLightStop::ModuleState & state, std::ostream * os)
{
  using ModuleState = VirtualTrafficLightStop::ModuleState;
  switch (state) {
    case ModuleState::NONE:
      *os << "NONE";
      return;
    case ModuleState::REQUESTING:
      *os << "REQUESTING";
      return;
    case ModuleState::PASSING:
      *os << "PASSING";
      return;
    case ModuleState::FINALIZING:
      *os << "FINALIZING";
      return;
    case ModuleState::FINALIZED:
      *os << "FINALIZED";
      return;
  }
  *os << "UNKNOWN(" << static_cast<int>(static_cast<std::underlying_type_t<ModuleState>>(state))
      << ")";
}
}  // namespace autoware::trajectory_modifier::plugin

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
using autoware_utils_debug::ProcessingTimeDetail;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using tier4_v2x_msgs::msg::InfrastructureCommandArray;
using tier4_v2x_msgs::msg::VirtualTrafficLightState;
using tier4_v2x_msgs::msg::VirtualTrafficLightStateArray;
using visualization_msgs::msg::MarkerArray;
using ModuleState = VirtualTrafficLightStop::ModuleState;
using StatesPtr = VirtualTrafficLightStateArray::ConstSharedPtr;

constexpr lanelet::Id INSTRUMENT_ID = 12345;
constexpr lanelet::Id SECOND_INSTRUMENT_ID = 54321;
constexpr double START_LINE_X_M = 5.0;
constexpr double STOP_LINE_X_M = 20.0;
constexpr double END_LINE_X_M = 40.0;

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.compare(0, prefix.size(), prefix) == 0;
}

bool ends_with(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool contains(const std::string & value, const std::string & part)
{
  return value.find(part) != std::string::npos;
}

void assign_time_from_start(TrajectoryPoints & trajectory)
{
  for (size_t i = 0; i < trajectory.size(); ++i) {
    trajectory.at(i).time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * 0.2);
  }
}

TrajectoryPoint make_straight_point(const double x)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = 5.0;
  return point;
}

TrajectoryPoints make_trajectory()
{
  TrajectoryPoints trajectory;
  for (size_t i = 0; i < 3; ++i) {
    trajectory.push_back(make_straight_point(static_cast<double>(i)));
  }
  assign_time_from_start(trajectory);
  return trajectory;
}

TrajectoryPoints make_straight_trajectory(const double end_x)
{
  TrajectoryPoints trajectory;
  for (size_t i = 0; i <= static_cast<size_t>(std::ceil(end_x)); ++i) {
    trajectory.push_back(make_straight_point(std::min(static_cast<double>(i), end_x)));
  }
  assign_time_from_start(trajectory);
  return trajectory;
}

//! Points every 1 m from `start_x`, plus `end_x` itself
TrajectoryPoints make_straight_trajectory(const double start_x, const double end_x)
{
  TrajectoryPoints trajectory;
  const auto whole_steps = static_cast<int>(std::floor(end_x - start_x));
  for (int i = 0; i <= whole_steps; ++i) {
    trajectory.push_back(make_straight_point(start_x + static_cast<double>(i)));
  }
  if (trajectory.empty() || trajectory.back().pose.position.x < end_x) {
    trajectory.push_back(make_straight_point(end_x));
  }
  assign_time_from_start(trajectory);
  return trajectory;
}

// Gentle start from standstill sampled every 0.1 s along `yaw`: the first points are < 5 mm apart.
TrajectoryPoints make_standstill_start_trajectory(const double start_x, const double yaw)
{
  constexpr double time_step_s = 0.1;
  constexpr double start_accel_mps2 = 0.5;
  TrajectoryPoints trajectory;
  for (int i = 0; i < 80; ++i) {
    const double t = time_step_s * static_cast<double>(i);
    const double distance_m = 0.5 * start_accel_mps2 * t * t;
    TrajectoryPoint point;
    point.pose.position.x = start_x + distance_m * std::cos(yaw);
    point.pose.position.y = distance_m * std::sin(yaw);
    point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
    point.longitudinal_velocity_mps = static_cast<float>(start_accel_mps2 * t);
    trajectory.push_back(point);
  }
  assign_time_from_start(trajectory);
  return trajectory;
}

TrajectoryPoints with_velocity(TrajectoryPoints trajectory, const float velocity_mps)
{
  for (auto & point : trajectory) {
    point.longitudinal_velocity_mps = velocity_mps;
  }
  return trajectory;
}

TrajectoryPoints with_zero_velocity_at_x(TrajectoryPoints trajectory, const double x)
{
  for (auto & point : trajectory) {
    if (std::abs(point.pose.position.x - x) < 1e-6) {
      point.longitudinal_velocity_mps = 0.0F;
      break;
    }
  }
  return trajectory;
}

TrajectoryPoints with_yaw(TrajectoryPoints trajectory, const double yaw)
{
  for (auto & point : trajectory) {
    point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  }
  return trajectory;
}

TrajectoryPoints shifted_laterally(TrajectoryPoints trajectory, const double y)
{
  for (auto & point : trajectory) {
    point.pose.position.y = y;
  }
  return trajectory;
}

TrajectoryPoints collapsed_to_first_point(TrajectoryPoints trajectory)
{
  for (auto & point : trajectory) {
    point.pose.position = trajectory.front().pose.position;
  }
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

VirtualTrafficLightState make_vtl_state(
  const lanelet::Id instrument_id, const bool approval, const bool is_finalized,
  const rclcpp::Time & stamp)
{
  VirtualTrafficLightState state;
  state.stamp = stamp;
  state.type = "virtual_traffic_light";
  state.id = std::to_string(instrument_id);
  state.approval = approval;
  state.is_finalized = is_finalized;
  return state;
}

StatesPtr make_vtl_states(
  const lanelet::Id instrument_id, const bool approval, const bool is_finalized,
  const rclcpp::Time & stamp)
{
  auto states = std::make_shared<VirtualTrafficLightStateArray>();
  states->stamp = stamp;
  states->states.push_back(make_vtl_state(instrument_id, approval, is_finalized, stamp));
  return states;
}

LaneletRoute::ConstSharedPtr make_route(const std::vector<lanelet::Id> & lanelet_ids)
{
  auto route = std::make_shared<LaneletRoute>();
  for (const auto id : lanelet_ids) {
    LaneletSegment segment;
    segment.preferred_primitive.id = id;
    route->segments.push_back(segment);
  }
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

//! A map with its route
struct VtlScene
{
  std::shared_ptr<lanelet::LaneletMap> map;
  LaneletRoute::ConstSharedPtr route;
};

//! Route along the first lanelet of a single-lane map
VtlScene make_scene(std::shared_ptr<lanelet::LaneletMap> map)
{
  auto route = make_route({map->laneletLayer.begin()->id()});
  return {std::move(map), std::move(route)};
}

VtlScene make_default_scene()
{
  return make_scene(make_vtl_map(INSTRUMENT_ID, START_LINE_X_M, STOP_LINE_X_M, END_LINE_X_M));
}

//! INSTRUMENT_ID stops at `first_stop_x` and comes first in map order; SECOND_INSTRUMENT_ID stops
//! at `second_stop_x`
VtlScene make_two_vtl_scene(const double first_stop_x, const double second_stop_x)
{
  constexpr double end_x = 50.0;
  auto map = make_vtl_map(INSTRUMENT_ID, START_LINE_X_M, first_stop_x, end_x);
  const auto other = make_vtl_map(SECOND_INSTRUMENT_ID, START_LINE_X_M, second_stop_x, end_x);
  auto lane = *map->laneletLayer.begin();
  for (const auto & regulatory_element :
       other->laneletLayer.begin()
         ->regulatoryElementsAs<lanelet::autoware::VirtualTrafficLight>()) {
    lane.addRegulatoryElement(regulatory_element);
  }
  return make_scene(map);
}

//! VTL lane spans x in [0, 100] and the preceding lane x in [-100, 0]; the route covers both
VtlScene make_scene_with_preceding_lane(const double start_x)
{
  auto instrument = make_cross_line(INSTRUMENT_ID, STOP_LINE_X_M + 1.0);
  instrument.attributes()["type"] = "intersection_coordination";

  lanelet::RuleParameterMap parameters;
  parameters[lanelet::RoleNameString::Refers].push_back(instrument);
  parameters[lanelet::RoleNameString::RefLine].push_back(
    make_cross_line(lanelet::utils::getId(), STOP_LINE_X_M));
  parameters["start_line"].push_back(make_cross_line(lanelet::utils::getId(), start_x));
  parameters["end_line"].push_back(make_cross_line(lanelet::utils::getId(), END_LINE_X_M, 0.0));
  auto vtl = lanelet::RegulatoryElementFactory::create(
    lanelet::autoware::VirtualTrafficLight::RuleName, lanelet::utils::getId(), parameters);

  const auto make_lane = [](const double begin_x, const double end_x_of_lane) {
    lanelet::LineString3d left(
      lanelet::utils::getId(),
      {lanelet::Point3d(lanelet::utils::getId(), begin_x, -5.0, 0.0),
       lanelet::Point3d(lanelet::utils::getId(), end_x_of_lane, -5.0, 0.0)});
    lanelet::LineString3d right(
      lanelet::utils::getId(),
      {lanelet::Point3d(lanelet::utils::getId(), begin_x, 5.0, 0.0),
       lanelet::Point3d(lanelet::utils::getId(), end_x_of_lane, 5.0, 0.0)});
    return lanelet::Lanelet(lanelet::utils::getId(), left, right);
  };
  auto preceding_lane = make_lane(-100.0, 0.0);
  auto vtl_lane = make_lane(0.0, 100.0);
  vtl_lane.addRegulatoryElement(vtl);

  return {
    lanelet::utils::createMap({preceding_lane, vtl_lane}),
    make_route({preceding_lane.id(), vtl_lane.id()})};
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
  lane.setCenterline(
    lanelet::LineString3d(
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

double braking_distance_m(
  const double velocity_mps, const double acceleration_mps2, const double max_deceleration_mps2,
  const double jerk_limit_mps3)
{
  const auto distance = autoware::motion_utils::calculate_stop_distance(
    velocity_mps, acceleration_mps2, max_deceleration_mps2, jerk_limit_mps3, 0.0);
  EXPECT_TRUE(distance.has_value());
  return distance.value_or(0.0);
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

//! Result of a batch with one candidate
struct BatchResult
{
  bool modified{false};
  ModuleState state{ModuleState::NONE};
};
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
    processing_time_pub_ =
      node_->create_publisher<ProcessingTimeDetail>("~/debug/processing_time_detail", 1);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(processing_time_pub_);
    context_ = std::make_shared<TrajectoryModifierContext>(node_.get());

    marker_sub_ = node_->create_subscription<MarkerArray>(
      "~/virtual_traffic_light_stop/debug/marker", 10,
      [this](const MarkerArray::ConstSharedPtr msg) { marker_messages_.push_back(*msg); });
    text_sub_ = node_->create_subscription<StringStamped>(
      "~/virtual_traffic_light_stop/debug/text", 10,
      [this](const StringStamped::ConstSharedPtr msg) { text_messages_.push_back(*msg); });
    processing_time_sub_ = node_->create_subscription<ProcessingTimeDetail>(
      "~/debug/processing_time_detail", 20, [this](const ProcessingTimeDetail::ConstSharedPtr msg) {
        processing_time_messages_.push_back(*msg);
      });
    command_sub_ = node_->create_subscription<InfrastructureCommandArray>(
      "~/output/infrastructure_commands", 10,
      [this](const InfrastructureCommandArray::ConstSharedPtr msg) {
        command_messages_.push_back(*msg);
      });
    executor_->add_node(node_);

    params_.use_virtual_traffic_light_stop = true;
    params_.virtual_traffic_light_stop.max_delay_sec = 3.0;
    params_.virtual_traffic_light_stop.near_line_distance = 1.0;
    params_.virtual_traffic_light_stop.dead_line_margin = 1.0;
    params_.virtual_traffic_light_stop.max_yaw_deviation_deg = 90.0;
    params_.virtual_traffic_light_stop.check_timeout_after_stop_line = true;
    params_.virtual_traffic_light_stop.min_hold_trajectory_length = 5.0;
    params_.stopping_constraints.nominal_deceleration = 1.0;
    params_.stopping_constraints.maximum_deceleration = 4.0;
    params_.stopping_constraints.jerk_limit = 3.0;
    params_.stopping_constraints.arrived_distance_threshold = 0.5;
    params_.stopping_constraints.delay_response_time = 0.5;
    params_.stopping_constraints.ego_stopped_vel_th = 0.1;

    reset_plugin();
    states_pub_ = node_->create_publisher<VirtualTrafficLightStateArray>(
      "~/input/virtual_traffic_light_states", rclcpp::QoS{1});
  }

  void TearDown() override
  {
    executor_->remove_node(node_);
    plugin_.reset();
    states_pub_.reset();
    context_.reset();
    time_keeper_.reset();
    node_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  //! Replaces the plugin with a new one built from params(), so that table rows do not share
  //! module state
  void reset_plugin()
  {
    plugin_ = std::make_unique<VirtualTrafficLightStop>();
    plugin_->initialize(
      "test_virtual_traffic_light_stop", node_.get(), time_keeper_, context_, params_);
  }

  rclcpp::Time now() const { return node_->now(); }
  rclcpp::Node & node() { return *node_; }
  rclcpp::executors::SingleThreadedExecutor & executor() { return *executor_; }
  VirtualTrafficLightStop & plugin() { return *plugin_; }
  trajectory_modifier_params::Params & params() { return params_; }
  const std::vector<MarkerArray> & markers() const { return marker_messages_; }
  const std::vector<StringStamped> & texts() const { return text_messages_; }
  const std::vector<ProcessingTimeDetail> & timings() const { return processing_time_messages_; }
  const std::vector<InfrastructureCommandArray> & commands() const { return command_messages_; }

  void spin_until(const std::function<bool()> & condition)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!condition() && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_->spin_some();
  }

  bool process_candidate(TrajectoryPoints & trajectory, TrajectoryModifierData & snapshot)
  {
    // Match the framework: each candidate gets a fresh copy with its position in this batch.
    auto input = snapshot;
    if (input.candidate_count == 0U) input.candidate_count = 1U;
    const auto result = plugin_->process(trajectory, input);
    ++snapshot.candidate_index;
    return result == autoware::trajectory_modifier::plugin::ProcessingResult::Modified;
  }

  //! Processes a batch of one candidate and reads the module state from the command it publishes
  BatchResult process_batch(TrajectoryPoints & trajectory, TrajectoryModifierData input)
  {
    const auto previous_command_count = command_messages_.size();
    input.candidate_index = 0U;
    input.candidate_count = 1U;
    BatchResult result;
    result.modified = process_candidate(trajectory, input);
    spin_until([&]() { return command_messages_.size() > previous_command_count; });
    if (
      command_messages_.size() <= previous_command_count ||
      command_messages_.back().commands.size() != 1U) {
      ADD_FAILURE() << "expected one infrastructure command after the batch";
      return result;
    }
    result.state = static_cast<ModuleState>(command_messages_.back().commands.front().state);
    return result;
  }

  void publish_vtl_states(const VirtualTrafficLightStateArray & states)
  {
    spin_until([this]() { return states_pub_->get_subscription_count() > 0U; });
    ASSERT_GT(states_pub_->get_subscription_count(), 0U);
    states_pub_->publish(states);
    // Wait for DDS receipt before the plugin polls; executor callbacks do not consume this input.
    ASSERT_TRUE(states_pub_->wait_for_all_acked(std::chrono::seconds(2)));
  }

  TrajectoryModifierData make_vtl_input(
    Odometry::ConstSharedPtr odometry, std::shared_ptr<lanelet::LaneletMap> lanelet_map,
    LaneletRoute::ConstSharedPtr route, StatesPtr states)
  {
    if (states) {
      publish_vtl_states(*states);
    }
    TrajectoryModifierData input;
    input.current_odometry = std::move(odometry);
    input.current_acceleration = make_acceleration(0.0);
    input.lanelet_map = std::move(lanelet_map);
    input.route = std::move(route);
    return input;
  }

  //! Publishes `states` (none if null) and returns the input of a straight-driving ego
  TrajectoryModifierData make_input(
    const VtlScene & scene, const double ego_x, const double velocity_mps, StatesPtr states)
  {
    return make_vtl_input(
      make_odometry(ego_x, velocity_mps, now()), scene.map, scene.route, std::move(states));
  }

  //! V2X answer for INSTRUMENT_ID, stamped `age_s` before now
  StatesPtr answer(const bool approval, const bool is_finalized = false, const double age_s = 0.0)
  {
    return make_vtl_states(
      INSTRUMENT_ID, approval, is_finalized, now() - rclcpp::Duration::from_seconds(age_s));
  }

  StatesPtr make_two_vtl_states(const bool first_approval, const bool second_approval)
  {
    auto states = std::make_shared<VirtualTrafficLightStateArray>();
    states->stamp = now();
    states->states.push_back(make_vtl_state(INSTRUMENT_ID, first_approval, false, now()));
    states->states.push_back(make_vtl_state(SECOND_INSTRUMENT_ID, second_approval, false, now()));
    return states;
  }

  void expect_single_stop_factor(const std::string & detail)
  {
    const auto factors = plugin_->get_planning_factors();
    ASSERT_EQ(factors.size(), 1U);
    EXPECT_EQ(factors.front().behavior, PlanningFactor::STOP);
    EXPECT_EQ(factors.front().detail, detail);
    EXPECT_EQ(factors.front().module, "modifier_virtual_traffic_light_stop");
  }

  double braking_distance(const double velocity, const double acceleration = 0.0) const
  {
    return braking_distance_m(
      velocity, acceleration, params_.stopping_constraints.maximum_deceleration,
      params_.stopping_constraints.jerk_limit);
  }

  double moving_stop_x(const double line_x, const double ego_x, const double velocity) const
  {
    return std::max(geometric_stop_x(line_x), ego_x + braking_distance(velocity));
  }

  //! Ego x at which the vehicle front touches the line at `line_x`
  double geometric_stop_x(const double line_x) const
  {
    return line_x - context_->vehicle_info.max_longitudinal_offset_m;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<ProcessingTimeDetail>::SharedPtr processing_time_pub_;
  rclcpp::Subscription<MarkerArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<StringStamped>::SharedPtr text_sub_;
  rclcpp::Subscription<ProcessingTimeDetail>::SharedPtr processing_time_sub_;
  rclcpp::Subscription<InfrastructureCommandArray>::SharedPtr command_sub_;
  rclcpp::Publisher<VirtualTrafficLightStateArray>::SharedPtr states_pub_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<TrajectoryModifierContext> context_;
  std::unique_ptr<VirtualTrafficLightStop> plugin_;
  trajectory_modifier_params::Params params_;
  std::vector<MarkerArray> marker_messages_;
  std::vector<StringStamped> text_messages_;
  std::vector<ProcessingTimeDetail> processing_time_messages_;
  std::vector<InfrastructureCommandArray> command_messages_;
};

TEST_F(VirtualTrafficLightStopIntegrationTest, CandidateIsUnchangedWhenNoStopApplies)
{
  {
    SCOPED_TRACE("plugin disabled");
    params().use_virtual_traffic_light_stop = false;
    reset_plugin();
    auto trajectory = make_trajectory();
    const auto original = trajectory;
    TrajectoryModifierData input;
    EXPECT_FALSE(process_candidate(trajectory, input));
    expect_same_trajectory(trajectory, original);
    params().use_virtual_traffic_light_stop = true;
  }
  {
    SCOPED_TRACE("no map, no route and no acceleration");
    reset_plugin();
    auto trajectory = make_trajectory();
    const auto original = trajectory;
    TrajectoryModifierData input;
    input.current_odometry = make_odometry(0.0, 0.0, now());
    EXPECT_FALSE(process_candidate(trajectory, input));
    expect_same_trajectory(trajectory, original);
  }

  struct Case
  {
    std::string name;
    VtlScene scene;
    StatesPtr states;
    double ego_x;
    double ego_velocity_mps;
    TrajectoryPoints trajectory;
  };
  const auto scene = make_default_scene();
  const auto distant_scene = make_scene(make_vtl_map(INSTRUMENT_ID, START_LINE_X_M, 80.0, 90.0));
  const std::vector<Case> cases{
    {"candidate facing backwards", scene, answer(false), 10.0, 1.0,
     with_yaw(make_straight_trajectory(0.0, 45.0), M_PI)},
    {"candidate 10 m beside ego", scene, answer(false), 10.0, 1.0,
     shifted_laterally(make_straight_trajectory(0.0, 45.0), 10.0)},
    {"candidate collapsed to one pose", scene, answer(false), 10.0, 1.0,
     collapsed_to_first_point(make_straight_trajectory(0.0, 3.0))},
    {"single-point candidate", scene, answer(false), 10.0, 1.0, TrajectoryPoints(1)},
    {"denied, front never reaches the stop line", scene, answer(false), 10.0, 1.0,
     make_straight_trajectory(10.0, 15.0)},
    {"no state, stop line beyond the horizon", distant_scene, nullptr, 10.0, 5.0,
     make_straight_trajectory(10.0, 15.0)},
    {"approved, stopped before the stop line", scene, answer(true), 10.0, 0.0,
     make_straight_trajectory(10.0, 15.0)},
    {"approved, 0.1 m creeping candidate", scene, answer(true), 10.0, 0.0,
     with_velocity(make_straight_trajectory(10.0, 10.1), 0.02F)},
    {"approved, between the lines, horizon ends before the end line", scene, answer(true), 25.0,
     5.0, make_straight_trajectory(25.0, 30.0)},
  };

  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    reset_plugin();
    auto input = make_input(row.scene, row.ego_x, row.ego_velocity_mps, row.states);
    auto trajectory = row.trajectory;
    EXPECT_FALSE(process_candidate(trajectory, input));
    expect_same_trajectory(trajectory, row.trajectory);
    EXPECT_TRUE(plugin().get_planning_factors().empty());
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, StopsAtStopLineWithoutRightOfWay)
{
  struct Case
  {
    std::string name;
    StatesPtr states;
    double ego_velocity_mps;
    TrajectoryPoints trajectory;
    std::string detail;
  };
  constexpr double ego_x = 10.0;
  const std::vector<Case> cases{
    {"denied, candidate ends before the end line", answer(false), 5.0,
     make_straight_trajectory(25.0), "VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE"},
    {"denied and finalized", answer(false, true), 5.0, make_straight_trajectory(25.0),
     "VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE"},
    {"no state", nullptr, 5.0, make_straight_trajectory(25.0), "VTL 12345: NO_STATE -> STOP_LINE"},
    {"approval older than max_delay_sec", answer(true, false, 4.0), 5.0,
     make_straight_trajectory(25.0), "VTL 12345: STATE_TIMEOUT_BEFORE_STOP_LINE -> STOP_LINE"},
    {"finalized approval older than max_delay_sec", answer(true, true, 4.0), 5.0,
     make_straight_trajectory(25.0), "VTL 12345: STATE_TIMEOUT_BEFORE_STOP_LINE -> STOP_LINE"},
    {"denied, candidate ends between the front-offset stop and the line", answer(false), 1.0,
     make_straight_trajectory(ego_x, 18.0), "VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE"},
  };

  const auto scene = make_default_scene();
  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    reset_plugin();
    auto input = make_input(scene, ego_x, row.ego_velocity_mps, row.states);
    auto trajectory = row.trajectory;
    ASSERT_TRUE(process_candidate(trajectory, input));
    expect_single_stop_factor(row.detail);
    expect_truncated_stop_at_x(
      trajectory, moving_stop_x(STOP_LINE_X_M, ego_x, row.ego_velocity_mps), 0.1);
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, StopsAtEgoWhenPastStopLineWithoutRightOfWay)
{
  struct Case
  {
    std::string name;
    StatesPtr states;
    double ego_x;
    double ego_velocity_mps;
    TrajectoryPoints trajectory;
    std::string detail;
  };
  const std::vector<Case> cases{
    {"denied, moving, stop line behind the candidate", answer(false), 22.0, 1.0,
     make_straight_trajectory(22.0, 30.0), "VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE"},
    {"denied, stopped past the stop line", answer(false), 22.0, 0.0,
     make_straight_trajectory(22.0, 30.0), "VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE"},
    {"approval older than max_delay_sec", answer(true, false, 4.0), 25.0, 5.0,
     make_straight_trajectory(10.0, 45.0), "VTL 12345: STATE_TIMEOUT_AFTER_STOP_LINE -> STOP_LINE"},
  };

  const auto scene = make_default_scene();
  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    reset_plugin();
    auto input = make_input(scene, row.ego_x, row.ego_velocity_mps, row.states);
    auto trajectory = row.trajectory;
    ASSERT_TRUE(process_candidate(trajectory, input));
    expect_single_stop_factor(row.detail);
    expect_truncated_stop_at_x(trajectory, row.ego_x + braking_distance(row.ego_velocity_mps), 0.1);
    if (row.ego_velocity_mps > 0.0) {
      EXPECT_GT(trajectory.front().longitudinal_velocity_mps, 0.0F);
    }
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, StopIsClampedToMaximumDecelerationFromEgo)
{
  struct Case
  {
    std::string name;
    StatesPtr states;
    double ego_x;
    double ego_velocity_mps;
    double ego_acceleration_mps2;
    TrajectoryPoints trajectory;
    double expected_stop_x;
    std::string detail;
    double tolerance_m{0.1};
    double delay_response_time_s{0.5};
    double maximum_deceleration_mps2{4.0};
    double jerk_limit_mps3{3.0};
  };
  const std::string stop_line_detail = "VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE";
  const std::string end_line_detail = "VTL 12345: WAITING_FINALIZATION -> END_LINE";
  const double at_end_line_x = geometric_stop_x(END_LINE_X_M);
  const double short_candidate_x = at_end_line_x - 4.0;
  const std::vector<Case> cases{
    {"stop line, candidate starts 15 m behind ego", answer(false), 15.0, 5.0, 0.0,
     make_straight_trajectory(0.0, 45.0), 15.0 + braking_distance(5.0), stop_line_detail},
    {"stop line, candidate starts 5 m behind ego", answer(false), 15.0, 5.0, 0.0,
     make_straight_trajectory(10.0, 45.0), 15.0 + braking_distance(5.0), stop_line_detail},
    {"stop line, candidate starts at ego", answer(false), 15.0, 5.0, 0.0,
     make_straight_trajectory(15.0, 45.0), 15.0 + braking_distance(5.0), stop_line_detail},
    {"stop line, response delay is not added", answer(false), 15.0, 5.0, 0.0,
     make_straight_trajectory(0.0, 60.0), 15.0 + braking_distance(5.0), stop_line_detail, 0.1, 0.8},
    {"stop line, accelerating ego", answer(false), 15.0, 5.0, 1.0,
     make_straight_trajectory(0.0, 60.0), 15.0 + braking_distance(5.0, 1.0), stop_line_detail, 0.1,
     0.8},
    {"stop line, weaker shared limits", answer(false), 15.0, 5.0, 1.0,
     make_straight_trajectory(0.0, 60.0), 15.0 + braking_distance_m(5.0, 1.0, 2.0, 1.0),
     stop_line_detail, 0.1, 0.8, 2.0, 1.0},
    {"end line, coasting, can stop at the line", answer(true), 25.0, 5.0, 0.0,
     make_straight_trajectory(10.0, 45.0), at_end_line_x, end_line_detail},
    {"end line, front at the line", answer(true), at_end_line_x, 1.0, 0.0,
     make_straight_trajectory(10.0, 45.0), at_end_line_x + braking_distance(1.0), end_line_detail},
    {"end line, braking distance beyond the candidate", answer(true), 25.0, 15.0, 2.0,
     make_straight_trajectory(25.0, 45.0), 45.0, end_line_detail, 1e-6},
    {"end line, short candidate that does not reach the line", answer(true), short_candidate_x,
     15.0, 2.0, make_straight_trajectory(short_candidate_x, short_candidate_x + 0.5),
     short_candidate_x + 0.5, end_line_detail, 1e-6},
  };

  const auto scene = make_default_scene();
  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    params().stopping_constraints.delay_response_time = row.delay_response_time_s;
    params().stopping_constraints.maximum_deceleration = row.maximum_deceleration_mps2;
    params().stopping_constraints.jerk_limit = row.jerk_limit_mps3;
    reset_plugin();
    auto input = make_input(scene, row.ego_x, row.ego_velocity_mps, row.states);
    input.current_acceleration = make_acceleration(row.ego_acceleration_mps2);
    auto trajectory = row.trajectory;
    ASSERT_TRUE(process_candidate(trajectory, input));
    expect_single_stop_factor(row.detail);
    expect_truncated_stop_at_x(trajectory, row.expected_stop_x, row.tolerance_m);
    EXPECT_GT(trajectory.front().longitudinal_velocity_mps, 0.0F);
    if (std::abs(row.expected_stop_x - row.trajectory.back().pose.position.x) < 1e-6) {
      EXPECT_EQ(trajectory.size(), row.trajectory.size())
        << "a stop at the candidate end adds points";
    }
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, StoppedEgoHoldsAtStopLineOnlyWithinHoldDistance)
{
  // The end line is never held. No row sits exactly on the threshold: ego's projection onto the
  // candidate is off by ~2 mm, so the plugin never sees exactly 2.0 m.
  ASSERT_DOUBLE_EQ(params().virtual_traffic_light_stop.hold_position_distance_threshold, 2.0);
  struct Case
  {
    std::string name;
    StatesPtr states;
    double line_x;
    double distance_to_stop_m;
    bool expect_hold;
  };
  const std::vector<Case> cases{
    {"denied, stop line 1.5 m ahead", answer(false), STOP_LINE_X_M, 1.5, true},
    {"denied, stop line 1.99 m ahead", answer(false), STOP_LINE_X_M, 1.99, true},
    {"denied, stop line 2.01 m ahead", answer(false), STOP_LINE_X_M, 2.01, false},
    {"denied, stop line 3.0 m ahead", answer(false), STOP_LINE_X_M, 3.0, false},
    {"waiting for finalization, end line 1.5 m ahead", answer(true), END_LINE_X_M, 1.5, false},
  };

  const auto scene = make_default_scene();
  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    reset_plugin();
    const double ego_x = geometric_stop_x(row.line_x) - row.distance_to_stop_m;
    auto input = make_input(scene, ego_x, 0.0, row.states);
    auto trajectory = make_straight_trajectory(ego_x, 45.0);
    ASSERT_TRUE(process_candidate(trajectory, input));
    if (row.expect_hold) {
      expect_three_point_stop_at_ego(trajectory, ego_x, params().trajectory_time_step);
    } else {
      expect_truncated_stop_at_x(trajectory, geometric_stop_x(row.line_x), 0.1);
    }
  }
}

TEST_F(
  VirtualTrafficLightStopIntegrationTest, FinalizingStartsOnlyWhenStoppedWithinNearLineDistance)
{
  // near_line_distance is 1.0 m; "stopped" is the 3D speed against the shared ego_stopped_vel_th.
  struct Case
  {
    std::string name;
    double front_to_end_line_m;
    double vx_mps;
    double vy_mps;
    double vz_mps;
    double stopped_velocity_threshold_mps;
    ModuleState expected_state;
  };
  const std::vector<Case> cases{
    {"stopped 0.5 m short", 0.5, 0.0, 0.0, 0.0, 0.1, ModuleState::FINALIZING},
    {"stopped 0.99 m short", 0.99, 0.0, 0.0, 0.0, 0.1, ModuleState::FINALIZING},
    {"stopped 1.01 m short", 1.01, 0.0, 0.0, 0.0, 0.1, ModuleState::PASSING},
    {"stopped 1.5 m short", 1.5, 0.0, 0.0, 0.0, 0.1, ModuleState::PASSING},
    {"0.02 m/s noise", 0.6, 0.02, 0.0, 0.0, 0.1, ModuleState::FINALIZING},
    {"-0.02 m/s noise", 0.6, -0.02, 0.0, 0.0, 0.1, ModuleState::FINALIZING},
    {"speed at the threshold", 0.6, 0.1, 0.0, 0.0, 0.1, ModuleState::FINALIZING},
    {"reverse speed at the threshold", 0.6, -0.1, 0.0, 0.0, 0.1, ModuleState::FINALIZING},
    {"0.11 m/s", 0.6, 0.11, 0.0, 0.0, 0.1, ModuleState::PASSING},
    {"-0.11 m/s", 0.6, -0.11, 0.0, 0.0, 0.1, ModuleState::PASSING},
    {"3D speed 0.087 m/s", 0.6, 0.05, 0.05, 0.05, 0.1, ModuleState::FINALIZING},
    {"3D speed 0.104 m/s", 0.6, 0.06, 0.06, 0.06, 0.1, ModuleState::PASSING},
    {"0.05 m/s with a 0.04 m/s threshold", 0.6, 0.05, 0.0, 0.0, 0.04, ModuleState::PASSING},
    {"0.15 m/s with a 0.2 m/s threshold", 0.6, 0.15, 0.0, 0.0, 0.2, ModuleState::FINALIZING},
  };

  const auto scene = make_default_scene();
  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    params().stopping_constraints.ego_stopped_vel_th = row.stopped_velocity_threshold_mps;
    reset_plugin();
    const double ego_x = geometric_stop_x(END_LINE_X_M) - row.front_to_end_line_m;
    auto odometry = std::make_shared<Odometry>(*make_odometry(ego_x, row.vx_mps, now()));
    odometry->twist.twist.linear.y = row.vy_mps;
    odometry->twist.twist.linear.z = row.vz_mps;
    auto input = make_vtl_input(odometry, scene.map, scene.route, answer(true));
    auto trajectory = make_straight_trajectory(ego_x, 45.0);
    const auto result = process_batch(trajectory, input);
    EXPECT_TRUE(result.modified);
    EXPECT_EQ(result.state, row.expected_state);
    if (row.expected_state == ModuleState::FINALIZING) {
      expect_three_point_stop_at_ego(trajectory, ego_x, params().trajectory_time_step);
    } else {
      expect_truncated_stop_at_x(trajectory, geometric_stop_x(END_LINE_X_M), 0.1);
      EXPECT_GT(trajectory.front().longitudinal_velocity_mps, 0.0F);
    }
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, FinalizingHoldsEgoUntilTheEndLineIsPassed)
{
  {
    SCOPED_TRACE("FINALIZING is kept through finalization and motion until the end line");
    const auto scene = make_default_scene();
    const auto run_cycle = [&](const double ego_x, const double velocity, const bool finalized) {
      auto input = make_input(scene, ego_x, velocity, answer(true, finalized));
      TrajectoryPoints unusable_candidate(1);
      const auto result = process_batch(unusable_candidate, input);
      EXPECT_FALSE(result.modified);
      return result.state;
    };
    const auto moving_velocity = params().stopping_constraints.ego_stopped_vel_th + 0.4;
    EXPECT_EQ(run_cycle(25.0, 5.0, false), ModuleState::PASSING);
    EXPECT_EQ(run_cycle(36.0, 0.0, false), ModuleState::FINALIZING);
    EXPECT_EQ(run_cycle(36.0, 0.0, true), ModuleState::FINALIZING);
    EXPECT_EQ(run_cycle(36.0, moving_velocity, true), ModuleState::FINALIZING);
    EXPECT_EQ(run_cycle(42.0, 5.0, true), ModuleState::FINALIZED);
  }
  {
    SCOPED_TRACE("curved short candidate near the end line is replaced at ego");
    reset_plugin();
    const auto map = make_curved_vtl_map(INSTRUMENT_ID, 400);
    auto input = make_vtl_input(
      make_odometry(12.0, 4.0, M_PI_4, 0.0, now()), map,
      make_route({map->laneletLayer.begin()->id()}), answer(true));
    auto trajectory = make_curved_short_trajectory();
    const auto previous_text_count = texts().size();
    ASSERT_TRUE(process_candidate(trajectory, input));
    plugin().publish_debug_data("trajectory_0");
    spin_until([&]() { return texts().size() > previous_text_count; });

    ASSERT_EQ(trajectory.size(), 3U);
    EXPECT_NEAR(trajectory.front().pose.position.x, 12.0, 1e-6);
    EXPECT_NEAR(trajectory.front().pose.position.y, 4.0, 1e-6);
    EXPECT_TRUE(std::all_of(trajectory.begin(), trajectory.end(), [](const auto & point) {
      return point.longitudinal_velocity_mps == 0.0F && point.acceleration_mps2 == 0.0F;
    }));
    ASSERT_GT(texts().size(), previous_text_count);
    EXPECT_TRUE(contains(texts().back().data, "target=END_LINE"));
  }
  {
    SCOPED_TRACE("short straight candidate near the end line keeps no moving tail");
    reset_plugin();
    const auto scene = make_scene(
      make_vtl_map(INSTRUMENT_ID, START_LINE_X_M, STOP_LINE_X_M, {{400, 40.0, 0.0}}, 40.5));
    auto input = make_input(scene, 36.0, 0.0, answer(true));
    auto trajectory = make_straight_trajectory(36.0, 36.5);
    ASSERT_TRUE(process_candidate(trajectory, input));
    EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
    EXPECT_LE(trajectory.back().pose.position.x, geometric_stop_x(END_LINE_X_M) + 0.3);
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ApprovalAndFinalizationReleaseTheStop)
{
  struct Case
  {
    std::string name;
    StatesPtr previous_states;
    StatesPtr states;
    double ego_x;
    double ego_velocity_mps;
    TrajectoryPoints trajectory;
  };
  const std::vector<Case> cases{
    {"approved and finalized between the lines", nullptr, answer(true, true), 25.0, 5.0,
     make_straight_trajectory(25.0, 45.0)},
    {"approved, front past the end line", nullptr, answer(true), 42.0, 1.0,
     make_straight_trajectory(35.0, 50.0)},
    {"approval after a denied stop", answer(false), answer(true), 10.0, 1.0,
     make_straight_trajectory(10.0, 30.0)},
  };

  const auto scene = make_default_scene();
  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    reset_plugin();
    if (row.previous_states) {
      auto previous_input = make_input(scene, row.ego_x, row.ego_velocity_mps, row.previous_states);
      auto previous_trajectory = row.trajectory;
      ASSERT_TRUE(process_candidate(previous_trajectory, previous_input));
    }
    auto input = make_input(scene, row.ego_x, row.ego_velocity_mps, row.states);
    auto trajectory = row.trajectory;
    EXPECT_FALSE(process_candidate(trajectory, input));
    expect_same_trajectory(trajectory, row.trajectory);
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ModuleStateFollowsTheRoutePosition)
{
  // dead_line_margin is 1.0 m. The start line lies on the preceding lanelet of the route.
  constexpr double start_x = -20.0;
  struct Case
  {
    std::string name;
    double ego_x;
    double ego_velocity_mps;
    ModuleState expected_state;
  };
  const std::vector<Case> cases{
    {"far before the start line", -90.0, 5.0, ModuleState::NONE},
    {"front 1 m before the start line", geometric_stop_x(start_x) - 1.0, 5.0, ModuleState::NONE},
    {"front 1 m past the start line", geometric_stop_x(start_x) + 1.0, 5.0,
     ModuleState::REQUESTING},
    {"approaching the stop line", 10.0, 5.0, ModuleState::REQUESTING},
    {"front 0.99 m past the stop line", geometric_stop_x(STOP_LINE_X_M) + 0.99, 5.0,
     ModuleState::REQUESTING},
    {"front 1.01 m past the stop line", geometric_stop_x(STOP_LINE_X_M) + 1.01, 5.0,
     ModuleState::PASSING},
    {"stopped with the front 0.21 m before the end line", 36.0, 0.0, ModuleState::FINALIZING},
    {"front 0.99 m past the end line", geometric_stop_x(END_LINE_X_M) + 0.99, 5.0,
     ModuleState::PASSING},
    {"front 1.01 m past the end line", geometric_stop_x(END_LINE_X_M) + 1.01, 5.0,
     ModuleState::FINALIZED},
  };

  const auto scene = make_scene_with_preceding_lane(start_x);
  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    reset_plugin();
    auto input = make_input(scene, row.ego_x, row.ego_velocity_mps, answer(true));
    TrajectoryPoints unusable_candidate(1);
    const auto result = process_batch(unusable_candidate, input);
    EXPECT_FALSE(result.modified);
    EXPECT_EQ(result.state, row.expected_state);
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, LeavingTheModuleResetsItsLatches)
{
  {
    SCOPED_TRACE("end-line hold is reset when the same route is driven again");
    const auto scene = make_scene_with_preceding_lane(START_LINE_X_M);
    const auto run_cycle = [&](const double ego_x) {
      auto input = make_input(scene, ego_x, 5.0, answer(true));
      TrajectoryPoints unusable_candidate(1);
      const auto result = process_batch(unusable_candidate, input);
      EXPECT_FALSE(result.modified);
      return result.state;
    };
    EXPECT_EQ(run_cycle(42.0), ModuleState::FINALIZED);
    EXPECT_EQ(run_cycle(-90.0), ModuleState::NONE);

    auto input = make_input(scene, 25.0, 5.0, answer(true, false, 4.0));
    auto trajectory = make_straight_trajectory(10.0, 45.0);
    ASSERT_TRUE(process_candidate(trajectory, input));
    expect_single_stop_factor("VTL 12345: STATE_TIMEOUT_AFTER_STOP_LINE -> STOP_LINE");
  }
  {
    SCOPED_TRACE("unstoppable-line pass latch is cleared");
    params().virtual_traffic_light_stop.allow_if_cannot_stop_distance = 1.0;
    reset_plugin();
    const auto scene = make_default_scene();
    const auto run_cycle = [&](const double ego_x, const double velocity) {
      auto input = make_input(scene, ego_x, velocity, answer(false));
      auto trajectory = make_straight_trajectory(ego_x, 40.0);
      return process_candidate(trajectory, input);
    };
    const double near_line_x = geometric_stop_x(STOP_LINE_X_M) - 0.5;
    EXPECT_FALSE(run_cycle(near_line_x, 2.0));
    EXPECT_FALSE(run_cycle(0.0, 2.0));
    // Slow enough to stop before the line: a stale pass latch would leave this unchanged.
    EXPECT_TRUE(run_cycle(near_line_x, 0.5));
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, LatestStateMessageAppliesFromTheNextBatch)
{
  const auto scene = make_default_scene();
  const auto original = make_straight_trajectory(10.0, 30.0);
  {
    SCOPED_TRACE("approval received between candidates");
    auto input = make_input(scene, 10.0, 1.0, answer(false));
    input.candidate_count = 2U;
    auto first = original;
    ASSERT_TRUE(process_candidate(first, input));
    expect_single_stop_factor("VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE");
    plugin().publish_planning_factor();

    publish_vtl_states(*answer(true));
    auto second = original;
    ASSERT_TRUE(process_candidate(second, input));
    expect_same_trajectory(first, second);
    expect_single_stop_factor("VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE");

    input.candidate_index = 0U;
    input.candidate_count = 1U;
    auto next_batch = original;
    EXPECT_FALSE(process_candidate(next_batch, input));
    expect_same_trajectory(next_batch, original);

    // Without a new message the last received approval is kept.
    input.candidate_index = 0U;
    auto following_batch = original;
    EXPECT_FALSE(process_candidate(following_batch, input));
    expect_same_trajectory(following_batch, original);
  }
  {
    SCOPED_TRACE("an empty state array clears the approval");
    reset_plugin();
    auto input = make_input(scene, 10.0, 1.0, answer(true));
    auto approved = original;
    EXPECT_FALSE(process_candidate(approved, input));
    expect_same_trajectory(approved, original);

    publish_vtl_states(VirtualTrafficLightStateArray{});
    input.candidate_index = 0U;
    auto no_state = original;
    ASSERT_TRUE(process_candidate(no_state, input));
    expect_truncated_stop_at_x(no_state, geometric_stop_x(STOP_LINE_X_M), 0.1);
    expect_single_stop_factor("VTL 12345: NO_STATE -> STOP_LINE");
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, EveryCandidateOfABatchUsesTheSameCycleInputs)
{
  {
    SCOPED_TRACE("odometry and acceleration");
    const auto scene = make_default_scene();
    auto input = make_input(scene, 15.0, 5.0, answer(false));
    input.current_acceleration = make_acceleration(1.0);
    input.candidate_count = 2U;
    auto first = make_straight_trajectory(0.0, 60.0);
    ASSERT_TRUE(process_candidate(first, input));
    input.current_odometry = make_odometry(22.0, 1.0, now());
    input.current_acceleration = make_acceleration(0.0);
    auto second = make_straight_trajectory(0.0, 60.0);
    ASSERT_TRUE(process_candidate(second, input));
    expect_same_trajectory(first, second);
    expect_truncated_stop_at_x(first, 15.0 + braking_distance(5.0, 1.0), 0.1);

    input.candidate_index = 0U;
    input.candidate_count = 1U;
    auto next_cycle = make_straight_trajectory(22.0, 60.0);
    ASSERT_TRUE(process_candidate(next_cycle, input));
    expect_truncated_stop_at_x(next_cycle, 22.0 + braking_distance(1.0), 0.1);
  }
  {
    SCOPED_TRACE("time for the state timeout and the debug age");
    reset_plugin();
    auto * clock = node().get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock, 10000000000LL), RCL_RET_OK);
    const auto scene = make_default_scene();
    auto input = make_input(scene, 10.0, 5.0, answer(true, false, 2.9));
    input.candidate_count = 2U;
    const auto original = make_straight_trajectory(0.0, 45.0);
    auto first = original;
    EXPECT_FALSE(process_candidate(first, input));
    ASSERT_EQ(rcl_set_ros_time_override(clock, 12000000000LL), RCL_RET_OK);
    auto second = original;
    EXPECT_FALSE(process_candidate(second, input));
    expect_same_trajectory(first, original);
    expect_same_trajectory(second, original);
    const auto previous_text_count = texts().size();
    plugin().publish_debug_data("second_candidate");
    spin_until([&]() { return texts().size() > previous_text_count; });
    ASSERT_GT(texts().size(), previous_text_count);
    EXPECT_TRUE(contains(texts().back().data, "age_sec=2.90 timeout=false"));

    input.candidate_index = 0U;
    input.candidate_count = 1U;
    auto next_cycle = original;
    ASSERT_TRUE(process_candidate(next_cycle, input));
    expect_single_stop_factor("VTL 12345: STATE_TIMEOUT_BEFORE_STOP_LINE -> STOP_LINE");
  }
}

TEST_F(
  VirtualTrafficLightStopIntegrationTest, ExistingStopsArePreservedAndModulesNeverUndoEachOther)
{
  struct ZeroStopCase
  {
    std::string name;
    StatesPtr states;
    double ego_x;
    double ego_velocity_mps;
    TrajectoryPoints trajectory;
  };
  const std::vector<ZeroStopCase> zero_stop_cases{
    {"denied, zero velocity before the stop line", answer(false), 10.0, 5.0,
     with_zero_velocity_at_x(make_straight_trajectory(0.0, 45.0), 12.0)},
    {"approved, zero velocity before the end line", answer(true), 25.0, 5.0,
     with_zero_velocity_at_x(make_straight_trajectory(10.0, 45.0), 28.0)},
    {"approved, front at the end line, zero velocity behind ego", answer(true),
     geometric_stop_x(END_LINE_X_M), 1.0,
     with_zero_velocity_at_x(make_straight_trajectory(10.0, 45.0), 15.0)},
    {"approved, stopped on an all-zero candidate", answer(true), 10.0, 0.0,
     with_velocity(make_straight_trajectory(10.0, 10.002), 0.0F)},
  };

  const auto scene = make_default_scene();
  for (const auto & row : zero_stop_cases) {
    SCOPED_TRACE(row.name);
    reset_plugin();
    auto input = make_input(scene, row.ego_x, row.ego_velocity_mps, row.states);
    auto trajectory = row.trajectory;
    EXPECT_FALSE(process_candidate(trajectory, input));
    expect_same_trajectory(trajectory, row.trajectory);
    EXPECT_TRUE(plugin().get_planning_factors().empty());
  }
  {
    SCOPED_TRACE("zero velocity at the end of a short candidate still gets the end-line stop");
    reset_plugin();
    auto input = make_input(scene, 36.0, 0.0, answer(true));
    auto trajectory = make_straight_trajectory(36.0, 36.5);
    trajectory.back().longitudinal_velocity_mps = 0.0F;
    ASSERT_TRUE(process_candidate(trajectory, input));
    EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);
    EXPECT_LE(trajectory.back().pose.position.x, geometric_stop_x(END_LINE_X_M) + 0.3);
    expect_single_stop_factor("VTL 12345: WAITING_FINALIZATION -> END_LINE");
  }

  struct TwoVtlCase
  {
    std::string name;
    double first_stop_x;
    bool first_approval;
    double second_stop_x;
    bool second_approval;
    double ego_x;
    double ego_velocity_mps;
  };
  const std::vector<TwoVtlCase> two_vtl_cases{
    {"both denied, nearer line first in map order", 20.0, false, 25.0, false, 10.0, 1.0},
    {"both denied, farther line first in map order", 25.0, false, 20.0, false, 10.0, 1.0},
    {"nearer denied, farther approved, stopped at the nearer line", 20.0, false, 25.0, true,
     geometric_stop_x(20.0), 0.0},
  };
  for (const auto & row : two_vtl_cases) {
    SCOPED_TRACE(row.name);
    reset_plugin();
    const auto two_vtl_scene = make_two_vtl_scene(row.first_stop_x, row.second_stop_x);
    auto input = make_input(
      two_vtl_scene, row.ego_x, row.ego_velocity_mps,
      make_two_vtl_states(row.first_approval, row.second_approval));
    auto trajectory = make_straight_trajectory(row.ego_x, 55.0);
    ASSERT_TRUE(process_candidate(trajectory, input));
    expect_truncated_stop_at_x(trajectory, geometric_stop_x(20.0), 0.1);
    if (row.ego_velocity_mps > 0.0) {
      EXPECT_GT(trajectory.front().longitudinal_velocity_mps, 0.0F);
    }
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, ActiveEndLineIsTheFirstDownstreamOnTheRoute)
{
  struct Case
  {
    std::string name;
    std::vector<std::tuple<lanelet::Id, double, double>> end_lines;
    double ego_x;
    double ego_velocity_mps;
    TrajectoryPoints trajectory;
    ModuleState expected_state;
    std::vector<std::string> expected_texts;
    std::optional<double> expected_end_line_marker_x;
  };
  const std::vector<Case> cases{
    {"first downstream line, whatever the map order",
     {{500, 50.0, 0.0}, {600, 40.0, 0.0}, {400, 40.0, 0.0}},
     36.0,
     0.0,
     make_straight_trajectory(36.0, 36.5),
     ModuleState::FINALIZING,
     {"ACTIVE_END_LINE_ID: 400"},
     40.0},
    {"line on another branch is ignored",
     {{300, 30.0, 20.0}, {400, 40.0, 0.0}},
     36.0,
     0.0,
     make_straight_trajectory(36.0, 36.5),
     ModuleState::FINALIZING,
     {"ACTIVE_END_LINE_ID: 400"},
     40.0},
    {"no end line after the stop line: keep requesting and stop",
     {{400, 15.0, 0.0}},
     10.0,
     5.0,
     make_straight_trajectory(0.0, 45.0),
     ModuleState::REQUESTING,
     {"ACTIVE_END_LINE_ID: INVALID", "reason=INVALID_END_LINE"},
     std::nullopt},
  };

  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    reset_plugin();
    const auto scene =
      make_scene(make_vtl_map(INSTRUMENT_ID, START_LINE_X_M, STOP_LINE_X_M, row.end_lines));
    auto input = make_input(scene, row.ego_x, row.ego_velocity_mps, answer(true));
    auto trajectory = row.trajectory;
    const auto result = process_batch(trajectory, input);
    ASSERT_TRUE(result.modified);
    EXPECT_EQ(result.state, row.expected_state);
    EXPECT_FLOAT_EQ(trajectory.back().longitudinal_velocity_mps, 0.0F);

    const auto previous_text_count = texts().size();
    const auto previous_marker_count = markers().size();
    plugin().publish_debug_data("trajectory_0");
    spin_until([&]() {
      return texts().size() > previous_text_count && markers().size() > previous_marker_count;
    });
    ASSERT_GT(texts().size(), previous_text_count);
    for (const auto & expected : row.expected_texts) {
      EXPECT_TRUE(contains(texts().back().data, expected)) << expected;
    }
    if (row.expected_end_line_marker_x) {
      ASSERT_GT(markers().size(), previous_marker_count);
      const auto & end_line_markers = markers().back().markers;
      const auto marker_it = std::find_if(
        end_line_markers.begin(), end_line_markers.end(),
        [](const auto & marker) { return marker.text == "active_end_line[400]"; });
      ASSERT_NE(marker_it, end_line_markers.end());
      EXPECT_DOUBLE_EQ(marker_it->pose.position.x, *row.expected_end_line_marker_x);
    }
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, EgoIsLocatedOnDenseStandstillCandidates)
{
  // Every candidate crosses the stop line, so any accepted ego association inserts a stop.
  struct Case
  {
    std::string name;
    double ego_yaw_rad;
    double candidate_start_x;
    bool expect_stop;
  };
  constexpr double ego_x = 10.0;
  const std::vector<Case> cases{
    {"candidate starts at ego", 0.0, ego_x, true},
    {"ego faces away from the candidate", M_PI, ego_x, false},
    {"candidate starts 6 m ahead of ego", 0.0, ego_x + 6.0, false},
  };

  const auto scene = make_default_scene();
  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    reset_plugin();
    auto input = make_vtl_input(
      make_odometry(ego_x, 0.0, row.ego_yaw_rad, 0.0, now()), scene.map, scene.route,
      answer(false));
    const auto original = make_standstill_start_trajectory(row.candidate_start_x, 0.0);
    auto trajectory = original;
    EXPECT_EQ(process_candidate(trajectory, input), row.expect_stop);
    if (row.expect_stop) {
      expect_single_stop_factor("VTL 12345: NO_RIGHT_OF_WAY -> STOP_LINE");
      expect_truncated_stop_at_x(trajectory, geometric_stop_x(STOP_LINE_X_M));
    } else {
      expect_same_trajectory(trajectory, original);
    }
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, UnstoppableStopLineIsPassedOnlyWhenOptedIn)
{
  struct Case
  {
    std::string name;
    double allow_if_cannot_stop_distance_m;
    double front_to_stop_line_m;
    double ego_velocity_mps;
    bool expect_pass;
  };
  const std::vector<Case> cases{
    {"0.99 m before the line at 2 m/s", 1.0, 0.99, 2.0, true},
    {"1.01 m before the line at 2 m/s", 1.0, 1.01, 2.0, false},
    {"0.99 m before the line at 0.5 m/s, can stop", 1.0, 0.99, 0.5, false},
    {"0.5 m past the line at 2 m/s", 1.0, -0.5, 2.0, false},
    {"rule off (0.0, the default), 0.99 m before the line at 2 m/s", 0.0, 0.99, 2.0, false},
  };

  const auto scene = make_default_scene();
  for (const auto & row : cases) {
    SCOPED_TRACE(row.name);
    params().virtual_traffic_light_stop.allow_if_cannot_stop_distance =
      row.allow_if_cannot_stop_distance_m;
    reset_plugin();
    const double ego_x = geometric_stop_x(STOP_LINE_X_M) - row.front_to_stop_line_m;
    auto input = make_input(scene, ego_x, row.ego_velocity_mps, answer(false));
    const auto original = make_straight_trajectory(ego_x, 40.0);
    auto trajectory = original;
    EXPECT_EQ(process_candidate(trajectory, input), !row.expect_pass);
    if (row.expect_pass) {
      expect_same_trajectory(trajectory, original);
    } else {
      expect_truncated_stop_at_x(
        trajectory, moving_stop_x(STOP_LINE_X_M, ego_x, row.ego_velocity_mps), 0.1);
    }
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, PublishesDebugMarkersTextAndTimingPerCandidate)
{
  const auto scene = make_default_scene();
  auto input = make_input(scene, 10.0, 5.0, nullptr);
  auto trajectory = make_straight_trajectory(0.0, 45.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_TRUE(process_candidate(trajectory, input));
  plugin().publish_debug_data("trajectory_0");
  plugin().publish_debug_data("trajectory_1");
  spin_until(
    [this]() { return markers().size() >= 2U && texts().size() >= 2U && timings().size() >= 2U; });

  ASSERT_GE(markers().size(), 2U);
  const auto & first_markers = markers().at(markers().size() - 2).markers;
  std::set<std::pair<std::string, int32_t>> marker_keys;
  std::set<std::string> first_namespaces;
  bool has_start = false;
  bool has_stop = false;
  bool has_instrument = false;
  bool has_end = false;
  bool has_start_label = false;
  bool has_stop_label = false;
  bool has_instrument_label = false;
  bool has_end_label = false;
  for (const auto & marker : first_markers) {
    EXPECT_EQ(marker.header.frame_id, "map");
    EXPECT_TRUE(starts_with(marker.ns, "trajectory_0/vtl_")) << marker.ns;
    EXPECT_TRUE(marker_keys.emplace(marker.ns, marker.id).second);
    first_namespaces.insert(marker.ns);
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
    } else if (contains(marker.ns, "/active_end_line/line")) {
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

  std::set<std::string> second_namespaces;
  for (const auto & marker : markers().back().markers) {
    EXPECT_TRUE(starts_with(marker.ns, "trajectory_1/vtl_")) << marker.ns;
    second_namespaces.insert(marker.ns);
  }
  std::vector<std::string> overlap;
  std::set_intersection(
    first_namespaces.begin(), first_namespaces.end(), second_namespaces.begin(),
    second_namespaces.end(), std::back_inserter(overlap));
  EXPECT_TRUE(overlap.empty());

  ASSERT_GE(texts().size(), 2U);
  const auto & text = texts().at(texts().size() - 2).data;
  for (const auto & expected :
       {"CANDIDATE: trajectory_0", "VTL[12345]",
        "LANELET_ID:", "REGULATORY_ELEMENT_ID:", "STATE: REQUESTING", "V2X: received=false",
        "DECISION: STOP target=STOP_LINE reason=NO_STATE modified=true",
        "ACTIVE_END_LINE_ID:", "DISTANCE_PATH_M:", "DISTANCE_ROUTE_M:", "ego_in_lane=true",
        "stop_line_relevant=true", "end_hold_active=false"}) {
    EXPECT_TRUE(contains(text, expected)) << expected;
  }

  std::set<std::string> timing_names;
  for (const auto & tree : timings()) {
    for (const auto & timing_node : tree.nodes) {
      timing_names.insert(timing_node.name);
    }
  }
  for (const auto & expected :
       {"VirtualTrafficLightStop::process", "VirtualTrafficLightStop::prepare_cycle",
        "VirtualTrafficLightStop::rebuild_modules", "VirtualTrafficLightStop::update_module_states",
        "VirtualTrafficLightStop::update_module_lifecycle",
        "VirtualTrafficLightStop::modify_trajectory", "VirtualTrafficLightStop::process_trajectory",
        "VirtualTrafficLightStop::process_module", "VirtualTrafficLightStop::insert_stop_velocity",
        "VirtualTrafficLightStop::publish_debug_data",
        "VirtualTrafficLightStop::publish_infrastructure_commands"}) {
    EXPECT_NE(timing_names.find(expected), timing_names.end()) << expected;
  }
}

TEST_F(VirtualTrafficLightStopIntegrationTest, PublishesOneCommandPerBatchAfterTheLastCandidate)
{
  const auto scene = make_default_scene();
  {
    SCOPED_TRACE("two stopping candidates");
    auto input = make_input(scene, 10.0, 1.0, answer(false));
    input.candidate_count = 2U;
    auto first = make_straight_trajectory(10.0, 30.0);
    EXPECT_TRUE(process_candidate(first, input));
    executor().spin_some();
    EXPECT_TRUE(commands().empty());
    auto second = make_straight_trajectory(10.0, 30.0);
    EXPECT_TRUE(process_candidate(second, input));
    EXPECT_FLOAT_EQ(first.back().longitudinal_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(second.back().longitudinal_velocity_mps, 0.0F);
    spin_until([this]() { return !commands().empty(); });
    ASSERT_EQ(commands().size(), 1U);
    ASSERT_EQ(commands().front().commands.size(), 1U);
  }
  {
    SCOPED_TRACE("the last candidate is unusable");
    const auto previous_command_count = commands().size();
    auto input = make_input(scene, 10.0, 1.0, answer(false));
    input.candidate_count = 2U;
    auto moving = make_straight_trajectory(10.0, 30.0);
    ASSERT_TRUE(process_candidate(moving, input));
    executor().spin_some();
    EXPECT_EQ(commands().size(), previous_command_count);

    TrajectoryPoints short_candidate(1);
    EXPECT_FALSE(process_candidate(short_candidate, input));
    spin_until([&]() { return commands().size() > previous_command_count; });
    ASSERT_EQ(commands().size(), previous_command_count + 1U);
    ASSERT_EQ(commands().back().commands.size(), 1U);
    EXPECT_EQ(
      static_cast<ModuleState>(commands().back().commands.front().state), ModuleState::REQUESTING);
  }
  {
    SCOPED_TRACE("candidate order changes neither the command nor the outputs");
    const auto run_cycle = [&](const bool reverse_order) {
      auto input = make_input(scene, 10.0, 5.0, answer(false));
      auto candidate_a = make_straight_trajectory(0.0, 45.0);
      auto candidate_b = make_straight_trajectory(2.0, 50.0);
      input.candidate_count = 2U;
      const auto previous_command_count = commands().size();
      if (reverse_order) {
        EXPECT_TRUE(process_candidate(candidate_b, input));
        EXPECT_TRUE(process_candidate(candidate_a, input));
      } else {
        EXPECT_TRUE(process_candidate(candidate_a, input));
        EXPECT_TRUE(process_candidate(candidate_b, input));
      }
      spin_until([&]() { return commands().size() > previous_command_count; });
      EXPECT_EQ(commands().size(), previous_command_count + 1U);
      return std::pair{candidate_a, candidate_b};
    };

    const auto first = run_cycle(false);
    ASSERT_FALSE(commands().empty());
    const auto first_command = commands().back();
    const auto second = run_cycle(true);
    const auto second_command = commands().back();

    expect_same_trajectory(first.first, second.first);
    expect_same_trajectory(first.second, second.second);
    ASSERT_EQ(first_command.commands.size(), 1U);
    ASSERT_EQ(second_command.commands.size(), 1U);
    EXPECT_EQ(first_command.commands.front().id, second_command.commands.front().id);
    EXPECT_EQ(first_command.commands.front().state, second_command.commands.front().state);
    EXPECT_EQ(
      first_command.commands.front().custom_tags, second_command.commands.front().custom_tags);
  }
}

TEST_F(
  VirtualTrafficLightStopIntegrationTest, NodeSkipsEmptyBatchAndPublishesOnceForUnchangedCandidates)
{
  using CandidateTrajectories = autoware_internal_planning_msgs::msg::CandidateTrajectories;
  rclcpp::NodeOptions options;
  const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
  autoware::test_utils::updateNodeOptions(
    options, {test_utils_dir + "/config/test_vehicle_info.param.yaml"});
  options.append_parameter_override(
    "plugin_names",
    std::vector<std::string>{"autoware::trajectory_modifier::plugin::VirtualTrafficLightStop"});
  options.append_parameter_override("use_virtual_traffic_light_stop", true);
  auto modifier = std::make_shared<autoware::trajectory_modifier::TrajectoryModifier>(options);
  executor().add_node(modifier);
  size_t command_count = 0;
  auto command_subscription = node().create_subscription<InfrastructureCommandArray>(
    "/trajectory_modifier/output/infrastructure_commands", 10,
    [&command_count](const InfrastructureCommandArray::ConstSharedPtr) { ++command_count; });
  size_t output_count = 0;
  auto outputs = node().create_subscription<CandidateTrajectories>(
    "/trajectory_modifier/output/trajectories", 10,
    [&output_count](const CandidateTrajectories::ConstSharedPtr) { ++output_count; });
  auto odometry = node().create_publisher<Odometry>("/trajectory_modifier/input/odometry", 1);
  auto acceleration = node().create_publisher<AccelWithCovarianceStamped>(
    "/trajectory_modifier/input/acceleration", 1);
  auto candidates =
    node().create_publisher<CandidateTrajectories>("/trajectory_modifier/input/trajectories", 1);
  spin_until([&]() {
    return candidates->get_subscription_count() > 0 && odometry->get_subscription_count() > 0 &&
           acceleration->get_subscription_count() > 0 &&
           command_subscription->get_publisher_count() > 0 && outputs->get_publisher_count() > 0;
  });
  ASSERT_GT(command_subscription->get_publisher_count(), 0U);
  ASSERT_GT(outputs->get_publisher_count(), 0U);
  for (size_t cycle = 0; cycle < 2; ++cycle) {
    odometry->publish(*make_odometry(0.0, 1.0, now()));
    acceleration->publish(AccelWithCovarianceStamped{});
    for (size_t i = 0; i < 5; ++i) {
      executor().spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    CandidateTrajectories batch;
    if (cycle == 1) {
      batch.candidate_trajectories.resize(2);
      for (auto & candidate : batch.candidate_trajectories) {
        candidate.points = make_straight_trajectory(0.0, 30.0);
      }
    }
    candidates->publish(batch);
    spin_until([&]() { return output_count >= cycle + 1U && command_count >= cycle; });
    EXPECT_EQ(output_count, cycle + 1U);
    EXPECT_EQ(command_count, cycle);
  }
  executor().remove_node(modifier);
}

TEST_F(VirtualTrafficLightStopIntegrationTest, NodeWithoutVtlPluginDoesNotSubscribeToVtlStates)
{
  rclcpp::NodeOptions options;
  const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
  autoware::test_utils::updateNodeOptions(
    options, {test_utils_dir + "/config/test_vehicle_info.param.yaml"});
  options.append_parameter_override(
    "plugin_names",
    std::vector<std::string>{"autoware::trajectory_modifier::plugin::ExternalVelocityLimit"});
  auto modifier = std::make_shared<autoware::trajectory_modifier::TrajectoryModifier>(options);
  auto states = node().create_publisher<VirtualTrafficLightStateArray>(
    "/trajectory_modifier/input/virtual_traffic_light_states", rclcpp::QoS{1});
  auto velocity_limit =
    node().create_publisher<autoware_internal_planning_msgs::msg::VelocityLimit>(
      "/trajectory_modifier/input/external_velocity_limit_mps", rclcpp::QoS{1});

  // Wait for discovery of the configured plugin before inspecting the node's subscriptions.
  spin_until([&]() { return velocity_limit->get_subscription_count() > 0U; });
  ASSERT_GT(velocity_limit->get_subscription_count(), 0U);
  EXPECT_EQ(states->get_subscription_count(), 0U);
}
