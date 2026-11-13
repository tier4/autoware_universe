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

#include "autoware/predicted_trajectory_validator/hazard_warning/alert_latch.hpp"
#include "autoware/predicted_trajectory_validator/hazard_warning/hazard_warning.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::predicted_trajectory_validator
{
namespace
{
using autoware_internal_control_msgs::msg::HazardAlert;
using autoware_internal_control_msgs::msg::HazardSector;
using autoware_utils_geometry::Point2d;
using hazard_warning::AlertLatch;
using hazard_warning::kNumSectors;
using hazard_warning::SectorState;

constexpr size_t kFront = 0;
constexpr size_t kFrontRight = 1;
constexpr size_t kRear = 4;
constexpr size_t kLeft = 6;
constexpr size_t kFrontLeft = 7;

rclcpp::Time at(const double sec)
{
  return rclcpp::Time(static_cast<int64_t>(sec * 1e9), RCL_ROS_TIME);
}

vehicle_info_utils::VehicleInfo test_vehicle_info(const double front_overhang = 1.0)
{
  return vehicle_info_utils::createVehicleInfo(
    0.383, 0.235, 2.79, 1.64, front_overhang, 1.1, 0.128, 0.128, 2.5, 0.70);
}

::predicted_trajectory_validator::Params default_params()
{
  ::predicted_trajectory_validator::Params params;
  auto & p = params.hazard_warning;
  p.min_alert_duration = 0.5;
  return params;
}

HazardWarning make_warning(
  const ::predicted_trajectory_validator::Params & params = default_params())
{
  return HazardWarning(rclcpp::get_logger("test_hazard_warning"), params);
}

HazardSource make_box(
  const double x_min, const double x_max, const double y_min, const double y_max,
  const uint8_t risk_level, const uint8_t sources = HazardSector::SOURCE_COLLISION)
{
  HazardSource source;
  source.shape.outer() = {
    {x_min, y_min}, {x_min, y_max}, {x_max, y_max}, {x_max, y_min}, {x_min, y_min}};
  boost::geometry::correct(source.shape);
  source.risk_level = risk_level;
  source.sources = sources;
  return source;
}

HazardSource make_point(
  const double x, const double y, const uint8_t risk_level,
  const uint8_t sources = HazardSector::SOURCE_BOUNDARY_DEPARTURE)
{
  HazardSource source;
  source.shape.outer().emplace_back(x, y);
  source.risk_level = risk_level;
  source.sources = sources;
  return source;
}

HazardWarningSources collision_only(const std::vector<HazardSource> & sources)
{
  HazardWarningSources hazard_sources;
  hazard_sources.collision = sources;
  return hazard_sources;
}

HazardWarningSources departure_only(const std::vector<HazardSource> & sources)
{
  HazardWarningSources hazard_sources;
  hazard_sources.departure = sources;
  return hazard_sources;
}

std::vector<size_t> raised_sectors(const HazardAlert & alert)
{
  std::vector<size_t> raised;
  for (size_t s = 0; s < kNumSectors; ++s) {
    if (SectorState{alert.sectors[s].risk_level, alert.sectors[s].sources}.is_on()) {
      raised.push_back(s);
    }
  }
  return raised;
}
}  // namespace

TEST(AlertLatchTest, RisesImmediatelyHoldsForMinDurationAndResetsOnTimeReversal)
{
  AlertLatch latch;
  const double t_min = 0.5;
  const SectorState off{};
  const SectorState low{HazardSector::LOW, HazardSector::SOURCE_COLLISION};
  const SectorState mid{HazardSector::MID, HazardSector::SOURCE_BOUNDARY_DEPARTURE};
  const SectorState proximity{HazardSector::NONE, HazardSector::SOURCE_PROXIMITY};
  const auto expect_state = [](const SectorState & actual, const SectorState & expected) {
    EXPECT_EQ(actual.risk_level, expected.risk_level);
    EXPECT_EQ(actual.sources, expected.sources);
  };

  expect_state(latch.update(off, at(0.0), t_min), off);
  expect_state(latch.update(low, at(0.1), t_min), low);
  expect_state(latch.update(off, at(0.3), t_min), low);
  expect_state(latch.update(off, at(0.6), t_min), low);
  expect_state(latch.update(off, at(0.61), t_min), off);
  expect_state(latch.update(off, at(0.7), t_min), off);

  // Re-raised within the hold refreshes the hold start.
  expect_state(latch.update(mid, at(1.0), t_min), mid);
  expect_state(latch.update(proximity, at(1.3), t_min), proximity);
  expect_state(latch.update(off, at(1.7), t_min), proximity);
  expect_state(latch.update(off, at(1.9), t_min), off);

  // Time going backwards clears the latch.
  expect_state(latch.update(low, at(5.0), t_min), low);
  expect_state(latch.update(off, at(4.9), t_min), off);
}

TEST(HazardWarningTest, CollisionSourceAheadRaisesFrontWithItsRiskLevel)
{
  auto warning = make_warning();

  const auto alert = warning.make_alert(
    collision_only({make_box(10.0, 11.0, -0.5, 0.5, HazardSector::LOW)}), test_vehicle_info(),
    at(1.0));

  ASSERT_TRUE(alert.has_value());
  EXPECT_EQ(rclcpp::Time(alert->header.stamp), at(1.0));
  EXPECT_EQ(alert->header.frame_id, "base_link");
  EXPECT_EQ(raised_sectors(*alert), std::vector<size_t>{kFront});
  EXPECT_EQ(alert->sectors[kFront].risk_level, HazardSector::LOW);
  EXPECT_EQ(alert->sectors[kFront].sources, HazardSector::SOURCE_COLLISION);
  EXPECT_EQ(alert->sectors[kRear].risk_level, HazardSector::NONE);
  EXPECT_EQ(alert->sectors[kRear].sources, 0);
}

TEST(HazardWarningTest, OriginAndSectorAnglesFollowTheSpec)
{
  auto warning = make_warning();
  const auto alert = warning.make_alert(collision_only({}), test_vehicle_info(), at(0.0));
  ASSERT_TRUE(alert.has_value());
  EXPECT_NEAR(alert->origin.x, 1.345, 1e-9);
  EXPECT_NEAR(alert->origin.y, 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(alert->origin.z, 0.0);

  const std::vector<std::array<double, 2>> expected{
    {-M_PI / 18.0, M_PI / 18.0},
    {-M_PI / 6.0, -M_PI / 18.0},
    {-5.0 * M_PI / 6.0, -M_PI / 6.0},
    {-17.0 * M_PI / 18.0, -5.0 * M_PI / 6.0},
    {17.0 * M_PI / 18.0, -17.0 * M_PI / 18.0},
    {5.0 * M_PI / 6.0, 17.0 * M_PI / 18.0},
    {M_PI / 6.0, 5.0 * M_PI / 6.0},
    {M_PI / 18.0, M_PI / 6.0},
  };
  ASSERT_EQ(alert->sectors.size(), expected.size());
  for (size_t s = 0; s < expected.size(); ++s) {
    EXPECT_NEAR(alert->sectors[s].start_angle, expected[s][0], 1e-12) << "sector " << s;
    EXPECT_NEAR(alert->sectors[s].end_angle, expected[s][1], 1e-12) << "sector " << s;
  }
}

TEST(HazardWarningTest, SourceWithoutLevelAndBitsDoesNotRaise)
{
  auto warning = make_warning();
  const auto alert = warning.make_alert(
    collision_only({make_box(10.0, 11.0, -0.5, 0.5, HazardSector::NONE, 0)}), test_vehicle_info(),
    at(0.0));
  ASSERT_TRUE(alert.has_value());
  EXPECT_TRUE(raised_sectors(*alert).empty());
  EXPECT_EQ(alert->sectors[kFront].risk_level, HazardSector::NONE);
}

TEST(HazardWarningTest, ProximityOnlyRaisesWithoutALevel)
{
  auto warning = make_warning();
  const auto alert = warning.make_alert(
    collision_only(
      {make_box(10.0, 11.0, -0.5, 0.5, HazardSector::NONE, HazardSector::SOURCE_PROXIMITY)}),
    test_vehicle_info(), at(0.0));
  ASSERT_TRUE(alert.has_value());
  EXPECT_EQ(raised_sectors(*alert), std::vector<size_t>{kFront});
  EXPECT_EQ(alert->sectors[kFront].risk_level, HazardSector::NONE);
  EXPECT_EQ(alert->sectors[kFront].sources, HazardSector::SOURCE_PROXIMITY);
}

TEST(HazardWarningTest, NoSourcesReturnsNulloptWithoutAdvancingTheLatch)
{
  auto warning = make_warning();
  const auto vehicle_info = test_vehicle_info();
  ASSERT_TRUE(
    warning
      .make_alert(
        collision_only({make_box(10.0, 11.0, -0.5, 0.5, HazardSector::LOW)}), vehicle_info, at(0.0))
      .has_value());

  EXPECT_FALSE(warning.make_alert(HazardWarningSources{}, vehicle_info, at(10.0)).has_value());

  const auto held = warning.make_alert(collision_only({}), vehicle_info, at(0.3));
  ASSERT_TRUE(held.has_value());
  EXPECT_EQ(held->sectors[kFront].risk_level, HazardSector::LOW);
}

TEST(HazardWarningTest, EitherSourceAloneReturnsAnAlert)
{
  const auto vehicle_info = test_vehicle_info();
  auto collision_warning = make_warning();
  const auto collision = collision_warning.make_alert(collision_only({}), vehicle_info, at(0.0));
  ASSERT_TRUE(collision.has_value());
  EXPECT_TRUE(raised_sectors(*collision).empty());

  auto departure_warning = make_warning();
  const auto departure = departure_warning.make_alert(
    departure_only({make_point(5.0, 3.0, HazardSector::MID)}), vehicle_info, at(0.0));
  ASSERT_TRUE(departure.has_value());
  EXPECT_FALSE(raised_sectors(*departure).empty());
}

TEST(HazardWarningTest, SectorLatchHoldsAndKeepsLastRiskLevel)
{
  auto warning = make_warning();
  const auto vehicle_info = test_vehicle_info();
  const auto raised = collision_only({make_box(10.0, 11.0, -0.5, 0.5, HazardSector::LOW)});
  const auto empty = collision_only({});

  warning.make_alert(raised, vehicle_info, at(0.0));
  const auto held = warning.make_alert(empty, vehicle_info, at(0.3));
  ASSERT_TRUE(held.has_value());
  EXPECT_EQ(held->sectors[kFront].risk_level, HazardSector::LOW);
  EXPECT_EQ(held->sectors[kFront].sources, HazardSector::SOURCE_COLLISION);

  const auto released = warning.make_alert(empty, vehicle_info, at(0.6));
  ASSERT_TRUE(released.has_value());
  EXPECT_EQ(released->sectors[kFront].risk_level, HazardSector::NONE);
  EXPECT_EQ(released->sectors[kFront].sources, 0);
}

TEST(HazardWarningTest, CollisionAndDepartureShareTheLatchOfASector)
{
  auto warning = make_warning();
  const auto vehicle_info = test_vehicle_info();
  HazardWarningSources both;
  both.collision = std::vector<HazardSource>{make_box(10.0, 11.0, -0.5, 0.5, HazardSector::HIGH)};
  both.departure = std::vector<HazardSource>{make_point(10.0, 0.5, HazardSector::MID)};
  const auto first = warning.make_alert(both, vehicle_info, at(0.0));
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->sectors[kFront].risk_level, HazardSector::HIGH);
  EXPECT_EQ(
    first->sectors[kFront].sources,
    HazardSector::SOURCE_COLLISION | HazardSector::SOURCE_BOUNDARY_DEPARTURE);

  HazardWarningSources departure_continues;
  departure_continues.collision = std::vector<HazardSource>{};
  departure_continues.departure = both.departure;
  const auto second = warning.make_alert(departure_continues, vehicle_info, at(0.1));
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->sectors[kFront].risk_level, HazardSector::MID);
  EXPECT_EQ(second->sectors[kFront].sources, HazardSector::SOURCE_BOUNDARY_DEPARTURE);

  HazardWarningSources proximity_and_departure;
  proximity_and_departure.collision = std::vector<HazardSource>{
    make_box(10.0, 11.0, -0.5, 0.5, HazardSector::NONE, HazardSector::SOURCE_PROXIMITY)};
  proximity_and_departure.departure = both.departure;
  const auto third = warning.make_alert(proximity_and_departure, vehicle_info, at(0.2));
  ASSERT_TRUE(third.has_value());
  EXPECT_EQ(third->sectors[kFront].risk_level, HazardSector::MID);
  EXPECT_EQ(
    third->sectors[kFront].sources,
    HazardSector::SOURCE_PROXIMITY | HazardSector::SOURCE_BOUNDARY_DEPARTURE);
}

TEST(HazardWarningTest, PolygonStraddlingWedgesRaisesEverySectorItTouches)
{
  auto warning = make_warning();
  // 6 m wide polygon 10 m ahead: bearing of its corners from the wedge apex is about +-19 deg,
  // so FRONT, FRONT_LEFT and FRONT_RIGHT are touched, LEFT / RIGHT (>= 29 deg) are not.
  const auto alert = warning.make_alert(
    collision_only({make_box(10.0, 11.0, -3.0, 3.0, HazardSector::LOW)}), test_vehicle_info(),
    at(0.0));
  ASSERT_TRUE(alert.has_value());
  EXPECT_EQ(raised_sectors(*alert), (std::vector<size_t>{kFront, kFrontRight, kFrontLeft}));
}

TEST(HazardWarningTest, PointRaisesOneSectorOrTwoWithinTheBoundaryTolerance)
{
  const auto vehicle_info = test_vehicle_info();
  const double apex_x = 1.345;

  auto warning = make_warning();
  const auto front = warning.make_alert(
    collision_only({make_point(apex_x + 10.0, 1.0, HazardSector::HIGH)}), vehicle_info, at(0.0));
  ASSERT_TRUE(front.has_value());
  EXPECT_EQ(raised_sectors(*front), std::vector<size_t>{kFront});
  EXPECT_EQ(front->sectors[kFront].risk_level, HazardSector::HIGH);

  auto warning2 = make_warning();
  const auto boundary = warning2.make_alert(
    collision_only(
      {make_point(apex_x + 10.0, 10.0 * std::tan(10.5 * M_PI / 180.0), HazardSector::MID)}),
    vehicle_info, at(0.0));
  ASSERT_TRUE(boundary.has_value());
  EXPECT_EQ(raised_sectors(*boundary), (std::vector<size_t>{kFront, kFrontLeft}));

  auto warning3 = make_warning();
  const auto left = warning3.make_alert(
    collision_only({make_point(apex_x, 20.0, HazardSector::LOW)}), vehicle_info, at(0.0));
  ASSERT_TRUE(left.has_value());
  EXPECT_EQ(raised_sectors(*left), std::vector<size_t>{kLeft});
}

TEST(HazardWarningTest, DistantSourceIsAssignedByItsBearing)
{
  auto warning = make_warning();
  const auto alert = warning.make_alert(
    collision_only({make_point(101.345, 0.0, HazardSector::LOW)}), test_vehicle_info(), at(0.0));
  ASSERT_TRUE(alert.has_value());
  EXPECT_EQ(raised_sectors(*alert), std::vector<size_t>{kFront});
}

TEST(HazardWarningTest, PolygonContainingTheCenterRaisesEverySector)
{
  auto warning = make_warning();
  const auto alert = warning.make_alert(
    collision_only({make_box(-5.0, 5.0, -5.0, 5.0, HazardSector::LOW)}), test_vehicle_info(),
    at(0.0));
  ASSERT_TRUE(alert.has_value());
  EXPECT_EQ(raised_sectors(*alert).size(), kNumSectors);
}

TEST(HazardWarningTest, PolygonSpanningSectorsRaisesTheSectorsInBetween)
{
  const double apex_x = 1.345;
  const auto at_bearing = [&](const double radius, const double deg) {
    const double rad = deg * M_PI / 180.0;
    return Point2d{apex_x + radius * std::cos(rad), radius * std::sin(rad)};
  };
  HazardSource band;
  band.shape.outer() = {
    at_bearing(20.0, 5.0), at_bearing(20.0, 95.0), at_bearing(21.0, 95.0), at_bearing(21.0, 5.0),
    at_bearing(20.0, 5.0)};
  boost::geometry::correct(band.shape);
  band.risk_level = HazardSector::MID;
  band.sources = HazardSector::SOURCE_COLLISION;

  auto warning = make_warning();
  const auto alert = warning.make_alert(collision_only({band}), test_vehicle_info(), at(0.0));
  ASSERT_TRUE(alert.has_value());
  EXPECT_EQ(raised_sectors(*alert), (std::vector<size_t>{kFront, kLeft, kFrontLeft}));
}

TEST(HazardWarningTest, WedgeApexFollowsTheVehicleInfo)
{
  const auto point = collision_only({make_point(11.345, 3.0, HazardSector::LOW)});

  auto warning = make_warning();
  const auto alert = warning.make_alert(point, test_vehicle_info(), at(0.0));
  ASSERT_TRUE(alert.has_value());
  EXPECT_EQ(raised_sectors(*alert), std::vector<size_t>{kFrontLeft});

  auto warning2 = make_warning();
  const auto long_front = warning2.make_alert(point, test_vehicle_info(17.0), at(0.0));
  ASSERT_TRUE(long_front.has_value());
  EXPECT_EQ(raised_sectors(*long_front), std::vector<size_t>{kLeft});
}

TEST(HazardWarningTest, WarningWithoutNodeGivesTheSameAlertAsWithNode)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("test_hazard_warning_node");
    HazardWarning with_node(
      node->get_logger(), default_params(),
      node->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/hazard/markers", 1));
    auto without_node = make_warning();
    HazardWarningSources sources;
    sources.collision =
      std::vector<HazardSource>{make_box(10.0, 11.0, -3.0, 3.0, HazardSector::MID)};
    sources.departure = std::vector<HazardSource>{make_point(1.345, 20.0, HazardSector::LOW)};

    for (const double t : {0.0, 0.3, 0.6}) {
      const auto a = with_node.make_alert(sources, test_vehicle_info(), at(t));
      const auto b = without_node.make_alert(sources, test_vehicle_info(), at(t));
      ASSERT_TRUE(a.has_value());
      ASSERT_TRUE(b.has_value());
      EXPECT_EQ(*a, *b);
      sources.collision = std::vector<HazardSource>{};
    }
  }
  rclcpp::shutdown();
}

TEST(HazardWarningTest, MarkersKeepTheLastRaisedSourcesWhileTheLatchHolds)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("test_hazard_markers_node");
    HazardWarning warning(
      node->get_logger(), default_params(),
      node->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/hazard/markers", 1));
    std::vector<visualization_msgs::msg::MarkerArray> received;
    auto sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/test_hazard_markers_node/debug/hazard/markers", 10,
      [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        received.push_back(*msg);
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto spin_until = [&](const size_t count) {
      for (int i = 0; i < 50 && received.size() < count; ++i) {
        executor.spin_some(std::chrono::milliseconds(20));
      }
    };
    for (int i = 0;
         i < 50 && node->count_subscribers("/test_hazard_markers_node/debug/hazard/markers") == 0;
         ++i) {
      executor.spin_some(std::chrono::milliseconds(20));
    }

    auto source = make_box(10.0, 11.0, -0.5, 0.5, HazardSector::MID);
    source.stamp = at(0.0);
    const auto markers_of =
      [](const visualization_msgs::msg::MarkerArray & msg, const std::string & ns) {
        std::vector<visualization_msgs::msg::Marker> out;
        for (const auto & marker : msg.markers) {
          if (marker.ns == ns) out.push_back(marker);
        }
        return out;
      };

    auto departure = make_point(10.0, 0.5, HazardSector::HIGH);
    departure.stamp = at(0.0);
    HazardWarningSources raised;
    raised.collision = std::vector<HazardSource>{source};
    raised.departure = std::vector<HazardSource>{departure};
    warning.make_alert(raised, test_vehicle_info(), at(0.0));
    spin_until(1);
    warning.make_alert(collision_only({}), test_vehicle_info(), at(0.3));
    spin_until(2);
    warning.make_alert(collision_only({}), test_vehicle_info(), at(0.6));
    spin_until(3);
    ASSERT_EQ(received.size(), 3U);

    for (size_t i = 0; i < 2; ++i) {
      const auto sources = markers_of(received[i], "collision_sources");
      ASSERT_EQ(sources.size(), 1U) << "message " << i;
      EXPECT_EQ(rclcpp::Time(sources.front().header.stamp), at(0.0));
      EXPECT_FALSE(sources.front().frame_locked);
      EXPECT_EQ(markers_of(received[i], "collision_risk_level").size(), 1U) << "message " << i;
      const auto departures = markers_of(received[i], "departure_sources");
      ASSERT_EQ(departures.size(), 1U) << "message " << i;
      EXPECT_EQ(departures.front().type, visualization_msgs::msg::Marker::SPHERE);
      EXPECT_EQ(markers_of(received[i], "departure_risk_level").size(), 1U) << "message " << i;
      const auto sectors = markers_of(received[i], "hazard_sectors");
      ASSERT_EQ(sectors.size(), 1U) << "message " << i;
      EXPECT_EQ(rclcpp::Time(sectors.front().header.stamp), at(i == 0 ? 0.0 : 0.3));
      EXPECT_TRUE(sectors.front().frame_locked);
    }
    EXPECT_TRUE(markers_of(received[2], "collision_sources").empty());
    EXPECT_TRUE(markers_of(received[2], "departure_sources").empty());
    EXPECT_TRUE(markers_of(received[2], "hazard_sectors").empty());
  }
  rclcpp::shutdown();
}
}  // namespace autoware::predicted_trajectory_validator
