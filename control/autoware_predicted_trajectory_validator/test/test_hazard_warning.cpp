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
#include "test_utils.hpp"

#include <boost/geometry.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace autoware::predicted_trajectory_validator
{
using namespace test_utils;  // NOLINT(build/namespaces)

namespace
{
using autoware_internal_control_msgs::msg::HazardAlert;
using autoware_internal_control_msgs::msg::HazardSector;
using hazard_warning::AlertLatch;
using hazard_warning::kNumSectors;
using hazard_warning::SectorState;

constexpr size_t kFront = 0;
constexpr size_t kFrontRight = 1;
constexpr size_t kRear = 4;
constexpr size_t kLeft = 6;
constexpr size_t kFrontLeft = 7;
constexpr double kApexX = 1.345;

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

HazardSource band_from_5_to_95_deg()
{
  const auto at_bearing = [&](const double radius, const double deg) {
    const double rad = deg * M_PI / 180.0;
    return Point2d{kApexX + radius * std::cos(rad), radius * std::sin(rad)};
  };
  HazardSource band;
  band.shape.outer() = {
    at_bearing(20.0, 5.0), at_bearing(20.0, 95.0), at_bearing(21.0, 95.0), at_bearing(21.0, 5.0),
    at_bearing(20.0, 5.0)};
  boost::geometry::correct(band.shape);
  band.risk_level = HazardSector::MID;
  band.sources = HazardSector::SOURCE_COLLISION;
  return band;
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
  EXPECT_NEAR(alert->origin.x, kApexX, 1e-9);
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

TEST(HazardWarningTest, SectorLatchHoldsTheLastRiskLevelAndIgnoresCyclesWithoutSources)
{
  auto warning = make_warning();
  const auto vehicle_info = test_vehicle_info();
  const auto raised = collision_only({make_box(10.0, 11.0, -0.5, 0.5, HazardSector::LOW)});
  const auto empty = collision_only({});

  ASSERT_TRUE(warning.make_alert(raised, vehicle_info, at(0.0)).has_value());
  EXPECT_FALSE(warning.make_alert(HazardWarningSources{}, vehicle_info, at(10.0)).has_value());

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

struct SectorCase
{
  std::string name;
  HazardSource source;
  std::vector<size_t> expected_sectors;
  std::optional<uint8_t> expected_front_level;
};

class SectorAssignmentTest : public ::testing::TestWithParam<SectorCase>
{
};

TEST_P(SectorAssignmentTest, SourceRaisesTheSectorsItTouches)
{
  auto warning = make_warning();
  const auto alert =
    warning.make_alert(collision_only({GetParam().source}), test_vehicle_info(), at(0.0));
  ASSERT_TRUE(alert.has_value());
  EXPECT_EQ(raised_sectors(*alert), GetParam().expected_sectors);
  if (GetParam().expected_front_level) {
    EXPECT_EQ(alert->sectors[kFront].risk_level, *GetParam().expected_front_level);
  }
}

INSTANTIATE_TEST_SUITE_P(
  HazardWarningTest, SectorAssignmentTest,
  ::testing::Values(
    // 6 m wide polygon 10 m ahead: bearing of its corners from the wedge apex is about +-19 deg,
    // so FRONT, FRONT_LEFT and FRONT_RIGHT are touched, LEFT / RIGHT (>= 29 deg) are not.
    SectorCase{
      "PolygonStraddlingWedges",
      make_box(10.0, 11.0, -3.0, 3.0, HazardSector::LOW),
      {kFront, kFrontRight, kFrontLeft},
      std::nullopt},
    SectorCase{
      "PointAhead",
      make_point(kApexX + 10.0, 1.0, HazardSector::HIGH),
      {kFront},
      HazardSector::HIGH},
    SectorCase{
      "PointWithinTheBoundaryTolerance",
      make_point(kApexX + 10.0, 10.0 * std::tan(10.5 * M_PI / 180.0), HazardSector::MID),
      {kFront, kFrontLeft},
      std::nullopt},
    SectorCase{
      "PointOnTheLeft", make_point(kApexX, 20.0, HazardSector::LOW), {kLeft}, std::nullopt},
    SectorCase{
      "DistantPointByItsBearing",
      make_point(101.345, 0.0, HazardSector::LOW),
      {kFront},
      std::nullopt},
    SectorCase{
      "PolygonContainingTheCenter",
      make_box(-5.0, 5.0, -5.0, 5.0, HazardSector::LOW),
      {0, 1, 2, 3, 4, 5, 6, 7},
      std::nullopt},
    SectorCase{
      "PolygonSpanningSectors",
      band_from_5_to_95_deg(),
      {kFront, kLeft, kFrontLeft},
      std::nullopt}),
  [](const ::testing::TestParamInfo<SectorCase> & info) { return info.param.name; });

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
}  // namespace autoware::predicted_trajectory_validator
