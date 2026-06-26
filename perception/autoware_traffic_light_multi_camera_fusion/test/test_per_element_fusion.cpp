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

#include "../src/multi_camera_fusion.hpp"
#include "../src/per_element_fusion.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
namespace per_element = autoware::traffic_light::per_element;
using autoware::traffic_light::ConflictType;
using autoware::traffic_light::FusionMode;
using autoware::traffic_light::MultiCameraFusion;
using autoware::traffic_light::MultiCameraFusionConfig;
using autoware_perception_msgs::msg::TrafficLightElement;
using sensor_msgs::msg::CameraInfo;
using T4Element = tier4_perception_msgs::msg::TrafficLightElement;
using T4Signal = tier4_perception_msgs::msg::TrafficLight;

// Two physical traffic lights bound to one regulatory element — the typical Japanese left/right
// arrangement at an intersection. Group fusion aggregates the two into a single output entry.
constexpr lanelet::Id LEFT_TL_ID = 11;
constexpr lanelet::Id RIGHT_TL_ID = 12;
constexpr lanelet::Id REGULATORY_ELEMENT_ID = 100;

constexpr uint32_t IMG_WIDTH = 1440;
constexpr uint32_t IMG_HEIGHT = 1080;
constexpr uint32_t ROI_OFFSET = 100;
constexpr uint32_t ROI_SIZE = 100;

lanelet::LaneletMapPtr make_lanelet_map()
{
  lanelet::Point3d road_l_start(1, 0.0, 2.0, 0.0);
  lanelet::Point3d road_l_end(2, 30.0, 2.0, 0.0);
  lanelet::Point3d road_r_start(3, 0.0, -2.0, 0.0);
  lanelet::Point3d road_r_end(4, 30.0, -2.0, 0.0);
  lanelet::LineString3d road_l(5, {road_l_start, road_l_end});
  lanelet::LineString3d road_r(6, {road_r_start, road_r_end});

  auto road_lanelet = lanelet::Lanelet(1000, road_l, road_r);
  road_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

  lanelet::Point3d ltl_l(7, 19.5, 0.5, 3.5);
  lanelet::Point3d ltl_r(8, 19.5, -0.5, 3.5);
  lanelet::LineString3d left_tl(LEFT_TL_ID, {ltl_l, ltl_r});
  left_tl.attributes()["subtype"] = "red_yellow_green";
  left_tl.attributes()["height"] = "1.0";

  lanelet::Point3d rtl_l(9, 20.5, 0.5, 3.5);
  lanelet::Point3d rtl_r(10, 20.5, -0.5, 3.5);
  lanelet::LineString3d right_tl(RIGHT_TL_ID, {rtl_l, rtl_r});
  right_tl.attributes()["subtype"] = "red_yellow_green";
  right_tl.attributes()["height"] = "1.0";

  auto reg = lanelet::autoware::AutowareTrafficLight::make(
    REGULATORY_ELEMENT_ID, lanelet::AttributeMap(), {left_tl, right_tl});
  road_lanelet.addRegulatoryElement(reg);

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(road_lanelet);
  return map;
}

MultiCameraFusionConfig make_per_element_config()
{
  MultiCameraFusionConfig cfg;
  cfg.message_lifespan = 1.0;
  cfg.prior_log_odds = 0.0;
  cfg.lanelet_map_ptr = make_lanelet_map();
  cfg.fusion_mode = FusionMode::PerElement;
  cfg.per_element_config.prior_log_odds = 0.0;
  cfg.per_element_config.on_threshold = 0.0;
  cfg.per_element_config.confidence_gate = 0.0;
  cfg.per_element_config.strict_mode = false;
  cfg.per_element_config.rules = per_element::default_japan_rules();
  return cfg;
}

CameraInfo make_cam_info(const rclcpp::Time & stamp, const std::string & frame_id)
{
  CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = frame_id;
  info.width = IMG_WIDTH;
  info.height = IMG_HEIGHT;
  return info;
}

tier4_perception_msgs::msg::TrafficLightRoi make_roi(lanelet::Id tl_id)
{
  tier4_perception_msgs::msg::TrafficLightRoi roi;
  roi.traffic_light_id = tl_id;
  roi.roi.x_offset = ROI_OFFSET;
  roi.roi.y_offset = ROI_OFFSET;
  roi.roi.width = ROI_SIZE;
  roi.roi.height = ROI_SIZE;
  return roi;
}

tier4_perception_msgs::msg::TrafficLightRoiArray make_roi_array(
  const rclcpp::Time & stamp, const std::string & frame_id, lanelet::Id tl_id)
{
  tier4_perception_msgs::msg::TrafficLightRoiArray arr;
  arr.header.stamp = stamp;
  arr.header.frame_id = frame_id;
  arr.rois.push_back(make_roi(tl_id));
  return arr;
}

T4Element make_element(uint8_t color, uint8_t shape, float confidence)
{
  T4Element e;
  e.color = color;
  e.shape = shape;
  e.status = T4Element::SOLID_ON;
  e.confidence = confidence;
  return e;
}

T4Signal make_signal(lanelet::Id tl_id, std::vector<T4Element> elements)
{
  T4Signal sig;
  sig.traffic_light_id = tl_id;
  sig.elements = std::move(elements);
  return sig;
}

tier4_perception_msgs::msg::TrafficLightArray make_signal_array(
  const rclcpp::Time & stamp, const std::string & frame_id, const T4Signal & sig)
{
  tier4_perception_msgs::msg::TrafficLightArray arr;
  arr.header.stamp = stamp;
  arr.header.frame_id = frame_id;
  arr.signals.push_back(sig);
  return arr;
}

struct Input
{
  CameraInfo cam_info;
  tier4_perception_msgs::msg::TrafficLightRoiArray rois;
  tier4_perception_msgs::msg::TrafficLightArray signals;
};

Input make_input(
  const std::string & frame_id, const T4Signal & sig,
  const rclcpp::Time & stamp = rclcpp::Time(0, 0))
{
  return Input{
    make_cam_info(stamp, frame_id), make_roi_array(stamp, frame_id, sig.traffic_light_id),
    make_signal_array(stamp, frame_id, sig)};
}

bool has_element(
  const autoware_perception_msgs::msg::TrafficLightGroup & group, uint8_t color, uint8_t shape)
{
  for (const auto & e : group.elements) {
    if (e.color == color && e.shape == shape) {
      return true;
    }
  }
  return false;
}

}  // namespace

// Single observation accumulates above the prior and is emitted.
TEST(PerElementFusion, SingleObservationProducesElement)
{
  MultiCameraFusion fusion(make_per_element_config());
  const auto input = make_input(
    "camera0", make_signal(LEFT_TL_ID, {make_element(T4Element::GREEN, T4Element::CIRCLE, 0.9f)}));

  const auto result = fusion.fuse(input.cam_info, input.rois, input.signals);

  ASSERT_EQ(result.traffic_light_groups.traffic_light_groups.size(), 1u);
  const auto & group = result.traffic_light_groups.traffic_light_groups.front();
  EXPECT_EQ(group.traffic_light_group_id, REGULATORY_ELEMENT_ID);
  EXPECT_TRUE(has_element(group, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE));
  EXPECT_TRUE(result.conflicted_regulatory_element_status.empty());
}

// Union case: A reports {RED-CIRCLE}, B reports {RED-CIRCLE, GREEN-UP_ARROW}. Per-element fusion
// emits both elements because each is positively supported by at least one observation.
// This is the motivating case from the design discussion.
TEST(PerElementFusion, UnionOfPartiallyOverlappingObservations)
{
  MultiCameraFusion fusion(make_per_element_config());
  const auto input_a = make_input(
    "camera0", make_signal(LEFT_TL_ID, {make_element(T4Element::RED, T4Element::CIRCLE, 0.9f)}));
  const auto input_b = make_input(
    "camera1", make_signal(
                 RIGHT_TL_ID, {make_element(T4Element::RED, T4Element::CIRCLE, 0.9f),
                               make_element(T4Element::GREEN, T4Element::UP_ARROW, 0.85f)}));

  fusion.fuse(input_a.cam_info, input_a.rois, input_a.signals);
  const auto result = fusion.fuse(input_b.cam_info, input_b.rois, input_b.signals);

  ASSERT_EQ(result.traffic_light_groups.traffic_light_groups.size(), 1u);
  const auto & group = result.traffic_light_groups.traffic_light_groups.front();
  EXPECT_TRUE(has_element(group, TrafficLightElement::RED, TrafficLightElement::CIRCLE));
  EXPECT_TRUE(has_element(group, TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW));
  EXPECT_TRUE(result.conflicted_regulatory_element_status.empty());
}

// Confidence gate drops a low-confidence observation entirely.
TEST(PerElementFusion, ConfidenceGateDropsLowConfidenceObservation)
{
  auto cfg = make_per_element_config();
  cfg.per_element_config.confidence_gate = 0.5;
  MultiCameraFusion fusion(cfg);
  const auto input = make_input(
    "camera0", make_signal(LEFT_TL_ID, {make_element(T4Element::GREEN, T4Element::CIRCLE, 0.3f)}));

  const auto result = fusion.fuse(input.cam_info, input.rois, input.signals);

  ASSERT_EQ(result.traffic_light_groups.traffic_light_groups.size(), 1u);
  const auto & group = result.traffic_light_groups.traffic_light_groups.front();
  // The single observation was dropped by the gate → group is emitted as UNKNOWN (no element
  // passed threshold but the regulatory element was observed).
  EXPECT_TRUE(has_element(group, TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN));
}

// All-UNKNOWN observation: group still emits an UNKNOWN element so downstream consumers see the
// regulatory element entry.
TEST(PerElementFusion, AllUnknownObservationsEmitUnknownGroup)
{
  MultiCameraFusion fusion(make_per_element_config());
  const auto input = make_input(
    "camera0",
    make_signal(LEFT_TL_ID, {make_element(T4Element::UNKNOWN, T4Element::UNKNOWN, 0.0f)}));

  const auto result = fusion.fuse(input.cam_info, input.rois, input.signals);

  ASSERT_EQ(result.traffic_light_groups.traffic_light_groups.size(), 1u);
  const auto & group = result.traffic_light_groups.traffic_light_groups.front();
  EXPECT_TRUE(has_element(group, TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN));
}

// Mutex conflict: RED-CIRCLE and GREEN-CIRCLE both pass threshold. Soft policy (default) keeps
// the argmax element. Higher-confidence GREEN wins over RED here.
TEST(PerElementFusion, MutexConflictSoftPolicyKeepsArgmax)
{
  MultiCameraFusion fusion(make_per_element_config());
  const auto input_red = make_input(
    "camera0", make_signal(LEFT_TL_ID, {make_element(T4Element::RED, T4Element::CIRCLE, 0.7f)}));
  const auto input_green = make_input(
    "camera1",
    make_signal(RIGHT_TL_ID, {make_element(T4Element::GREEN, T4Element::CIRCLE, 0.95f)}));

  fusion.fuse(input_red.cam_info, input_red.rois, input_red.signals);
  const auto result = fusion.fuse(input_green.cam_info, input_green.rois, input_green.signals);

  ASSERT_EQ(result.traffic_light_groups.traffic_light_groups.size(), 1u);
  const auto & group = result.traffic_light_groups.traffic_light_groups.front();
  EXPECT_TRUE(has_element(group, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE));
  EXPECT_FALSE(has_element(group, TrafficLightElement::RED, TrafficLightElement::CIRCLE));
  ASSERT_EQ(result.conflicted_regulatory_element_status.size(), 1u);
  EXPECT_EQ(
    result.conflicted_regulatory_element_status.front().conflict_type,
    ConflictType::PARTIAL_CONFLICT);
}

// Mutex conflict with strict_mode = true → failsafe UNKNOWN output, full CONFLICT diagnostic.
TEST(PerElementFusion, MutexConflictStrictModeEmitsFailsafe)
{
  auto cfg = make_per_element_config();
  cfg.per_element_config.strict_mode = true;
  MultiCameraFusion fusion(cfg);
  const auto input_red = make_input(
    "camera0", make_signal(LEFT_TL_ID, {make_element(T4Element::RED, T4Element::CIRCLE, 0.9f)}));
  const auto input_green = make_input(
    "camera1", make_signal(RIGHT_TL_ID, {make_element(T4Element::GREEN, T4Element::CIRCLE, 0.9f)}));

  fusion.fuse(input_red.cam_info, input_red.rois, input_red.signals);
  const auto result = fusion.fuse(input_green.cam_info, input_green.rois, input_green.signals);

  ASSERT_EQ(result.traffic_light_groups.traffic_light_groups.size(), 1u);
  const auto & group = result.traffic_light_groups.traffic_light_groups.front();
  EXPECT_TRUE(has_element(group, TrafficLightElement::UNKNOWN, TrafficLightElement::UNKNOWN));
  ASSERT_EQ(result.conflicted_regulatory_element_status.size(), 1u);
  EXPECT_EQ(
    result.conflicted_regulatory_element_status.front().conflict_type, ConflictType::CONFLICT);
}

// Two cameras agree on RED → both contribute → log-odds reinforce → RED is emitted.
TEST(PerElementFusion, AgreementBetweenCamerasReinforces)
{
  MultiCameraFusion fusion(make_per_element_config());
  const auto input_a = make_input(
    "camera0", make_signal(LEFT_TL_ID, {make_element(T4Element::RED, T4Element::CIRCLE, 0.7f)}));
  const auto input_b = make_input(
    "camera1", make_signal(RIGHT_TL_ID, {make_element(T4Element::RED, T4Element::CIRCLE, 0.7f)}));

  fusion.fuse(input_a.cam_info, input_a.rois, input_a.signals);
  const auto result = fusion.fuse(input_b.cam_info, input_b.rois, input_b.signals);

  ASSERT_EQ(result.traffic_light_groups.traffic_light_groups.size(), 1u);
  const auto & group = result.traffic_light_groups.traffic_light_groups.front();
  ASSERT_FALSE(group.elements.empty());
  EXPECT_TRUE(has_element(group, TrafficLightElement::RED, TrafficLightElement::CIRCLE));
  // The output confidence (sigmoid of accumulated log-odds) should exceed the single-observation
  // confidence because two pieces of evidence at p=0.7 each compound positively.
  for (const auto & e : group.elements) {
    if (e.color == TrafficLightElement::RED && e.shape == TrafficLightElement::CIRCLE) {
      EXPECT_GT(e.confidence, 0.7f);
    }
  }
}

// Temporal smoothing: a brief UNKNOWN frame after a confident RED still emits RED, because the
// older RED record stays within the lifespan window.
TEST(PerElementFusion, BriefUnknownIsOutvotedByRecentKnownObservation)
{
  MultiCameraFusion fusion(make_per_element_config());
  const auto known_input = make_input(
    "camera0", make_signal(LEFT_TL_ID, {make_element(T4Element::RED, T4Element::CIRCLE, 0.9f)}),
    rclcpp::Time(100, 0));
  const auto unknown_input = make_input(
    "camera0",
    make_signal(LEFT_TL_ID, {make_element(T4Element::UNKNOWN, T4Element::UNKNOWN, 0.0f)}),
    rclcpp::Time(100, 100000000));  // 0.1s later; still inside the 1.0s default lifespan

  fusion.fuse(known_input.cam_info, known_input.rois, known_input.signals);
  const auto result =
    fusion.fuse(unknown_input.cam_info, unknown_input.rois, unknown_input.signals);

  ASSERT_EQ(result.traffic_light_groups.traffic_light_groups.size(), 1u);
  const auto & group = result.traffic_light_groups.traffic_light_groups.front();
  EXPECT_TRUE(has_element(group, TrafficLightElement::RED, TrafficLightElement::CIRCLE));
}
