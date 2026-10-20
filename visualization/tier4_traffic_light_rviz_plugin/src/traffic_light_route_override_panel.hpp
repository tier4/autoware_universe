//
//  Copyright 2026 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef TRAFFIC_LIGHT_ROUTE_OVERRIDE_PANEL_HPP_
#define TRAFFIC_LIGHT_ROUTE_OVERRIDE_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QTableWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#endif

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace rviz_plugins
{

using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::LaneletRoute;

struct TrafficLightEntry
{
  int64_t id{};
  bool is_pedestrian{false};
  uint8_t override_color{TrafficLightElement::GREEN};
  QComboBox * override_combo{nullptr};
  QLabel * current_color_label{nullptr};
};

struct RouteTrafficLightInfo
{
  int64_t id{};
  bool is_pedestrian{false};
};

class TrafficLightRouteOverridePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TrafficLightRouteOverridePanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:
  void onToggleEnable();

protected:
  void onVectorMap(const LaneletMapBin::ConstSharedPtr msg);
  void onRoute(const LaneletRoute::ConstSharedPtr msg);
  void onTrafficSignals(const TrafficLightGroupArray::ConstSharedPtr msg);
  void onTimer();

  void rebuildTable(const std::vector<RouteTrafficLightInfo> & traffic_lights);
  void updateCurrentColorLabels();
  TrafficLightGroupArray buildTrafficLightGroupArray() const;
  std::vector<RouteTrafficLightInfo> extractTrafficLightIdsOnRoute(
    const LaneletRoute::ConstSharedPtr & route) const;
  void updateToggleButtonStyle();

  static QString colorToString(uint8_t color);
  static void setColorLabelStyle(QLabel * label, uint8_t color);
  static uint8_t colorFromComboText(const QString & text);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr pub_traffic_signals_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr sub_traffic_signals_;

  QTableWidget * traffic_table_{nullptr};
  QPushButton * toggle_button_{nullptr};

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;

  std::vector<TrafficLightEntry> entries_;
  std::unordered_map<int64_t, uint8_t> current_colors_;

  std::optional<std::array<uint8_t, 16>> last_route_uuid_;
  bool enable_publish_{false};
  bool received_vector_map_{false};
};

}  // namespace rviz_plugins

#endif  // TRAFFIC_LIGHT_ROUTE_OVERRIDE_PANEL_HPP_
