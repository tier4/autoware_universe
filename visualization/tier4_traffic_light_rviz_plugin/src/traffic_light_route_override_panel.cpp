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

#include "traffic_light_route_override_panel.hpp"

#include <QFont>
#include <QHeaderView>
#include <QVBoxLayout>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rviz_common/display_context.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <set>
#include <string>
#include <utility>

#undef signals
namespace rviz_plugins
{

namespace
{
constexpr int kPublishRateHz = 10;
constexpr int kPedestrianGraphId = 1;

bool isSameUuid(
  const std::array<uint8_t, 16> & a, const std::array<uint8_t, 16> & b)
{
  return a == b;
}

std::array<uint8_t, 16> toUuidArray(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::array<uint8_t, 16> result{};
  std::copy(uuid.uuid.begin(), uuid.uuid.end(), result.begin());
  return result;
}

void appendUniqueTrafficLightIds(
  const std::vector<lanelet::AutowareTrafficLightConstPtr> & traffic_lights,
  std::set<int64_t> & ids)
{
  for (const auto & tl : traffic_lights) {
    ids.insert(tl->id());
  }
}
}  // namespace

TrafficLightRouteOverridePanel::TrafficLightRouteOverridePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * vertical_header = new QHeaderView(Qt::Vertical);
  vertical_header->hide();
  auto * horizontal_header = new QHeaderView(Qt::Horizontal);
  horizontal_header->setSectionResizeMode(QHeaderView::Stretch);

  traffic_table_ = new QTableWidget();
  traffic_table_->setColumnCount(3);
  traffic_table_->setHorizontalHeaderLabels({"ID", "Current", "Override"});
  traffic_table_->setVerticalHeader(vertical_header);
  traffic_table_->setHorizontalHeader(horizontal_header);

  toggle_button_ = new QPushButton("DISABLED");
  toggle_button_->setMinimumHeight(50);
  toggle_button_->setFont(QFont("Sans", 10, QFont::Bold));
  connect(toggle_button_, SIGNAL(clicked()), this, SLOT(onToggleEnable()));

  auto * layout = new QVBoxLayout();
  layout->addWidget(traffic_table_);
  layout->addWidget(toggle_button_);
  setLayout(layout);

  updateToggleButtonStyle();
}

void TrafficLightRouteOverridePanel::onInitialize()
{
  using std::placeholders::_1;
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pub_traffic_signals_ = raw_node_->create_publisher<TrafficLightGroupArray>(
    "/perception/traffic_light_recognition/traffic_signals", rclcpp::QoS(1));

  sub_vector_map_ = raw_node_->create_subscription<LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightRouteOverridePanel::onVectorMap, this, _1));

  sub_route_ = raw_node_->create_subscription<LaneletRoute>(
    "/planning/mission_planning/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightRouteOverridePanel::onRoute, this, _1));

  sub_traffic_signals_ = raw_node_->create_subscription<TrafficLightGroupArray>(
    "/perception/traffic_light_recognition/traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightRouteOverridePanel::onTrafficSignals, this, _1));

  const auto period = std::chrono::milliseconds(static_cast<int64_t>(1000 / kPublishRateHz));
  pub_timer_ = raw_node_->create_wall_timer(
    period, std::bind(&TrafficLightRouteOverridePanel::onTimer, this));
}

void TrafficLightRouteOverridePanel::onToggleEnable()
{
  enable_publish_ = !enable_publish_;
  updateToggleButtonStyle();

  if (enable_publish_ && !entries_.empty()) {
    auto msg = buildTrafficLightGroupArray();
    msg.stamp = raw_node_->get_clock()->now();
    pub_traffic_signals_->publish(msg);
  }
}

void TrafficLightRouteOverridePanel::updateToggleButtonStyle()
{
  if (enable_publish_) {
    toggle_button_->setText("ENABLED (10Hz)");
    toggle_button_->setStyleSheet(
      "QPushButton { background-color: #336633; color: white; border-radius: 6px; }");
  } else {
    toggle_button_->setText("DISABLED");
    toggle_button_->setStyleSheet(
      "QPushButton { background-color: #808080; color: white; border-radius: 6px; }");
  }
}

void TrafficLightRouteOverridePanel::onVectorMap(const LaneletMapBin::ConstSharedPtr msg)
{
  if (received_vector_map_) {
    return;
  }

  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  const lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  const lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);

  received_vector_map_ = true;
  RCLCPP_INFO(raw_node_->get_logger(), "Received vector map for route traffic light override panel");
}

void TrafficLightRouteOverridePanel::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  if (!received_vector_map_ || lanelet_map_ptr_ == nullptr || overall_graphs_ptr_ == nullptr) {
    RCLCPP_WARN(
      raw_node_->get_logger(), "Cannot extract route traffic lights because vector map is not ready");
    return;
  }

  const auto route_uuid = toUuidArray(msg->uuid);
  if (last_route_uuid_.has_value() && isSameUuid(last_route_uuid_.value(), route_uuid)) {
    return;
  }
  last_route_uuid_ = route_uuid;

  const auto traffic_lights = extractTrafficLightIdsOnRoute(msg);
  rebuildTable(traffic_lights);

  RCLCPP_INFO(
    raw_node_->get_logger(), "Updated route traffic lights: %zu signals found",
    traffic_lights.size());
}

std::vector<RouteTrafficLightInfo> TrafficLightRouteOverridePanel::extractTrafficLightIdsOnRoute(
  const LaneletRoute::ConstSharedPtr & route) const
{
  lanelet::ConstLanelets route_lanelets;
  for (const auto & segment : route->segments) {
    for (const auto & primitive : segment.primitives) {
      try {
        route_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(primitive.id));
      } catch (const lanelet::NoSuchPrimitiveError & ex) {
        RCLCPP_ERROR(raw_node_->get_logger(), "%s", ex.what());
        return {};
      }
    }
  }

  std::set<int64_t> vehicle_traffic_light_ids;
  appendUniqueTrafficLightIds(
    lanelet::utils::query::autowareTrafficLights(route_lanelets), vehicle_traffic_light_ids);

  std::set<int64_t> pedestrian_traffic_light_ids;
  lanelet::ConstLanelets conflicting_crosswalks;
  for (const auto & route_lanelet : route_lanelets) {
    const auto conflict_lanelets =
      overall_graphs_ptr_->conflictingInGraph(route_lanelet, kPedestrianGraphId);
    for (const auto & lanelet : conflict_lanelets) {
      conflicting_crosswalks.push_back(lanelet);
    }
  }
  appendUniqueTrafficLightIds(
    lanelet::utils::query::autowareTrafficLights(conflicting_crosswalks),
    pedestrian_traffic_light_ids);

  std::vector<RouteTrafficLightInfo> traffic_lights;
  traffic_lights.reserve(vehicle_traffic_light_ids.size() + pedestrian_traffic_light_ids.size());

  for (const auto id : vehicle_traffic_light_ids) {
    traffic_lights.push_back(RouteTrafficLightInfo{id, false});
  }
  for (const auto id : pedestrian_traffic_light_ids) {
    if (vehicle_traffic_light_ids.count(id) == 0) {
      traffic_lights.push_back(RouteTrafficLightInfo{id, true});
    }
  }

  std::sort(traffic_lights.begin(), traffic_lights.end(), [](const auto & a, const auto & b) {
    return a.id < b.id;
  });

  return traffic_lights;
}

void TrafficLightRouteOverridePanel::rebuildTable(
  const std::vector<RouteTrafficLightInfo> & traffic_lights)
{
  entries_.clear();
  traffic_table_->setRowCount(static_cast<int>(traffic_lights.size()));

  for (size_t i = 0; i < traffic_lights.size(); ++i) {
    const auto & traffic_light = traffic_lights.at(i);

    TrafficLightEntry entry;
    entry.id = traffic_light.id;
    entry.is_pedestrian = traffic_light.is_pedestrian;
    entry.override_color =
      traffic_light.is_pedestrian ? TrafficLightElement::RED : TrafficLightElement::GREEN;

    auto * id_label = new QLabel(QString::number(traffic_light.id));
    id_label->setAlignment(Qt::AlignCenter);

    auto * current_label = new QLabel(colorToString(TrafficLightElement::UNKNOWN));
    current_label->setAlignment(Qt::AlignCenter);
    setColorLabelStyle(current_label, TrafficLightElement::UNKNOWN);

    auto * override_combo = new QComboBox();
    override_combo->addItems({"RED", "AMBER", "GREEN", "WHITE", "UNKNOWN"});
    override_combo->setCurrentText(colorToString(entry.override_color));

    const int row = static_cast<int>(i);
    traffic_table_->setCellWidget(row, 0, id_label);
    traffic_table_->setCellWidget(row, 1, current_label);
    traffic_table_->setCellWidget(row, 2, override_combo);

    entry.current_color_label = current_label;
    entry.override_combo = override_combo;
    entries_.push_back(entry);
  }

  updateCurrentColorLabels();
  traffic_table_->update();
}

void TrafficLightRouteOverridePanel::onTrafficSignals(
  const TrafficLightGroupArray::ConstSharedPtr msg)
{
  if (enable_publish_) {
    return;
  }

  for (const auto & signal : msg->traffic_light_groups) {
    if (signal.elements.empty()) {
      current_colors_[signal.traffic_light_group_id] = TrafficLightElement::UNKNOWN;
      continue;
    }
    current_colors_[signal.traffic_light_group_id] = signal.elements.front().color;
  }

  updateCurrentColorLabels();
}

void TrafficLightRouteOverridePanel::updateCurrentColorLabels()
{
  for (auto & entry : entries_) {
    if (entry.current_color_label == nullptr) {
      continue;
    }

    const auto it = current_colors_.find(entry.id);
    const uint8_t color =
      it != current_colors_.end() ? it->second : TrafficLightElement::UNKNOWN;
    entry.current_color_label->setText(colorToString(color));
    setColorLabelStyle(entry.current_color_label, color);
  }
}

TrafficLightGroupArray TrafficLightRouteOverridePanel::buildTrafficLightGroupArray() const
{
  TrafficLightGroupArray msg;
  msg.traffic_light_groups.reserve(entries_.size());

  for (const auto & entry : entries_) {
    TrafficLightElement element;
    element.shape = TrafficLightElement::CIRCLE;
    element.status = TrafficLightElement::SOLID_ON;
    element.confidence = 1.0F;
    element.color = entry.override_combo != nullptr
                      ? colorFromComboText(entry.override_combo->currentText())
                      : entry.override_color;

    TrafficLightGroup group;
    group.traffic_light_group_id = entry.id;
    group.elements.push_back(element);
    msg.traffic_light_groups.push_back(group);
  }

  return msg;
}

void TrafficLightRouteOverridePanel::onTimer()
{
  if (enable_publish_ && !entries_.empty()) {
    auto msg = buildTrafficLightGroupArray();
    msg.stamp = raw_node_->get_clock()->now();
    pub_traffic_signals_->publish(msg);

    for (const auto & group : msg.traffic_light_groups) {
      if (group.elements.empty()) {
        continue;
      }
      current_colors_[group.traffic_light_group_id] = group.elements.front().color;
    }
    updateCurrentColorLabels();
  }
}

QString TrafficLightRouteOverridePanel::colorToString(const uint8_t color)
{
  switch (color) {
    case TrafficLightElement::RED:
      return "RED";
    case TrafficLightElement::AMBER:
      return "AMBER";
    case TrafficLightElement::GREEN:
      return "GREEN";
    case TrafficLightElement::WHITE:
      return "WHITE";
    default:
      return "UNKNOWN";
  }
}

void TrafficLightRouteOverridePanel::setColorLabelStyle(QLabel * label, const uint8_t color)
{
  switch (color) {
    case TrafficLightElement::RED:
      label->setStyleSheet("background-color: #FF0000;");
      break;
    case TrafficLightElement::AMBER:
      label->setStyleSheet("background-color: #FFBF00;");
      break;
    case TrafficLightElement::GREEN:
      label->setStyleSheet("background-color: #7CFC00;");
      break;
    case TrafficLightElement::WHITE:
      label->setStyleSheet("background-color: #FFFFFF;");
      break;
    default:
      label->setStyleSheet("background-color: #808080;");
      break;
  }
}

uint8_t TrafficLightRouteOverridePanel::colorFromComboText(const QString & text)
{
  if (text == "RED") {
    return TrafficLightElement::RED;
  }
  if (text == "AMBER") {
    return TrafficLightElement::AMBER;
  }
  if (text == "GREEN") {
    return TrafficLightElement::GREEN;
  }
  if (text == "WHITE") {
    return TrafficLightElement::WHITE;
  }
  return TrafficLightElement::UNKNOWN;
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::TrafficLightRouteOverridePanel, rviz_common::Panel)
