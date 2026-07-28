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

#include "driving_factor_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

#include <autoware_lane_event_classifier/msg/driving_state.hpp>

namespace rviz_plugins
{

namespace
{

struct StateStyle
{
  const char * label;
  const char * bg_color;
};

StateStyle style_for_state(uint8_t state)
{
  using DrivingState = autoware_lane_event_classifier::msg::DrivingState;
  switch (state) {
    case DrivingState::LANE_FOLLOWING:
      return {"LANE FOLLOWING", "#2E7D32"};
    case DrivingState::LANE_CHANGING:
      return {"LANE CHANGING", "#F57C00"};
    case DrivingState::ABORTING_LANE_CHANGE:
      return {"ABORTING LANE CHANGE", "#C62828"};
    case DrivingState::INTENTIONAL_LANE_CROSSING:
      return {"INTENTIONAL CROSSING", "#1565C0"};
    case DrivingState::ABORTING_INTENTIONAL_LANE_CROSSING:
      return {"ABORTING CROSSING", "#BF360C"};
    default:  // UNKNOWN
      return {"UNKNOWN", "#424242"};
  }
}

}  // namespace

DrivingFactorPanel::DrivingFactorPanel(QWidget * parent)
: rviz_common::Panel(parent), subscription_(nullptr)
{
  auto * layout = new QVBoxLayout(this);

  state_label_ = new QLabel(this);
  state_label_->setAlignment(Qt::AlignCenter);
  state_label_->setText("No data");
  state_label_->setStyleSheet(
    "QLabel {"
    "  padding: 12px;"
    "  border-radius: 6px;"
    "  background-color: #424242;"
    "  color: white;"
    "  font-size: 14px;"
    "  font-weight: bold;"
    "}");

  layout->addWidget(state_label_);
  setLayout(layout);
}

DrivingFactorPanel::~DrivingFactorPanel()
{
  unsubscribe();
}

void DrivingFactorPanel::onInitialize()
{
  subscribe();
}

void DrivingFactorPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

void DrivingFactorPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void DrivingFactorPanel::processMessage(
  const autoware_lane_event_classifier::msg::DrivingFactor::ConstSharedPtr msg)
{
  const auto [label, bg_color] = style_for_state(msg->driving_state.state);
  state_label_->setText(label);
  state_label_->setStyleSheet(
    QString(
      "QLabel {"
      "  padding: 12px;"
      "  border-radius: 6px;"
      "  background-color: %1;"
      "  color: white;"
      "  font-size: 14px;"
      "  font-weight: bold;"
      "}")
      .arg(bg_color));
}

void DrivingFactorPanel::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  try {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    subscription_ =
      node->create_subscription<autoware_lane_event_classifier::msg::DrivingFactor>(
        "/planning/driving_factor", 10,
        std::bind(&DrivingFactorPanel::processMessage, this, std::placeholders::_1));
  } catch (const std::exception & e) {
    state_label_->setText("Error: subscription failed");
    state_label_->setStyleSheet(
      "QLabel {"
      "  padding: 12px;"
      "  border-radius: 6px;"
      "  background-color: #B71C1C;"
      "  color: white;"
      "  font-size: 14px;"
      "  font-weight: bold;"
      "}");
  }
}

void DrivingFactorPanel::unsubscribe()
{
  subscription_.reset();
}

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::DrivingFactorPanel, rviz_common::Panel)
