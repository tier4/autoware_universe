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

#include "guidance_trigger_panel.hpp"

#include <QFont>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QVBoxLayout>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include <memory>

namespace autoware::guidance_trigger_rviz_plugin
{

namespace
{
constexpr auto k_default_start_service =
  "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/service/"
  "set_start_guidance_enabled";
constexpr auto k_default_centerline_service =
  "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/service/"
  "set_centerline_guidance_enabled";
constexpr auto k_default_stop_service =
  "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/service/"
  "set_stop_guidance_enabled";
}  // namespace

GuidanceTriggerPanel::GuidanceTriggerPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  auto * button_layout = new QHBoxLayout();
  start_button_ = new QPushButton("START");
  centerline_button_ = new QPushButton("CENTERLINE");
  stop_button_ = new QPushButton("STOP");
  for (auto * button : {start_button_, centerline_button_, stop_button_}) {
    button->setMinimumHeight(64);
    button->setFont(QFont("Sans", 12, QFont::Bold));
    button_layout->addWidget(button);
    updateButtonStyle(button, false);
  }
  layout->addLayout(button_layout);

  status_label_ = new QLabel("Ready");
  layout->addWidget(status_label_);

  connect(start_button_, &QPushButton::pressed, this, &GuidanceTriggerPanel::onStartPressed);
  connect(start_button_, &QPushButton::released, this, &GuidanceTriggerPanel::onStartReleased);
  connect(
    centerline_button_, &QPushButton::pressed, this, &GuidanceTriggerPanel::onCenterlinePressed);
  connect(
    centerline_button_, &QPushButton::released, this, &GuidanceTriggerPanel::onCenterlineReleased);
  connect(stop_button_, &QPushButton::pressed, this, &GuidanceTriggerPanel::onStopPressed);
  connect(stop_button_, &QPushButton::released, this, &GuidanceTriggerPanel::onStopReleased);

  setLayout(layout);
}

void GuidanceTriggerPanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  createClients();
}

void GuidanceTriggerPanel::onStartPressed()
{
  updateButtonStyle(start_button_, true);
  callService(start_client_, true, "start guidance");
}

void GuidanceTriggerPanel::onStartReleased()
{
  updateButtonStyle(start_button_, false);
  callService(start_client_, false, "start guidance");
}

void GuidanceTriggerPanel::onCenterlinePressed()
{
  updateButtonStyle(centerline_button_, true);
  callService(centerline_client_, true, "centerline guidance");
}

void GuidanceTriggerPanel::onCenterlineReleased()
{
  updateButtonStyle(centerline_button_, false);
  callService(centerline_client_, false, "centerline guidance");
}

void GuidanceTriggerPanel::onStopPressed()
{
  updateButtonStyle(stop_button_, true);
  callService(stop_client_, true, "stop guidance");
}

void GuidanceTriggerPanel::onStopReleased()
{
  updateButtonStyle(stop_button_, false);
  callService(stop_client_, false, "stop guidance");
}

void GuidanceTriggerPanel::createClients()
{
  if (rviz_ros_node_.expired()) {
    setStatus("RViz ROS node is not ready");
    return;
  }

  auto node = rviz_ros_node_.lock()->get_raw_node();
  start_client_ = node->create_client<SetBool>(k_default_start_service);
  centerline_client_ = node->create_client<SetBool>(k_default_centerline_service);
  stop_client_ = node->create_client<SetBool>(k_default_stop_service);
  setStatus("Ready");
}

void GuidanceTriggerPanel::callService(
  const SetBoolClient::SharedPtr & client, const bool enabled, const QString & name)
{
  if (!client) {
    setStatus("Service client is not ready");
    return;
  }

  auto request = std::make_shared<SetBool::Request>();
  request->data = enabled;

  const auto state = enabled ? QString("enabled") : QString("disabled");
  if (!client->service_is_ready()) {
    setStatus(QString("%1 service is not ready").arg(name));
    return;
  }

  client->async_send_request(
    request, [this, name, state](const SetBoolClient::SharedFuture future) {
      const auto & response = future.get();
      if (response->success) {
        setStatus(QString("%1 %2").arg(name, state));
      } else {
        setStatus(
          QString("%1 request failed: %2").arg(name, QString::fromStdString(response->message)));
      }
    });
}

void GuidanceTriggerPanel::updateButtonStyle(QPushButton * button, const bool enabled)
{
  if (enabled) {
    button->setStyleSheet(
      "QPushButton { background-color: #CC3333; color: white; border-radius: 6px; }");
  } else {
    button->setStyleSheet(
      "QPushButton { background-color: #3A5F8A; color: white; border-radius: 6px; }");
  }
}

void GuidanceTriggerPanel::setStatus(const QString & status)
{
  QMetaObject::invokeMethod(
    this, [this, status]() { status_label_->setText(status); }, Qt::QueuedConnection);
}

void GuidanceTriggerPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void GuidanceTriggerPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

}  // namespace autoware::guidance_trigger_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::guidance_trigger_rviz_plugin::GuidanceTriggerPanel, rviz_common::Panel)
