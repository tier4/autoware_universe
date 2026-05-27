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

#ifndef GUIDANCE_TRIGGER_PANEL_HPP_
#define GUIDANCE_TRIGGER_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <std_srvs/srv/set_bool.hpp>

namespace autoware::guidance_trigger_rviz_plugin
{

class GuidanceTriggerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GuidanceTriggerPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void onStartPressed();
  void onStartReleased();
  void onCenterlinePressed();
  void onCenterlineReleased();
  void onStopPressed();
  void onStopReleased();

private:
  using SetBool = std_srvs::srv::SetBool;
  using SetBoolClient = rclcpp::Client<SetBool>;

  void createClients();
  void callService(const SetBoolClient::SharedPtr & client, bool enabled, const QString & name);
  void updateButtonStyle(QPushButton * button, bool enabled);
  void setStatus(const QString & status);

  QPushButton * start_button_;
  QPushButton * centerline_button_;
  QPushButton * stop_button_;
  QLabel * status_label_;

  SetBoolClient::SharedPtr start_client_;
  SetBoolClient::SharedPtr centerline_client_;
  SetBoolClient::SharedPtr stop_client_;

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

}  // namespace autoware::guidance_trigger_rviz_plugin

#endif  // GUIDANCE_TRIGGER_PANEL_HPP_
