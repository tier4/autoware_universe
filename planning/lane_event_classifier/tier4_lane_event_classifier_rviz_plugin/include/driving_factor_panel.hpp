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

#ifndef DRIVING_FACTOR_PANEL_HPP_
#define DRIVING_FACTOR_PANEL_HPP_

#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_lane_event_classifier/msg/driving_factor.hpp>

namespace rviz_plugins
{

class DrivingFactorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DrivingFactorPanel(QWidget * parent = nullptr);
  ~DrivingFactorPanel() override;

protected:
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  void processMessage(
    const autoware_lane_event_classifier::msg::DrivingFactor::ConstSharedPtr msg);
  void subscribe();
  void unsubscribe();

  rclcpp::Subscription<autoware_lane_event_classifier::msg::DrivingFactor>::SharedPtr subscription_;
  QLabel * state_label_;
};

}  // namespace rviz_plugins

#endif  // DRIVING_FACTOR_PANEL_HPP_
