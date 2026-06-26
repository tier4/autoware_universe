// Copyright 2025 Autoware Foundation
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

#ifndef TIER4_LOCALIZATION_POSE_ESTIMATOR_SWTICH_RVIZ_PLUGIN_HPP_
#define TIER4_LOCALIZATION_POSE_ESTIMATOR_SWTICH_RVIZ_PLUGIN_HPP_

#include <QHBoxLayout>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <std_msgs/msg/string.hpp>
#include <autoware_internal_localization_msgs/srv/pose_estimator_switch.hpp>

#include <memory>

class PoseEstimatorSwitch : public rviz_common::Panel
{
  enum class PoseEstimatorType : int { ndt = 1, yabloc = 2, eagleye = 4, artag = 8 };

  Q_OBJECT
public:
  explicit PoseEstimatorSwitch(QWidget * parent = nullptr);

private:
  void send_switch_request(PoseEstimatorType type);

  QPushButton * ndt_button_;
  QPushButton * gnss_button_;
  QPushButton * yabloc_button_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<autoware_internal_localization_msgs::srv::PoseEstimatorSwitch>::SharedPtr client_;
  QTimer * timer_;

  std::unordered_map<PoseEstimatorType, bool> enable_list_;
};

#endif  // TIER4_LOCALIZATION_POSE_ESTIMATOR_SWTICH_RVIZ_PLUGIN_HPP_
