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

#include "tier4_localization_pose_estimator_switch_rviz_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <memory>
PLUGINLIB_EXPORT_CLASS(PoseEstimatorSwitch, rviz_common::Panel)

PoseEstimatorSwitch::PoseEstimatorSwitch(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QHBoxLayout;

  ndt_button_ = new QPushButton("NDT");
  // yabloc_button_ = new QPushButton("YabLoc");
  gnss_button_ = new QPushButton("GNSS");

  ndt_button_->setStyleSheet("background-color: #606060;");
  // yabloc_button_->setStyleSheet("background-color: #606060;");
  gnss_button_->setStyleSheet("background-color: #606060;");

  layout->addWidget(ndt_button_);
  // layout->addWidget(yabloc_button_);
  layout->addWidget(gnss_button_);
  setLayout(layout);

  enable_list_.emplace(PoseEstimatorType::ndt, false);
  // enable_list_.emplace(PoseEstimatorType::yabloc, false);
  enable_list_.emplace(PoseEstimatorType::eagleye, false);

  node_ = std::make_shared<rclcpp::Node>("tier4_localization_pose_estimate_switch_rviz_plugin");

  // Create the service client
  client_ = node_->create_client<autoware_internal_localization_msgs::srv::PoseEstimatorSwitch>(
    "/localization/pose_estimator_arbiter/switcher");

  // Connect button signals
  connect(ndt_button_, &QPushButton::clicked, this, [this]() { send_switch_request(PoseEstimatorType::ndt); });
  // connect(yabloc_button_, &QPushButton::clicked, this, [this]() { send_switch_request(PoseEstimatorType::yabloc); });
  connect(gnss_button_, &QPushButton::clicked, this, [this]() { send_switch_request(PoseEstimatorType::eagleye); });

  // Optional: Timer to spin the node (needed for service responses)
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() { rclcpp::spin_some(node_); });
  timer_->start(100);  // ms
}

void PoseEstimatorSwitch::send_switch_request(PoseEstimatorType type)
{
  enable_list_[type] = !enable_list_[type];

  if (enable_list_[PoseEstimatorType::ndt]) {
    ndt_button_->setStyleSheet("background-color: #00FF00;");
    ndt_button_->setCheckable(true);
  }
  else {
    ndt_button_->setStyleSheet("background-color: #606060;");
    ndt_button_->setCheckable(true);
  }
  // if (enable_list_[PoseEstimatorType::yabloc]) {
  //   yabloc_button_->setStyleSheet("background-color: #00FF00;");
  //   yabloc_button_->setCheckable(true);
  // }
  // else {
  //   yabloc_button_->setStyleSheet("background-color: #606060;");
  //   yabloc_button_->setCheckable(true);
  // }
  if (enable_list_[PoseEstimatorType::eagleye]) {
    gnss_button_->setStyleSheet("background-color: #00FF00;");
    gnss_button_->setCheckable(true);
  }
  else {
    gnss_button_->setStyleSheet("background-color: #606060;");
    gnss_button_->setCheckable(true);
  }


  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    qWarning("PoseEstimatorSwitch: Service not available");
    return;
  }

  auto request = std::make_shared<autoware_internal_localization_msgs::srv::PoseEstimatorSwitch::Request>();

  uint8_t method = 0;
  for (const auto & estimator_enable : enable_list_ ) {
    if (estimator_enable.second) {
      method += static_cast<uint8_t>(estimator_enable.first);
    }
  }
  request->method = method;

  // Async call
  auto future = client_->async_send_request(
    request, [method](rclcpp::Client<autoware_internal_localization_msgs::srv::PoseEstimatorSwitch>::SharedFuture response) {
      qInfo(
        "PoseEstimatorSwitch: Sent %s lane change -> Success: %d",
        method == autoware_internal_localization_msgs::srv::PoseEstimatorSwitch::Request::NDT    ? "NDT"
        : method == autoware_internal_localization_msgs::srv::PoseEstimatorSwitch::Request::YABLOC ? "YABLOC"
        : method == autoware_internal_localization_msgs::srv::PoseEstimatorSwitch::Request::EAGLEYE ? "GNSS"
                                                        : "Error",
        response.get()->success);
    });
}
