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

#ifndef AUTOWARE__LOCALIZATION_MONITOR_HOST__MONITOR_DATA_HPP_
#define AUTOWARE__LOCALIZATION_MONITOR_HOST__MONITOR_DATA_HPP_

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <memory>
#include <optional>

namespace autoware::localization_monitor_host
{

struct MonitorData
{
  std::optional<nav_msgs::msg::Odometry> latest_odom{std::nullopt};
  std::optional<nav_msgs::msg::Odometry> prev_odom{std::nullopt};
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> twist_buffer;
  bool odom_received{false};
};

}  // namespace autoware::localization_monitor_host

#endif  // AUTOWARE__LOCALIZATION_MONITOR_HOST__MONITOR_DATA_HPP_
