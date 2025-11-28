// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__LIDAR_FRNET__LIDAR_FRNET_NODE_HPP_
#define AUTOWARE__LIDAR_FRNET__LIDAR_FRNET_NODE_HPP_

#include "autoware/lidar_frnet/lidar_frnet.hpp"
#include "autoware/lidar_frnet/ros_utils.hpp"
#include "autoware/lidar_frnet/utils.hpp"
#include "autoware/lidar_frnet/visibility_control.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::lidar_frnet
{

class LIDAR_FRNET_PUBLIC LidarFRNetNode : public rclcpp::Node
{
public:
  explicit LidarFRNetNode(const rclcpp::NodeOptions & options);

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const std::string & topic_name);
  void diagnoseProcessingTime(diagnostic_updater::DiagnosticStatusWrapper & stat);

private:
  struct LidarProcessor
  {
    std::string topic_name;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_seg_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_viz_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_filtered_pub;
    std::unique_ptr<LidarFRNet> frnet;
    std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr;
    std::optional<double> last_processing_time_ms;
  };

  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_pub_{nullptr};

  std::unique_ptr<diagnostic_updater::Updater> diag_updater_{nullptr};

  const ros_utils::PointCloudLayout cloud_seg_layout_;
  const ros_utils::PointCloudLayout cloud_viz_layout_;
  const ros_utils::PointCloudLayout cloud_filtered_layout_;

  utils::DiagnosticParams diag_params_{};
  std::optional<rclcpp::Time> last_in_time_processing_timestamp_;
  std::mutex
    diagnostic_mutex_;  // Protects last_in_time_processing_timestamp_ and debug_publisher_ptr_

  // Multi-LiDAR support
  bool joint_inference_{};
  std::vector<std::string> input_topics_;
  std::unordered_map<std::string, std::shared_ptr<LidarProcessor>> lidar_processors_;
  std::mutex processors_mutex_;  // Protects lidar_processors_ map access
};

}  // namespace autoware::lidar_frnet

#endif  // AUTOWARE__LIDAR_FRNET__LIDAR_FRNET_NODE_HPP_
