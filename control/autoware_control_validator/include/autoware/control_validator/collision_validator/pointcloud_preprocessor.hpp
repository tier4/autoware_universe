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

#ifndef AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__POINTCLOUD_PREPROCESSOR_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__POINTCLOUD_PREPROCESSOR_HPP_

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::control_validator::collision_validator
{
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

struct PointCloudParam
{
  double roi_rear{2.0};
  double roi_front_margin{2.0};
  double roi_lateral{5.0};
  double z_margin_low{0.0};
  double z_margin_high{0.0};
  size_t max_voxels{20000};
  double voxel_size{0.10};
  double cluster_tolerance{0.30};
  size_t min_cluster_size{5};
};

/// Points of one euclidean cluster and their centroid, in the map frame.
struct PointCluster
{
  std::vector<Point2d> points;
  Point2d centroid;
  size_t raw_point_count{0};
};

struct PointCloudClusters
{
  std::vector<PointCluster> clusters;
  rclcpp::Time stamp;
};

/**
 * @brief Ego-footprint exclusion -> ROI crop -> voxel grid (raw point counts) -> subsample ->
 * euclidean clustering, keeping only the points that belong to a cluster. Points are expressed in
 * the map frame through the ego pose at the pointcloud stamp.
 */
class PointCloudPreprocessor
{
public:
  PointCloudPreprocessor(
    const rclcpp::Logger & logger, const PointCloudParam & param,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils_debug::TimeKeeper>());

  void update_parameters(const PointCloudParam & param);

  /**
   * @brief Process a new frame and keep its clusters.
   * @param pointcloud pointcloud in base_link
   * @param ego_pose_at_pointcloud_stamp ego pose (map) at the pointcloud stamp
   * @param v_ego ego longitudinal velocity [m/s]
   * @param horizon_s evaluated time horizon, extends the ROI ahead (behind when reversing) by
   *   |v_ego| * horizon_s [s]
   * @param vehicle_info height range and footprint of the ego vehicle
   * @return clusters of the last processed frame; nullopt when no frame has been processed. The
   *   same stamp as the last frame, a frame other than base_link or no ego pose is not processed.
   *   An empty cloud gives no clusters.
   * @throws std::runtime_error when the cloud has no x / y / z fields
   */
  const std::optional<PointCloudClusters> & process(
    const sensor_msgs::msg::PointCloud2 & pointcloud,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose_at_pointcloud_stamp, double v_ego,
    double horizon_s, const vehicle_info_utils::VehicleInfo & vehicle_info);

  static constexpr const char * kExpectedFrame = "base_link";

private:
  double z_min(const vehicle_info_utils::VehicleInfo & vehicle_info) const;
  double z_max(const vehicle_info_utils::VehicleInfo & vehicle_info) const;

  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  PointCloudParam param_;
  std::optional<PointCloudClusters> result_;
};
}  // namespace autoware::control_validator::collision_validator

#endif  // AUTOWARE__CONTROL_VALIDATOR__COLLISION_VALIDATOR__POINTCLOUD_PREPROCESSOR_HPP_
