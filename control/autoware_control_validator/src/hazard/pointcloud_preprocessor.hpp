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

#ifndef HAZARD__POINTCLOUD_PREPROCESSOR_HPP_
#define HAZARD__POINTCLOUD_PREPROCESSOR_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::control_validator::hazard
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
  size_t max_points{20000};
  double voxel_size{0.10};
  double cluster_tolerance{0.30};
  size_t min_cluster_size{3};
};

/// Points of one euclidean cluster and their centroid, in the map frame.
struct PointCluster
{
  std::vector<Point2d> points;
  Point2d centroid;
};

/**
 * @brief Ego-footprint exclusion -> ROI crop -> subsample -> voxel grid -> euclidean clustering,
 * keeping only the points that belong to a cluster. Points are expressed in the map frame through
 * the ego pose at the cloud stamp. No hull is built: the points themselves are the output.
 */
class PointCloudPreprocessor
{
public:
  PointCloudPreprocessor(
    const PointCloudParam & param, const vehicle_info_utils::VehicleInfo & vehicle_info);

  void update_parameters(const PointCloudParam & param);

  /**
   * @brief Process a new frame.
   * @param cloud pointcloud in base_link
   * @param ego_at_cloud ego pose (map) at the cloud stamp
   * @param v_ego ego longitudinal velocity [m/s]
   * @param horizon_s evaluated time horizon, extends the ROI ahead by v_ego * horizon_s [s]
   * @param ego_footprint_base_link current ego footprint (base_link); points inside are dropped
   * @return true when the frame was processed. Same stamp as the previous frame, a frame other
   *   than base_link, or an empty cloud returns false and keeps the previous clusters.
   * @throws std::runtime_error when the cloud has no x / y / z fields
   */
  bool process(
    const sensor_msgs::msg::PointCloud2 & cloud, const geometry_msgs::msg::Pose & ego_at_cloud,
    double v_ego, double horizon_s, const Polygon2d & ego_footprint_base_link);

  const std::vector<PointCluster> & clusters() const { return clusters_; }
  const std::optional<rclcpp::Time> & last_stamp() const { return last_stamp_; }
  size_t last_roi_point_count() const { return last_roi_point_count_; }

  /// Effective height bounds (base_link): [z_min(), z_max()).
  double z_min() const { return vehicle_info_.min_height_offset_m + param_.z_margin_low; }
  double z_max() const { return vehicle_info_.max_height_offset_m + param_.z_margin_high; }

  static constexpr const char * kExpectedFrame = "base_link";

private:
  PointCloudParam param_;
  vehicle_info_utils::VehicleInfo vehicle_info_;
  std::optional<rclcpp::Time> last_stamp_;
  std::vector<PointCluster> clusters_;
  size_t last_roi_point_count_{0};
};

/**
 * @brief Drop clusters whose centroid lies inside the polygon of a predicted object whose most
 * probable label is in `enabled_labels`. Each object is first moved by its twist from
 * objects.header.stamp to `cloud_stamp`, so that it is compared at the time the points were taken.
 */
std::vector<PointCluster> remove_clusters_inside_objects(
  const std::vector<PointCluster> & clusters,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const std::vector<uint8_t> & enabled_labels, const builtin_interfaces::msg::Time & cloud_stamp);
}  // namespace autoware::control_validator::hazard

#endif  // HAZARD__POINTCLOUD_PREPROCESSOR_HPP_
