// Copyright 2023-2026 the Autoware Foundation
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
//

#ifndef OCCLUSION_PREDICTOR_HPP_
#define OCCLUSION_PREDICTOR_HPP_

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/pcl_conversion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp>  // for ROS 2 Jazzy or newer
#else
#include <image_geometry/pinhole_camera_model.h>  // for ROS 2 Humble or older
#endif

#include <lanelet2_core/Forward.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace autoware::traffic_light
{

struct Ray
{
  float azimuth;
  float elevation;
  float dist;
};

inline Ray point2ray(const pcl::PointXYZ & pt)
{
  Ray ray;
  ray.dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
  ray.elevation = RAD2DEG(std::atan2(pt.y, std::hypot(pt.x, pt.z)));
  ray.azimuth = RAD2DEG(std::atan2(pt.x, pt.z));
  return ray;
}

class CloudOcclusionPredictor
{
public:
  CloudOcclusionPredictor(
    rclcpp::Logger logger, float max_valid_pt_distance, float azimuth_occlusion_resolution_deg,
    float elevation_occlusion_resolution_deg);

  template <typename BufferT>
  void predict(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & rois_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const BufferT & tf_buffer,
    const std::map<lanelet::Id, tf2::Vector3> & traffic_light_position_map,
    std::vector<int> & occlusion_ratios);

private:
  uint32_t predict(const pcl::PointXYZ & roi_top_left, const pcl::PointXYZ & roi_bottom_right);

  void filterCloud(
    const pcl::PointCloud<pcl::PointXYZ> & cloud_in, const std::vector<pcl::PointXYZ> & roi_tls,
    const std::vector<pcl::PointXYZ> & roi_brs, pcl::PointCloud<pcl::PointXYZ> & cloud_out) const;

  static void sampleTrafficLightRoi(
    const pcl::PointXYZ & top_left, const pcl::PointXYZ & bottom_right,
    uint32_t horizontal_sample_num, uint32_t vertical_sample_num,
    pcl::PointCloud<pcl::PointXYZ> & cloud_out);

  static void calcRoiVector3D(
    const tier4_perception_msgs::msg::TrafficLightRoi & roi,
    const image_geometry::PinholeCameraModel & pinhole_model,
    const std::map<lanelet::Id, tf2::Vector3> & traffic_light_position_map,
    const tf2::Transform & tf_camera2map, pcl::PointXYZ & top_left, pcl::PointXYZ & bottom_right);

  std::map<int, std::map<int, std::vector<Ray>>> lidar_rays_;
  rclcpp::Logger logger_;
  float max_valid_pt_distance_;
  float azimuth_occlusion_resolution_deg_;
  float elevation_occlusion_resolution_deg_;
};

// Template method implementation (must be in header for template instantiation)
template <typename BufferT>
void CloudOcclusionPredictor::predict(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & rois_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const BufferT & tf_buffer,
  const std::map<lanelet::Id, tf2::Vector3> & traffic_light_position_map,
  std::vector<int> & occlusion_ratios)
{
  if (camera_info_msg == nullptr || rois_msg == nullptr || cloud_msg == nullptr) {
    return;
  }
  occlusion_ratios.resize(rois_msg->rois.size());
  // get transformation from cloud to camera
  Eigen::Matrix4d camera2cloud;
  try {
    camera2cloud =
      tf2::transformToEigen(tf_buffer.lookupTransform(
                              camera_info_msg->header.frame_id, cloud_msg->header.frame_id,
                              rclcpp::Time(camera_info_msg->header.stamp),
                              rclcpp::Duration::from_seconds(0.2)))
        .matrix();
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      logger_, "Error: cannot get transform from << " << camera_info_msg->header.frame_id << " to "
                                                      << cloud_msg->header.frame_id);
    return;
  }
  // get transformation from map to camera
  tf2::Transform tf_camera2map;
  try {
    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer.lookupTransform(
      camera_info_msg->header.frame_id, "map", rclcpp::Time(camera_info_msg->header.stamp),
      rclcpp::Duration::from_seconds(0.2));
    tf2::fromMsg(tf_stamped.transform, tf_camera2map);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Error: cannot get transform from << " << camera_info_msg->header.frame_id << " to map");
    return;
  }

  std::vector<pcl::PointXYZ> roi_tls(rois_msg->rois.size()), roi_brs(rois_msg->rois.size());
  image_geometry::PinholeCameraModel pinhole_model;
  pinhole_model.fromCameraInfo(*camera_info_msg);
  for (size_t i = 0; i < rois_msg->rois.size(); i++) {
    // skip if no detection
    if (rois_msg->rois[i].roi.height == 0) {
      continue;
    }
    calcRoiVector3D(
      rois_msg->rois[i], pinhole_model, traffic_light_position_map, tf_camera2map, roi_tls[i],
      roi_brs[i]);
  }

  lidar_rays_.clear();
  // points in camera frame
  pcl::PointCloud<pcl::PointXYZ> cloud_camera;
  // points within roi
  pcl::PointCloud<pcl::PointXYZ> cloud_roi;
  autoware_utils::transform_point_cloud_from_ros_msg(*cloud_msg, cloud_camera, camera2cloud);

  filterCloud(cloud_camera, roi_tls, roi_brs, cloud_roi);

  for (const pcl::PointXYZ & pt : cloud_roi) {
    Ray ray = point2ray(pt);
    lidar_rays_[static_cast<int>(ray.azimuth)][static_cast<int>(ray.elevation)].push_back(ray);
  }
  for (size_t i = 0; i < roi_tls.size(); i++) {
    occlusion_ratios[i] = rois_msg->rois[i].roi.height == 0 ? 0 : predict(roi_tls[i], roi_brs[i]);
  }
}

}  // namespace autoware::traffic_light

#endif  // OCCLUSION_PREDICTOR_HPP_
