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

#include "occlusion_predictor.hpp"

#include <algorithm>
#include <map>
#include <vector>

namespace autoware::traffic_light
{

CloudOcclusionPredictor::CloudOcclusionPredictor(
  rclcpp::Logger logger, float max_valid_pt_distance, float azimuth_occlusion_resolution_deg,
  float elevation_occlusion_resolution_deg)
: logger_(logger),
  max_valid_pt_distance_(max_valid_pt_distance),
  azimuth_occlusion_resolution_deg_(azimuth_occlusion_resolution_deg),
  elevation_occlusion_resolution_deg_(elevation_occlusion_resolution_deg)
{
}

void CloudOcclusionPredictor::calcRoiVector3D(
  const tier4_perception_msgs::msg::TrafficLightRoi & roi,
  const image_geometry::PinholeCameraModel & pinhole_model,
  const std::map<lanelet::Id, tf2::Vector3> & traffic_light_position_map,
  const tf2::Transform & tf_camera2map, pcl::PointXYZ & top_left, pcl::PointXYZ & bottom_right)
{
  if (traffic_light_position_map.count(roi.traffic_light_id) == 0) {
    return;
  }
  double dist2cam = (tf_camera2map * traffic_light_position_map.at(roi.traffic_light_id)).length();
  {
    cv::Point2d pixel(roi.roi.x_offset, roi.roi.y_offset);
    cv::Point2d rectified_pixel = pinhole_model.rectifyPoint(pixel);
    cv::Point3d ray = pinhole_model.projectPixelTo3dRay(rectified_pixel);
    double ray_len = std::sqrt(ray.ddot(ray));
    top_left.x = dist2cam * ray.x / ray_len;
    top_left.y = dist2cam * ray.y / ray_len;
    top_left.z = dist2cam * ray.z / ray_len;
  }
  {
    cv::Point2d pixel(roi.roi.x_offset + roi.roi.width, roi.roi.y_offset + roi.roi.height);
    cv::Point2d rectified_pixel = pinhole_model.rectifyPoint(pixel);
    cv::Point3d ray = pinhole_model.projectPixelTo3dRay(rectified_pixel);
    double ray_len = std::sqrt(ray.ddot(ray));
    bottom_right.x = dist2cam * ray.x / ray_len;
    bottom_right.y = dist2cam * ray.y / ray_len;
    bottom_right.z = dist2cam * ray.z / ray_len;
  }
}

void CloudOcclusionPredictor::filterCloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in, const std::vector<pcl::PointXYZ> & roi_tls,
  const std::vector<pcl::PointXYZ> & roi_brs, pcl::PointCloud<pcl::PointXYZ> & cloud_out) const
{
  float min_x = 0, max_x = 0, min_y = 0, max_y = 0, min_z = 0, max_z = 0;
  for (const auto & pt : roi_tls) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
    min_z = std::min(min_z, pt.z);
    max_z = std::max(max_z, pt.z);
  }
  for (const auto & pt : roi_brs) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
    min_z = std::min(min_z, pt.z);
    max_z = std::max(max_z, pt.z);
  }
  const float min_dist_to_cam = 1.0f;
  cloud_out.clear();
  for (const auto & pt : cloud_in) {
    if (
      pt.x < min_x || pt.x > max_x || pt.y < min_y || pt.y > max_y || pt.z < min_z ||
      pt.z > max_z) {
      continue;
    }
    float dist = pt.getVector3fMap().squaredNorm();
    if (
      dist <= min_dist_to_cam * min_dist_to_cam ||
      dist >= max_valid_pt_distance_ * max_valid_pt_distance_) {
      continue;
    }
    cloud_out.push_back(pt);
  }
}

void CloudOcclusionPredictor::sampleTrafficLightRoi(
  const pcl::PointXYZ & top_left, const pcl::PointXYZ & bottom_right,
  uint32_t horizontal_sample_num, uint32_t vertical_sample_num,
  pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
  cloud_out.clear();
  float x1 = top_left.x;
  float y1 = top_left.y;
  float z1 = top_left.z;
  float x2 = bottom_right.x;
  float y2 = bottom_right.y;
  float z2 = bottom_right.z;
  for (uint32_t i1 = 0; i1 < horizontal_sample_num; i1++) {
    for (uint32_t i2 = 0; i2 < vertical_sample_num; i2++) {
      float x = x1 + (x2 - x1) * i1 / (horizontal_sample_num - 1);
      float y = y1 + (y2 - y1) * i2 / (vertical_sample_num - 1);
      float z = z1 + (z2 - z1) * i1 / (horizontal_sample_num - 1);
      cloud_out.push_back(pcl::PointXYZ(x, y, z));
    }
  }
}

uint32_t CloudOcclusionPredictor::predict(
  const pcl::PointXYZ & roi_top_left, const pcl::PointXYZ & roi_bottom_right)
{
  const uint32_t horizontal_sample_num = 20;
  const uint32_t vertical_sample_num = 20;
  static_assert(horizontal_sample_num > 1);
  static_assert(vertical_sample_num > 1);

  const float min_dist_from_occlusion_to_tl = 5.0f;

  pcl::PointCloud<pcl::PointXYZ> tl_sample_cloud;
  sampleTrafficLightRoi(
    roi_top_left, roi_bottom_right, horizontal_sample_num, vertical_sample_num, tl_sample_cloud);
  uint32_t occluded_num = 0;
  for (const pcl::PointXYZ & tl_pt : tl_sample_cloud) {
    Ray tl_ray = point2ray(tl_pt);
    bool occluded = false;
    // the azimuth and elevation range to search for points that may occlude tl_pt
    int min_azimuth = static_cast<int>(tl_ray.azimuth - azimuth_occlusion_resolution_deg_);
    int max_azimuth = static_cast<int>(tl_ray.azimuth + azimuth_occlusion_resolution_deg_);
    int min_elevation = static_cast<int>(tl_ray.elevation - elevation_occlusion_resolution_deg_);
    int max_elevation = static_cast<int>(tl_ray.elevation + elevation_occlusion_resolution_deg_);
    for (int azimuth = min_azimuth; (azimuth <= max_azimuth) && !occluded; azimuth++) {
      for (int elevation = min_elevation; (elevation <= max_elevation) && !occluded; elevation++) {
        for (const Ray & lidar_ray : lidar_rays_[azimuth][elevation]) {
          if (
            std::abs(lidar_ray.azimuth - tl_ray.azimuth) <= azimuth_occlusion_resolution_deg_ &&
            std::abs(lidar_ray.elevation - tl_ray.elevation) <=
              elevation_occlusion_resolution_deg_ &&
            lidar_ray.dist < tl_ray.dist - min_dist_from_occlusion_to_tl) {
            occluded = true;
            break;
          }
        }
      }
    }
    occluded_num += occluded;
  }
  return 100 * occluded_num / tl_sample_cloud.size();
}

}  // namespace autoware::traffic_light
