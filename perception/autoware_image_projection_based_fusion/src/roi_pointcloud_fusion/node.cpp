// Copyright 2023 TIER IV, Inc.
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

#include "autoware/image_projection_based_fusion/roi_pointcloud_fusion/node.hpp"

#include "autoware/image_projection_based_fusion/utils/geometry.hpp"
#include "autoware/image_projection_based_fusion/utils/utils.hpp"

#include <autoware_utils/system/time_keeper.hpp>

#include <algorithm>
#include <memory>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include "autoware/euclidean_cluster/utils.hpp"

namespace autoware::image_projection_based_fusion
{
using autoware_utils::ScopedTimeTrack;
using Classification = autoware_perception_msgs::msg::ObjectClassification;

RoiPointCloudFusionNode::RoiPointCloudFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<PointCloudMsgType, RoiMsgType, ClusterMsgType>("roi_pointcloud_fusion", options)
{
  fuse_unknown_only_ = declare_parameter<bool>("fuse_unknown_only");
  min_cluster_size_ = declare_parameter<int>("min_cluster_size");
  max_cluster_size_ = declare_parameter<int>("max_cluster_size");
  cluster_2d_tolerance_ = declare_parameter<double>("cluster_2d_tolerance");
  roi_scale_factor_ = declare_parameter<double>("roi_scale_factor");
  override_class_with_unknown_ = declare_parameter<bool>("override_class_with_unknown");
  max_object_size_ = declare_parameter<double>("max_object_size");

  // publisher
  pub_ptr_ = this->create_publisher<ClusterMsgType>("output", rclcpp::QoS{1});
  cluster_debug_pub_ = this->create_publisher<PointCloudMsgType>("debug/clusters", 1);
}

void RoiPointCloudFusionNode::fuseOnSingleImage(
  const PointCloudMsgType & input_pointcloud_msg, const Det2dStatus<RoiMsgType> & det2d,
  const RoiMsgType & input_roi_msg,
  __attribute__((unused)) PointCloudMsgType & output_pointcloud_msg)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (input_pointcloud_msg.data.empty()) {
    return;
  }

  std::vector<DetectedObjectWithFeature> output_objs;
  std::vector<float> output_objs_distances;
  std::vector<sensor_msgs::msg::RegionOfInterest> debug_image_rois;
  std::vector<Eigen::Vector2d> debug_image_points;
  // select ROIs for fusion
  double min_distance = -1.0;
  for (const auto & feature_obj : input_roi_msg.feature_objects) {
    // add calculate object rough distance
    if (fuse_unknown_only_) {
      bool is_roi_label_unknown =
        feature_obj.object.classification.front().label == Classification::UNKNOWN;
      if (is_roi_label_unknown) {
        output_objs.push_back(feature_obj);
        const auto & roi = feature_obj.feature.roi;
        min_distance =
          pixelTo3DPoint(roi, det2d.camera2lidar_mul_inv_projection_, margin_distance_);
        output_objs_distances.push_back(min_distance);
        debug_image_rois.push_back(feature_obj.feature.roi);
      }
    } else {
      // TODO(badai-nguyen): selected class from a list
      if (override_class_with_unknown_) {
        auto feature_obj_remap = feature_obj;
        feature_obj_remap.object.classification.front().label = Classification::UNKNOWN;
        output_objs.push_back(feature_obj_remap);
        const auto & roi = feature_obj_remap.feature.roi;
        min_distance =
          pixelTo3DPoint(roi, det2d.camera2lidar_mul_inv_projection_, margin_distance_);
        output_objs_distances.push_back(min_distance);
      } else {
        output_objs.push_back(feature_obj);
        const auto & roi = feature_obj.feature.roi;
        min_distance =
          pixelTo3DPoint(roi, det2d.camera2lidar_mul_inv_projection_, margin_distance_);
        output_objs_distances.push_back(min_distance);
      }
      debug_image_rois.push_back(feature_obj.feature.roi);
    }
  }

  // check if there is no object to fuse
  if (output_objs.empty()) {
    // publish debug image, which is empty
    if (debugger_) {
      debugger_->image_rois_ = debug_image_rois;
      debugger_->obstacle_points_ = debug_image_points;
      debugger_->publishImage(det2d.id, input_roi_msg.header.stamp);
    }
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, input_roi_msg.header.frame_id, input_pointcloud_msg.header.frame_id,
      input_roi_msg.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }
  const int point_step = input_pointcloud_msg.point_step;
  const int x_offset =
    input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "x")].offset;
  const int y_offset =
    input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "y")].offset;
  const int z_offset =
    input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "z")].offset;

  PointCloudMsgType transformed_cloud;
  tf2::doTransform(input_pointcloud_msg, transformed_cloud, transform_stamped);

  std::vector<PointCloudMsgType> clusters;
  clusters.resize(output_objs.size());
  for (auto & cluster : clusters) {
    cluster.point_step = input_pointcloud_msg.point_step;
    cluster.height = input_pointcloud_msg.height;
    cluster.fields = input_pointcloud_msg.fields;
    cluster.data.reserve(input_pointcloud_msg.data.size());
  }
  for (size_t offset = 0; offset < input_pointcloud_msg.data.size(); offset += point_step) {
    const float transformed_x =
      *reinterpret_cast<const float *>(&transformed_cloud.data[offset + x_offset]);
    const float transformed_y =
      *reinterpret_cast<const float *>(&transformed_cloud.data[offset + y_offset]);
    const float transformed_z =
      *reinterpret_cast<const float *>(&transformed_cloud.data[offset + z_offset]);
    if (transformed_z <= 0.0) {
      continue;
    }
    const float origin_x =
      *reinterpret_cast<const float *>(&input_pointcloud_msg.data[offset + x_offset]);
    const float origin_y =
      *reinterpret_cast<const float *>(&input_pointcloud_msg.data[offset + y_offset]);
    Eigen::Vector2d projected_point;
    if (det2d.camera_projector_ptr->calcImageProjectedPoint(
          cv::Point3d(transformed_x, transformed_y, transformed_z), projected_point)) {
      for (std::size_t i = 0; i < output_objs.size(); ++i) {
        // check the distance to the object
        const double min_distance = output_objs_distances.at(i);
        double distance = std::hypot(origin_x, origin_y);
        auto & feature_obj = output_objs.at(i);
        const auto & check_roi = feature_obj.feature.roi;
        auto & cluster = clusters.at(i);

        const double px = projected_point.x();
        const double py = projected_point.y();

        // is not correct to skip fusion and keep a cluster with a partial of points
        if (isPointInsideRoi(check_roi, px, py, roi_scale_factor_)) {
          // append point data to clusters data vector
          cluster.data.insert(
            cluster.data.end(), &input_pointcloud_msg.data[offset],
            &input_pointcloud_msg.data[offset + point_step]);
        }
      }
      if (debugger_) {
        // add all points inside image to debug
        debug_image_points.push_back(projected_point);
      }
    }
  }

  // refine and update output_fused_objects_
  updateOutputFusedObjects(
    output_objs, clusters, input_pointcloud_msg, input_roi_msg.header, tf_buffer_,
    min_cluster_size_, max_cluster_size_, cluster_2d_tolerance_, max_object_size_,
    output_fused_objects_);

  // publish debug image
  if (debugger_) {
    debugger_->image_rois_ = debug_image_rois;
    debugger_->obstacle_points_ = debug_image_points;
    debugger_->publishImage(det2d.id, input_roi_msg.header.stamp);
  }
}

void RoiPointCloudFusionNode::postprocess(
  const PointCloudMsgType & pointcloud_msg, ClusterMsgType & output_msg)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  output_msg.header = pointcloud_msg.header;
  output_msg.feature_objects = output_fused_objects_;

  output_fused_objects_.clear();

  // publish debug cluster
  if (cluster_debug_pub_->get_subscription_count() > 0) {
    PointCloudMsgType debug_cluster_msg;
    autoware::euclidean_cluster::convertObjectMsg2SensorMsg(output_msg, debug_cluster_msg);
    cluster_debug_pub_->publish(debug_cluster_msg);
  }
}

double RoiPointCloudFusionNode::pixelTo3DPoint(
  const sensor_msgs::msg::RegionOfInterest & roi, const Eigen::Matrix4f & transform,
  const double & margin_distance)
{
  const double bottom_middle_y = roi.y_offset + roi.height;
  const double bottom_middle_x = roi.x_offset + roi.width / 2.0;
  auto w_div_projected_z =
    -(transform(2, 0) * bottom_middle_x + transform(2, 1) * bottom_middle_y + transform(2, 2)) /
    transform(2, 3);
  auto projected_z = 1.0 / (transform(3, 0) * bottom_middle_x + transform(3, 1) * bottom_middle_y +
                            transform(3, 2) + transform(3, 3) * w_div_projected_z);
  auto w = w_div_projected_z * projected_z;
  Eigen::Vector4f projected_point =
    Eigen::Vector4f(bottom_middle_x * projected_z, bottom_middle_y * projected_z, projected_z, w);
  Eigen::Vector4f point = transform * projected_point;
  return sqrt(pow(point(0), 2) + pow(point(1), 2) + pow(point(2), 2)) - margin_distance;
}

void RoiPointCloudFusionNode::publish(const ClusterMsgType & output_msg)
{
  const auto objects_sub_count =
    pub_ptr_->get_subscription_count() + pub_ptr_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }
  pub_ptr_->publish(output_msg);
}
}  // namespace autoware::image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_projection_based_fusion::RoiPointCloudFusionNode)
