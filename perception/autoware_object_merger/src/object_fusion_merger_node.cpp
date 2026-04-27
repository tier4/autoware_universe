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

#include "autoware/object_merger/object_fusion_merger_node.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{
using DetectedObject = autoware_perception_msgs::msg::DetectedObject;
using Shape = autoware_perception_msgs::msg::Shape;
using MultiPoint2d = autoware_utils::MultiPoint2d;
using Point2d = autoware_utils::Point2d;
using Polygon2d = autoware_utils::Polygon2d;

void update_shape_height(
  DetectedObject & output, const DetectedObject & main_object, const DetectedObject & sub_object)
{
  const double output_center_z = output.kinematics.pose_with_covariance.pose.position.z;
  const double main_half_z = main_object.shape.dimensions.z * 0.5;
  const double sub_half_z = sub_object.shape.dimensions.z * 0.5;
  const double main_min_z =
    main_object.kinematics.pose_with_covariance.pose.position.z - main_half_z;
  const double main_max_z =
    main_object.kinematics.pose_with_covariance.pose.position.z + main_half_z;
  const double sub_min_z = sub_object.kinematics.pose_with_covariance.pose.position.z - sub_half_z;
  const double sub_max_z = sub_object.kinematics.pose_with_covariance.pose.position.z + sub_half_z;
  const double min_local_z = std::min(main_min_z, sub_min_z) - output_center_z;
  const double max_local_z = std::max(main_max_z, sub_max_z) - output_center_z;
  output.shape.dimensions.z = 2.0 * std::max(std::abs(min_local_z), std::abs(max_local_z));
}

MultiPoint2d collect_union_points_in_output_frame(
  const DetectedObject & output, const DetectedObject & main_object,
  const DetectedObject & sub_object)
{
  const auto append_local_polygon_points =
    [&output](const Polygon2d & polygon, MultiPoint2d & combined_points) {
      for (const auto & point : polygon.outer()) {
        geometry_msgs::msg::Point world_point;
        world_point.x = boost::geometry::get<0>(point);
        world_point.y = boost::geometry::get<1>(point);
        world_point.z = output.kinematics.pose_with_covariance.pose.position.z;
        const auto local_point = autoware_utils::inverse_transform_point(
          world_point, output.kinematics.pose_with_covariance.pose);
        combined_points.push_back(Point2d(local_point.x, local_point.y));
      }
    };

  const auto main_polygon = autoware_utils::to_polygon2d(main_object);
  const auto sub_polygon = autoware_utils::to_polygon2d(sub_object);
  MultiPoint2d combined_points;
  append_local_polygon_points(main_polygon, combined_points);
  append_local_polygon_points(sub_polygon, combined_points);
  return combined_points;
}

DetectedObject enclose_union_with_main_shape(
  const DetectedObject & main_object, const DetectedObject & sub_object)
{
  DetectedObject output = main_object;
  output.shape = main_object.shape;
  const auto combined_points =
    collect_union_points_in_output_frame(output, main_object, sub_object);
  if (combined_points.empty()) {
    return output;
  }

  if (main_object.shape.type == Shape::BOUNDING_BOX) {
    double max_abs_x = 0.0;
    double max_abs_y = 0.0;
    for (const auto & point : combined_points) {
      max_abs_x = std::max(max_abs_x, std::abs(boost::geometry::get<0>(point)));
      max_abs_y = std::max(max_abs_y, std::abs(boost::geometry::get<1>(point)));
    }
    output.shape.dimensions.x = 2.0 * max_abs_x;
    output.shape.dimensions.y = 2.0 * max_abs_y;
    update_shape_height(output, main_object, sub_object);
    return output;
  }

  if (main_object.shape.type == Shape::CYLINDER) {
    double max_radius = 0.0;
    for (const auto & point : combined_points) {
      max_radius = std::max(
        max_radius, std::hypot(boost::geometry::get<0>(point), boost::geometry::get<1>(point)));
    }
    output.shape.dimensions.x = 2.0 * max_radius;
    output.shape.dimensions.y = 2.0 * max_radius;
    update_shape_height(output, main_object, sub_object);
    return output;
  }

  if (main_object.shape.type == Shape::POLYGON) {
    Polygon2d hull_polygon;
    boost::geometry::convex_hull(combined_points, hull_polygon);
    output.shape.footprint.points.clear();
    for (const auto & point : hull_polygon.outer()) {
      output.shape.footprint.points.push_back(
        geometry_msgs::build<geometry_msgs::msg::Point32>()
          .x(static_cast<float>(boost::geometry::get<0>(point)))
          .y(static_cast<float>(boost::geometry::get<1>(point)))
          .z(0.0f));
    }
    update_shape_height(output, main_object, sub_object);
  }
  return output;
}

}  // namespace

namespace autoware::object_merger
{
ObjectFusionMergerNode::ObjectFusionMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("object_fusion_merger_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  main_object_sub_(this, "input/main_object", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sub_object_sub_(this, "input/sub_object", rclcpp::QoS{1}.get_rmw_qos_profile())
{
  // Get parameters for data association
  base_link_frame_id_ = declare_parameter<std::string>("base_link_frame_id");
  const auto sync_queue_size = static_cast<int>(declare_parameter<int64_t>("sync_queue_size"));

  const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());
  const auto max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
  const auto max_rad_matrix = this->declare_parameter<std::vector<double>>("max_rad_matrix");
  const auto min_iou_matrix = this->declare_parameter<std::vector<double>>("min_iou_matrix");
  data_association_ = std::make_unique<DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_rad_matrix, min_iou_matrix);

  // Set up publishers, subscribers, and synchronizer
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_ptr_ =
    std::make_shared<Sync>(SyncPolicy(sync_queue_size), main_object_sub_, sub_object_sub_);
  sync_ptr_->registerCallback(std::bind(&ObjectFusionMergerNode::callback, this, _1, _2));

  fused_object_pub_ = create_publisher<DetectedObjects>("output/object", rclcpp::QoS{1});

  processing_time_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, get_name());
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

void ObjectFusionMergerNode::callback(
  const DetectedObjects::ConstSharedPtr & main_objects_msg,
  const DetectedObjects::ConstSharedPtr & sub_objects_msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  if (fused_object_pub_->get_subscription_count() < 1) {
    return;
  }

  DetectedObjects transformed_main_objects;
  DetectedObjects transformed_sub_objects;
  if (
    !autoware::object_recognition_utils::transformObjects(
      *main_objects_msg, base_link_frame_id_, tf_buffer_, transformed_main_objects) ||
    !autoware::object_recognition_utils::transformObjects(
      *sub_objects_msg, base_link_frame_id_, tf_buffer_, transformed_sub_objects)) {
    RCLCPP_WARN(
      get_logger(), "Failed to transform objects to %s frame. Skipping fusion.",
      base_link_frame_id_.c_str());
    return;
  }

  const auto fused_objects_msg = fuse_objects(transformed_main_objects, transformed_sub_objects);
  fused_object_pub_->publish(fused_objects_msg);

  // Publish debug info
  published_time_publisher_->publish_if_subscribed(
    fused_object_pub_, fused_objects_msg.header.stamp);
  processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/cyclic_time_ms", stop_watch_ptr_->toc("cyclic_time", true));
  processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));
}

ObjectFusionMergerNode::DetectedObjects ObjectFusionMergerNode::fuse_objects(
  const DetectedObjects & main_objects_msg, const DetectedObjects & sub_objects_msg)
{
  // 1. Associate main and sub objects using the data association module
  std::unordered_map<int, int> direct_assignment;
  std::unordered_map<int, int> reverse_assignment;
  Eigen::MatrixXd score_matrix =
    data_association_->calcScoreMatrix(sub_objects_msg, main_objects_msg);
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

  const auto & main_objects = main_objects_msg.objects;
  const auto & sub_objects = sub_objects_msg.objects;

  // 2. Fuse matched objects and keep unmatched main objects as they are
  std::vector<DetectedObject> matched_objects;
  for (std::size_t main_index = 0; main_index < main_objects.size(); ++main_index) {
    const auto & main_object = main_objects.at(main_index);
    const auto direct_itr = direct_assignment.find(static_cast<int>(main_index));
    if (direct_itr == direct_assignment.end()) {
      // No matched sub object, keep the main object as it is
      matched_objects.push_back(main_object);
      continue;
    }
    matched_objects.push_back(
      enclose_union_with_main_shape(main_object, sub_objects.at(direct_itr->second)));
  }

  return autoware_perception_msgs::build<DetectedObjects>()
    .header(std_msgs::build<std_msgs::msg::Header>()
              .stamp(main_objects_msg.header.stamp)
              .frame_id(base_link_frame_id_))
    .objects(matched_objects);
}

}  // namespace autoware::object_merger

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::object_merger::ObjectFusionMergerNode)
