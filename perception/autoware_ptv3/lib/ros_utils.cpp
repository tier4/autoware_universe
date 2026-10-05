// Copyright 2025 TIER IV, Inc.
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

#include "autoware/ptv3/ros_utils.hpp"

#include <Eigen/Geometry>
#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ptv3
{

using Label = autoware_perception_msgs::msg::ObjectClassification;

void box3d_to_detected_object(
  const Box3D & box3d, const std::vector<std::string> & class_names, const bool has_twist,
  autoware_perception_msgs::msg::DetectedObject & obj)
{
  obj.existence_probability = box3d.score;

  Label classification;
  classification.probability = 1.0F;
  if (box3d.label >= 0 && static_cast<std::size_t>(box3d.label) < class_names.size()) {
    classification.label = get_classification_type(class_names[box3d.label]);
  } else {
    classification.label = Label::UNKNOWN;
  }

  if (autoware::object_recognition_utils::isCarLikeVehicle(classification.label)) {
    obj.kinematics.orientation_availability =
      autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
  }

  obj.classification.emplace_back(classification);

  const float yaw = box3d.yaw;
  obj.kinematics.pose_with_covariance.pose.position =
    autoware_utils::create_point(box3d.x, box3d.y, box3d.z);
  obj.kinematics.pose_with_covariance.pose.orientation =
    autoware_utils::create_quaternion_from_yaw(yaw);
  obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions =
    autoware_utils::create_translation(box3d.length, box3d.width, box3d.height);

  if (has_twist) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = std::cos(yaw) * box3d.vel_x + std::sin(yaw) * box3d.vel_y;
    twist.linear.y = -std::sin(yaw) * box3d.vel_x + std::cos(yaw) * box3d.vel_y;
    obj.kinematics.twist_with_covariance.twist = twist;
    obj.kinematics.has_twist = true;
  }
}

std::uint8_t get_classification_type(const std::string & class_name)
{
  if (class_name == "CAR") {
    return Label::CAR;
  }
  if (class_name == "TRUCK") {
    return Label::TRUCK;
  }
  if (class_name == "BUS") {
    return Label::BUS;
  }
  if (class_name == "TRAILER") {
    return Label::TRAILER;
  }
  if (class_name == "MOTORBIKE" || class_name == "MOTORCYCLE") {
    return Label::MOTORCYCLE;
  }
  if (class_name == "BICYCLE") {
    return Label::BICYCLE;
  }
  if (class_name == "PEDESTRIAN") {
    return Label::PEDESTRIAN;
  }
  if (class_name == "ANIMAL") {
    return Label::ANIMAL;
  }
  if (class_name == "TRAFFIC_CONE") {
    return Label::HAZARD;
  }
  if (class_name == "BARRIER") {
    return Label::HAZARD;
  }
  if (class_name == "DEBRIS") {
    return Label::HAZARD;
  }
  // Autoware has no train label, a train is published as an unknown object with its box.
  if (class_name == "TRAIN") {
    return Label::UNKNOWN;
  }
  return Label::UNKNOWN;
}

std::unordered_map<std::string, std::string> declare_class_mapping(
  rclcpp::Node & node, const std::vector<std::string> & class_names,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  std::unordered_map<std::string, std::string> class_mapping;
  class_mapping.reserve(class_names.size());

  // The mapping keys live in ptv3.param.yaml while class_names comes from the ml_package parameter
  // file, so report which one is out of sync when an entry is missing.
  std::optional<std::string> missing_classes;
  for (const auto & class_name : class_names) {
    const std::string param_name = "segmentation3d.class_mapping." + class_name;
    const auto mapped_class =
      node.declare_parameter<std::string>(param_name, std::string{}, descriptor);
    if (!mapped_class.empty()) {
      class_mapping.emplace(class_name, mapped_class);
      continue;
    }

    if (missing_classes) {
      *missing_classes += ", " + class_name;
    } else {
      missing_classes = class_name;
    }
  }

  if (missing_classes) {
    throw std::runtime_error(
      "segmentation3d.class_mapping is missing entries for segmentation3d.class_names: [" +
      *missing_classes + "].");
  }
  return class_mapping;
}

std::unordered_map<std::uint8_t, BboxMargins> declare_bbox_adjustment(
  rclcpp::Node & node, const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  // Every published label needs margins rather than a list of adjusted classes, because rclcpp
  // rejects an empty list in a parameter file, so no class could be left unadjusted.
  constexpr std::array<std::uint8_t, 10> labels{
    Label::UNKNOWN,    Label::CAR,     Label::TRUCK,      Label::BUS,    Label::TRAILER,
    Label::MOTORCYCLE, Label::BICYCLE, Label::PEDESTRIAN, Label::ANIMAL, Label::HAZARD};

  std::unordered_map<std::uint8_t, BboxMargins> margins_by_label;
  margins_by_label.reserve(labels.size());
  for (const auto label : labels) {
    const std::string param_name = "detection3d.post_process_params.bbox_adjustment.margins." +
                                   autoware::object_recognition_utils::convertLabelToString(label);
    const auto values = node.declare_parameter<std::vector<double>>(param_name, descriptor);
    BboxMargins margins{};
    if (values.size() != margins.size()) {
      throw std::runtime_error(param_name + " must contain 6 values [-x, -y, -z, x, y, z].");
    }
    if (!std::all_of(
          values.begin(), values.end(), [](double value) { return std::isfinite(value); })) {
      throw std::runtime_error(param_name + " values must be finite.");
    }
    std::copy(values.begin(), values.end(), margins.begin());
    margins_by_label.emplace(label, margins);
  }
  return margins_by_label;
}

void adjust_bbox(
  const std::unordered_map<std::uint8_t, BboxMargins> & margins_by_label,
  autoware_perception_msgs::msg::DetectedObject & obj)
{
  if (obj.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return;
  }
  const auto it = margins_by_label.find(
    autoware::object_recognition_utils::getHighestProbLabel(obj.classification));
  if (it == margins_by_label.end()) {
    return;
  }
  const auto & margins = it->second;

  auto & dimensions = obj.shape.dimensions;
  const std::array<double *, 3> sizes{&dimensions.x, &dimensions.y, &dimensions.z};
  Eigen::Vector3d center_shift;
  for (std::size_t axis = 0; axis < sizes.size(); ++axis) {
    const double min_face_margin = margins[axis];
    const double max_face_margin = margins[axis + 3];
    const double growth = min_face_margin + max_face_margin;
    double & size = *sizes[axis];
    // Both faces keep their share of a shrink that is cut short at the minimum dimension.
    double scale = 1.0;
    if (growth < 0.0 && size + growth < min_bbox_dimension) {
      scale = std::max(0.0, (min_bbox_dimension - size) / growth);
    }
    size += scale * growth;
    center_shift[static_cast<Eigen::Index>(axis)] =
      0.5 * scale * (max_face_margin - min_face_margin);
  }

  auto & pose = obj.kinematics.pose_with_covariance.pose;
  const Eigen::Quaterniond orientation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const Eigen::Vector3d position_shift = orientation * center_shift;
  pose.position.x += position_shift.x();
  pose.position.y += position_shift.y();
  pose.position.z += position_shift.z();
}

}  // namespace autoware::ptv3
