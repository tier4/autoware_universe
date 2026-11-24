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

#include "autoware/tensorrt_e2e/postprocess/detection_postprocessor.hpp"

#include <autoware/bevfusion/ros_utils.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace
{

//! A square matrix over ObjectClassification labels, the shape the remapper expects.
size_t square_side(const size_t elements, const std::string & name)
{
  const auto side = static_cast<size_t>(std::llround(std::sqrt(static_cast<double>(elements))));
  if (side * side != elements) {
    throw std::runtime_error(
      "bev_feature.detection." + name + " has " + std::to_string(elements) +
      " elements, which is not a square matrix");
  }
  return side;
}

}  // namespace

DetectionPostprocessor::DetectionPostprocessor(const Config & config) : config_(config)
{
  if (config_.class_names.empty()) {
    throw std::runtime_error("bev_feature.detection.class_names is empty");
  }
  if (config_.circle_nms_dist_threshold < 0.0) {
    throw std::runtime_error("bev_feature.detection.circle_nms_dist_threshold must be >= 0");
  }

  autoware::bevfusion::NMSParams nms_params;
  nms_params.search_distance_2d_ = config_.iou_nms_search_distance_2d;
  nms_params.iou_threshold_ = config_.iou_nms_threshold;
  if (nms_params.search_distance_2d_ < 0.0) {
    throw std::runtime_error("bev_feature.detection.iou_nms_search_distance_2d must be >= 0");
  }
  if (nms_params.iou_threshold_ < 0.0 || nms_params.iou_threshold_ > 1.0) {
    throw std::runtime_error("bev_feature.detection.iou_nms_threshold must be in [0, 1]");
  }
  iou_bev_nms_.setParameters(nms_params);

  const bool any_matrix = !config_.allow_remapping_by_area_matrix.empty() ||
                          !config_.min_area_matrix.empty() || !config_.max_area_matrix.empty();
  if (any_matrix) {
    // The matrices are indexed by ObjectClassification label, not by head class, so their
    // side is the label count and has nothing to do with class_names.size().
    const size_t side = square_side(
      config_.allow_remapping_by_area_matrix.size(), "allow_remapping_by_area_matrix");
    if (
      square_side(config_.min_area_matrix.size(), "min_area_matrix") != side ||
      square_side(config_.max_area_matrix.size(), "max_area_matrix") != side) {
      throw std::runtime_error(
        "the detection class remapper matrices have different sizes; all three describe the "
        "same label-by-label table");
    }
    class_remapper_.setParameters(
      config_.allow_remapping_by_area_matrix, config_.min_area_matrix, config_.max_area_matrix);
    remap_classes_ = true;
  }
}

std::vector<autoware::bevfusion::Box3D> circle_nms(
  const std::vector<autoware::bevfusion::Box3D> & boxes, const float distance_threshold)
{
  if (!(distance_threshold > 0.f)) {
    return boxes;
  }
  const float threshold_squared = distance_threshold * distance_threshold;
  std::vector<autoware::bevfusion::Box3D> kept;
  kept.reserve(boxes.size());
  std::vector<bool> suppressed(boxes.size(), false);
  for (size_t i = 0; i < boxes.size(); ++i) {
    if (suppressed[i]) {
      continue;
    }
    kept.push_back(boxes[i]);
    for (size_t j = i + 1; j < boxes.size(); ++j) {
      const float dx = boxes[i].x - boxes[j].x;
      const float dy = boxes[i].y - boxes[j].y;
      if (dx * dx + dy * dy < threshold_squared) {
        suppressed[j] = true;
      }
    }
  }
  return kept;
}

autoware_perception_msgs::msg::DetectedObjects DetectionPostprocessor::build(
  const std::vector<autoware::bevfusion::Box3D> & boxes, const std_msgs::msg::Header & header)
{
  const auto kept = circle_nms(boxes, static_cast<float>(config_.circle_nms_dist_threshold));
  std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
  raw_objects.reserve(kept.size());
  for (const auto & box : kept) {
    autoware_perception_msgs::msg::DetectedObject object;
    autoware::bevfusion::box3DToDetectedObject(box, config_.class_names, object);
    raw_objects.emplace_back(std::move(object));
  }

  autoware_perception_msgs::msg::DetectedObjects message;
  message.header = header;
  message.objects = iou_bev_nms_.apply(raw_objects);
  if (remap_classes_) {
    class_remapper_.mapClasses(message);
  }
  return message;
}

}  // namespace autoware::tensorrt_e2e
