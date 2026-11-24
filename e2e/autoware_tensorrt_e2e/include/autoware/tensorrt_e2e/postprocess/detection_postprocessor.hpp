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

#ifndef AUTOWARE__TENSORRT_E2E__POSTPROCESS__DETECTION_POSTPROCESSOR_HPP_
#define AUTOWARE__TENSORRT_E2E__POSTPROCESS__DETECTION_POSTPROCESSOR_HPP_

#include <autoware/bevfusion/detection_class_remapper.hpp>
#include <autoware/bevfusion/postprocess/non_maximum_suppression.hpp>
#include <autoware/bevfusion/utils.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @class DetectionPostprocessor
 * @brief Turns the decoded, score-cut proposals into `DetectedObjects`.
 *
 * The device side (TransFusion coder, yaw-norm gate, per-(distance band, class) score
 * thresholds) is TrtBevFeatureExtractor's decode kernel, which hands over the survivors in
 * `autoware_bevfusion`'s order. What is left is that node's host sequence after its own
 * decode: class-agnostic circle NMS (on the host here, with the device kernel's float
 * arithmetic; see circle_nms()), one `DetectedObject` per box, BEV-IoU NMS, then the
 * area-based class remapper.
 *
 * Class mapping: `class_names[label]` is looked up with `autoware_bevfusion`'s
 * `getSemanticType`, so a name that ObjectClassification has no label for becomes
 * UNKNOWN. The model directory's ml_package file already writes those slots as
 * "UNKNOWN" (the gen2 head's `traffic_cone` and `barrier`), so the deployed
 * configuration states what is published rather than leaving it to be inferred.
 */
class DetectionPostprocessor
{
public:
  struct Config
  {
    //! One Autoware class name per head class, indexed by the head's label.
    std::vector<std::string> class_names;
    //! Class-agnostic centre-distance NMS [m]; 0 disables it.
    double circle_nms_dist_threshold{0.5};
    //! BEV-IoU NMS, as `autoware_bevfusion` parameterizes it.
    double iou_nms_search_distance_2d{10.0};
    double iou_nms_threshold{0.1};
    //! Area-based class remapping. All three empty disables the step.
    std::vector<int64_t> allow_remapping_by_area_matrix;
    std::vector<double> min_area_matrix;
    std::vector<double> max_area_matrix;
  };

  /**
   * @throws std::runtime_error on a negative NMS parameter, or when the remapper matrices
   *         are not a consistent square set.
   */
  explicit DetectionPostprocessor(const Config & config);

  /**
   * @brief Build the message for one frame's boxes.
   * @param boxes Score-cut proposals, score-descending, in the cloud's own frame.
   * @param header Header of the point cloud the boxes were detected in.
   */
  autoware_perception_msgs::msg::DetectedObjects build(
    const std::vector<autoware::bevfusion::Box3D> & boxes, const std_msgs::msg::Header & header);

private:
  Config config_;
  bool remap_classes_{false};
  autoware::bevfusion::NonMaximumSuppression iou_bev_nms_;
  autoware::bevfusion::DetectionClassRemapper class_remapper_;
};

/**
 * @brief `autoware_bevfusion`'s circle NMS over score-descending boxes: a box is dropped when
 *        its centre lies closer than `distance_threshold` to a kept, higher-scored box of any
 *        class. Float arithmetic, as the device kernel compares `dist2dPow < threshold^2`.
 */
std::vector<autoware::bevfusion::Box3D> circle_nms(
  const std::vector<autoware::bevfusion::Box3D> & boxes, float distance_threshold);

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__POSTPROCESS__DETECTION_POSTPROCESSOR_HPP_
