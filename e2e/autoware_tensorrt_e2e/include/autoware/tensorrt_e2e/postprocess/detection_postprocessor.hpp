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
 * @brief Turns the extractor's decoded proposals into `DetectedObjects`.
 *
 * The device side of the postprocessing (TransFusion bbox coder, yaw-norm gate, score
 * cut, circle NMS) is `autoware_bevfusion`'s `PostprocessCuda` and runs inside
 * `TrtBevFeatureExtractor`. What is left is the host side, and it is the same sequence
 * `autoware_bevfusion`'s node runs after its own `detect()`: build one `DetectedObject`
 * per box, suppress overlaps with BEV-IoU NMS, then remap classes by area.
 *
 * The one step that is this package's own is the per-class score cut. The head's bbox
 * coder states a threshold PER CLASS and `PostprocessCuda` holds a single float, so the
 * device filter runs at the lowest of them and the per-class cut is applied here.
 * Together they are exactly the head's own thresholds.
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
    //! One score threshold per head class; empty keeps every box the device kept.
    std::vector<double> score_thresholds;
    //! BEV-IoU NMS, as `autoware_bevfusion` parameterizes it.
    double iou_nms_search_distance_2d{10.0};
    double iou_nms_threshold{0.1};
    //! Area-based class remapping. All three empty disables the step.
    std::vector<int64_t> allow_remapping_by_area_matrix;
    std::vector<double> min_area_matrix;
    std::vector<double> max_area_matrix;
  };

  /**
   * @throws std::runtime_error when the class names and thresholds disagree in length,
   *         or when the remapper matrices are not a consistent square set.
   */
  explicit DetectionPostprocessor(const Config & config);

  /**
   * @brief Build the message for one frame's boxes.
   * @param boxes Decoded proposals, score-descending, in the cloud's own frame.
   * @param header Header of the point cloud the boxes were detected in.
   */
  autoware_perception_msgs::msg::DetectedObjects build(
    const std::vector<autoware::bevfusion::Box3D> & boxes, const std_msgs::msg::Header & header);

  /// Boxes dropped by the per-class score cut in the last build().
  size_t last_below_threshold() const { return last_below_threshold_; }

private:
  Config config_;
  bool remap_classes_{false};
  autoware::bevfusion::NonMaximumSuppression iou_bev_nms_;
  autoware::bevfusion::DetectionClassRemapper class_remapper_;
  size_t last_below_threshold_{0};
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__POSTPROCESS__DETECTION_POSTPROCESSOR_HPP_
