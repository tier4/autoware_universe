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

#ifndef CLASSIFIER__COMLOPS_TLR_CLASSIFIER_HPP_
#define CLASSIFIER__COMLOPS_TLR_CLASSIFIER_HPP_

#include "classifier_interface.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/stream_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light
{

struct BBox {
  float x1{0.f}, y1{0.f};  ///< Top-left corner of the bounding box.
  float x2{0.f}, y2{0.f};  ///< Bottom-right corner of the bounding box.
};

struct KeypointInfo {
  float x{0.f}, y{0.f};
};

/**
 * @brief Contains information about a detected object, including its bounding box,
 * label, class ID, and the probability of the detection.
 */
struct BBoxInfo {
  BBox box;                                    ///< Bounding box of the detected object.
  int label{0};                                ///< Label of the detected object.
  int classId{0};                              ///< Class ID (e.g. type: circle, arrow, ...).
  float prob{0.f};                             ///< Probability of the detection.
  bool isHierarchical{false};                  ///< True if subClassId (e.g. color) is used.
  int subClassId{0};                           ///< Sub-class ID (e.g. color: red, green, ...).
  float sin{0.f};                              ///< Sine of angle, if applicable.
  float cos{1.f};                              ///< Cosine of angle, if applicable.
  std::vector<KeypointInfo> keypoint;          ///< Associated keypoints (empty for TLR).
};

using autoware::cuda_utils::CudaUniquePtr;
using autoware::cuda_utils::CudaUniquePtrHost;
using autoware::cuda_utils::makeCudaStream;
using autoware::cuda_utils::StreamUniquePtr;

/**
 * @brief Traffic light classifier using CoMLOps-TLR ONNX model.
 */
class CoMLOpsTLRClassifier : public ClassifierInterface
{
public:
  explicit CoMLOpsTLRClassifier(rclcpp::Node * node_ptr);
  ~CoMLOpsTLRClassifier() override = default;

  bool getTrafficSignals(
    const std::vector<cv::Mat> & images,
    tier4_perception_msgs::msg::TrafficLightArray & traffic_signals) override;

private:
  void preprocess(const std::vector<cv::Mat> & images);
  bool doInference(size_t batch_size);
  /** Decode all raw detections, filter by score, run NMS; output per-ROI BBoxInfo list. */
  void decodeTlrOutput(size_t batch_size, std::vector<std::vector<BBoxInfo>> & detections_per_roi);
  void toTrafficLightElements(
    int color_index, int type_index, float confidence, float angle_rad,
    tier4_perception_msgs::msg::TrafficLight & traffic_signal);
  void outputDebugImage(
    cv::Mat & debug_image, const tier4_perception_msgs::msg::TrafficLight & traffic_signal);

  rclcpp::Node * node_ptr_;
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_common_;

  std::vector<float> input_h_;
  CudaUniquePtr<float[]> input_d_;
  /** Device buffers for each output binding (one per output; index 0 = first output) */
  std::vector<CudaUniquePtr<float[]>> output_d_;
  /** Host buffers for each output binding */
  std::vector<CudaUniquePtrHost<float[]>> output_h_;

  StreamUniquePtr stream_{makeCudaStream()};

  int input_c_;
  int input_height_;
  int input_width_;
  int max_batch_size_;
  int out_c_;              /**< output channels from engine (e.g. 48 or 7), first output only */
  int output_grid_h_;
  int output_grid_w_;
  float score_threshold_;
  float nms_threshold_;    /**< IoU threshold for NMS (overlapping detections suppressed) */

  image_transport::Publisher image_pub_;
};

}  // namespace autoware::traffic_light

#endif  // CLASSIFIER__COMLOPS_TLR_CLASSIFIER_HPP_
