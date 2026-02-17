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

// --- Detection / geometry types ---
struct BBox {
  float x1{0.f}, y1{0.f};
  float x2{0.f}, y2{0.f};
};

struct KeypointInfo {
  float x{0.f}, y{0.f};
};

struct BBoxInfo {
  BBox box;
  int label{0};
  int classId{0};       // type: circle=0, arrow=1, uturn=2, ped=3, number=4, cross=5
  float prob{0.f};
  bool isHierarchical{false};
  int subClassId{0};     // color: green=0, amber=1, red=2
  float sin{0.f};
  float cos{1.f};
  std::vector<KeypointInfo> keypoint;
};

// --- Internal element representation (before mapping to ROS message) ---
enum class Color { GREEN = 0, AMBER = 1, RED = 2, UNKNOWN = 3 };

enum class ArrowDirection {
  UP_ARROW = 0,
  DOWN_ARROW = 1,
  LEFT_ARROW = 2,
  RIGHT_ARROW = 3,
  UP_LEFT_ARROW = 4,
  UP_RIGHT_ARROW = 5,
  DOWN_LEFT_ARROW = 6,
  DOWN_RIGHT_ARROW = 7,
  UNKNOWN = 8,
};

enum class Shape {
  CIRCLE = 0,
  ARROW = 1,
  U_TURN = 2,
  PED = 3,
  NUMBER = 4,
  CROSS = 5,
  UNKNOWN = 6,
};

struct TrafficLightElement {
  Color color;
  Shape shape;
  ArrowDirection arrow_direction;
  BBox box;
  float confidence{0.f};
};

struct UniqueTrafficLightElement {
  Color color;
  Shape shape;
  BBox box;
  float confidence{0.f};
};

// --- CUDA / TensorRT aliases ---
using autoware::cuda_utils::CudaUniquePtr;
using autoware::cuda_utils::CudaUniquePtrHost;
using autoware::cuda_utils::makeCudaStream;
using autoware::cuda_utils::StreamUniquePtr;

/**
 * @brief Traffic light classifier using CoMLOps-TLR ONNX/TensorRT model.
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
  void decodeTlrOutput(size_t batch_size, std::vector<std::vector<BBoxInfo>> & detections_per_roi);
  void updateTrafficSignals(
    const std::vector<TrafficLightElement> & elements,
    tier4_perception_msgs::msg::TrafficLight & traffic_signal);
  void outputDebugImage(
    cv::Mat & debug_image,
    const tier4_perception_msgs::msg::TrafficLight & traffic_signal,
    const std::vector<TrafficLightElement> * elements = nullptr);

  rclcpp::Node * node_ptr_;
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_common_;
  StreamUniquePtr stream_{makeCudaStream()};

  std::vector<float> input_h_;
  CudaUniquePtr<float[]> input_d_;
  std::vector<CudaUniquePtr<float[]>> output_d_;
  std::vector<CudaUniquePtrHost<float[]>> output_h_;

  int input_c_;
  int input_height_;
  int input_width_;
  int max_batch_size_;
  int out_c_;
  int output_grid_h_;
  int output_grid_w_;
  float score_threshold_;
  float nms_threshold_;

  image_transport::Publisher image_pub_;
};

}  // namespace autoware::traffic_light

#endif  // CLASSIFIER__COMLOPS_TLR_CLASSIFIER_HPP_
