// Copyright 2024 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE_TRAFFIC_LIGHT_WHOLE_IMAGE_DETECTOR__COMLOPS_DETECTOR_HPP_
#define AUTOWARE_TRAFFIC_LIGHT_WHOLE_IMAGE_DETECTOR__COMLOPS_DETECTOR_HPP_

#include "autoware_traffic_light_whole_image_detector/lightnet_decoder.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/stream_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace autoware::traffic_light::whole_image_detector
{

using autoware::cuda_utils::CudaUniquePtr;
using autoware::cuda_utils::CudaUniquePtrHost;
using autoware::cuda_utils::makeCudaStream;
using autoware::cuda_utils::StreamUniquePtr;

/**
 * @brief Whole-image traffic light detector using CoMLOps ONNX/TensorRT model.
 * Same inference pattern as autoware_traffic_light_classifier ComlopsTLRClassifier.
 */
class ComlopsDetector
{
public:
  ComlopsDetector(
    rclcpp::Node * node_ptr,
    const std::string & model_path,
    const std::string & precision,
    const std::vector<int> & anchors,
    int num_anchors,
    int num_classes,
    float score_thresh,
    float nms_thresh);

  bool detect(
    const cv::Mat & image,
    std::vector<BBoxInfo> & out_boxes);

  int getInputHeight() const { return input_height_; }
  int getInputWidth() const { return input_width_; }

private:
  void preprocess(const cv::Mat & image);
  bool doInference();
  void decodeOutputs(int image_h, int image_w, std::vector<BBoxInfo> & all_boxes);

  rclcpp::Node * node_ptr_;
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_common_;
  StreamUniquePtr stream_{makeCudaStream()};

  std::vector<float> input_h_;
  CudaUniquePtr<float[]> input_d_;
  std::vector<CudaUniquePtr<float[]>> output_d_;
  std::vector<CudaUniquePtrHost<float[]>> output_h_;

  int input_height_;
  int input_width_;
  std::vector<int> anchors_;
  int num_anchors_;
  int num_classes_;
  float score_thresh_;
  float nms_thresh_;
  std::shared_ptr<LightnetDecoder> decoder_;
};

}  // namespace autoware::traffic_light::whole_image_detector

#endif  // AUTOWARE_TRAFFIC_LIGHT_WHOLE_IMAGE_DETECTOR__COMLOPS_DETECTOR_HPP_
