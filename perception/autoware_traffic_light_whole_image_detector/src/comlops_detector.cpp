// Copyright 2024 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include "autoware_traffic_light_whole_image_detector/comlops_detector.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <numeric>
#include <stdexcept>

namespace autoware::traffic_light::whole_image_detector
{

ComlopsDetector::ComlopsDetector(
  rclcpp::Node * node_ptr,
  const std::string & model_path,
  const std::string & precision,
  const std::vector<int> & anchors,
  int num_anchors,
  int num_classes,
  float score_thresh,
  float nms_thresh)
: node_ptr_(node_ptr),
  anchors_(anchors),
  num_anchors_(num_anchors),
  num_classes_(num_classes),
  score_thresh_(score_thresh),
  nms_thresh_(nms_thresh)
{
  autoware::tensorrt_common::TrtCommonConfig config(
    model_path, precision, "", (1ULL << 30U), -1, false);
  trt_common_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(config);

  nvinfer1::Dims input_dims = trt_common_->getInputDims(0);
  const int input_c = input_dims.d[1] > 0 ? input_dims.d[1] : 3;
  input_height_ = input_dims.d[2] > 0 ? input_dims.d[2] : 640;
  input_width_ = input_dims.d[3] > 0 ? input_dims.d[3] : 640;

  nvinfer1::Dims dim;
  dim.nbDims = 4;
  dim.d[0] = 1;
  dim.d[1] = input_c;
  dim.d[2] = input_height_;
  dim.d[3] = input_width_;

  auto profile_dims = std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>();
  profile_dims->push_back(autoware::tensorrt_common::ProfileDims(0, dim, dim, dim));

  if (!trt_common_->setup(std::move(profile_dims), nullptr)) {
    throw std::runtime_error("ComlopsDetector: Failed to setup TensorRT engine");
  }

  const int32_t nb_io = trt_common_->getNbIOTensors();
  const int32_t num_outputs = nb_io - 1;
  if (num_outputs < 1) {
    throw std::runtime_error("ComlopsDetector: engine has no output bindings");
  }

  output_d_.resize(num_outputs);
  output_h_.resize(num_outputs);

  for (int32_t o = 0; o < num_outputs; ++o) {
    nvinfer1::Dims dims = trt_common_->getOutputDims(o);
    for (int32_t j = 0; j < dims.nbDims; ++j) {
      if (dims.d[j] <= 0) dims.d[j] = 1;
    }
    const size_t vol =
      std::accumulate(dims.d, dims.d + dims.nbDims, static_cast<size_t>(1), std::multiplies<size_t>());
    output_d_[o] = autoware::cuda_utils::make_unique<float[]>(vol);
    output_h_[o] = autoware::cuda_utils::make_unique_host<float[]>(vol, cudaHostAllocPortable);
  }

  const size_t input_vol =
    static_cast<size_t>(1) * static_cast<size_t>(input_c) * input_height_ * input_width_;
  input_d_ = autoware::cuda_utils::make_unique<float[]>(input_vol);

  decoder_ = std::make_shared<LightnetDecoder>(
    num_classes_, num_anchors_, anchors_, score_thresh_, nms_thresh_);

  RCLCPP_INFO(
    node_ptr_->get_logger(), "ComlopsDetector: loaded %s, input %dx%d",
    model_path.c_str(), input_width_, input_height_);
}

void ComlopsDetector::preprocess(const cv::Mat & image)
{
  const float scale = 1.0f / 255.0f;
  const cv::Size input_size(input_width_, input_height_);
  cv::Mat blob = cv::dnn::blobFromImage(
    image, scale, input_size, cv::Scalar(0, 0, 0), true, false, CV_32F);
  if (!blob.isContinuous()) {
    blob = blob.clone();
  }
  input_h_.assign(blob.ptr<float>(), blob.ptr<float>() + blob.total());
  const size_t copy_size = input_h_.size() * sizeof(float);
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(input_d_.get(), input_h_.data(), copy_size, cudaMemcpyHostToDevice, *stream_));
}

bool ComlopsDetector::doInference()
{
  nvinfer1::Dims input_dims = trt_common_->getInputDims(0);
  input_dims.d[0] = 1;
  if (!trt_common_->setInputShape(0, input_dims)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "ComlopsDetector: setInputShape failed");
    return false;
  }

  std::vector<void *> buffers = {input_d_.get()};
  for (size_t i = 0; i < output_d_.size(); ++i) {
    buffers.push_back(output_d_.at(i).get());
  }
  if (!trt_common_->setTensorsAddresses(buffers)) {
    return false;
  }
  if (!trt_common_->enqueueV3(*stream_)) {
    return false;
  }

  for (size_t i = 0; i < output_d_.size(); ++i) {
    nvinfer1::Dims dims = trt_common_->getOutputDims(static_cast<int32_t>(i));
    dims.d[0] = 1;
    for (int32_t j = 1; j < dims.nbDims; ++j) {
      if (dims.d[j] <= 0) dims.d[j] = 1;
    }
    const size_t output_size = std::accumulate(
      dims.d, dims.d + dims.nbDims, static_cast<size_t>(1), std::multiplies<size_t>());
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      output_h_.at(i).get(), output_d_.at(i).get(), output_size * sizeof(float),
      cudaMemcpyDeviceToHost, *stream_));
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
  return true;
}

void ComlopsDetector::decodeOutputs(int image_h, int image_w, std::vector<BBoxInfo> & all_boxes)
{
  all_boxes.clear();
  const int chan_size = (4 + 1 + num_classes_) * num_anchors_;
  const int tlr_size = (4 + 1 + num_classes_ + 3 + 2) * num_anchors_;
  int detection_count = 0;

  for (size_t i = 0; i < output_h_.size(); ++i) {
    nvinfer1::Dims dims = trt_common_->getOutputDims(static_cast<int32_t>(i));
    const int grid_h = dims.d[2] > 0 ? dims.d[2] : 1;
    const int grid_w = dims.d[3] > 0 ? dims.d[3] : 1;
    const int chan = dims.d[1] > 0 ? dims.d[1] : 1;
    const float * data = output_h_.at(i).get();

    if (chan == chan_size) {
      std::vector<BBoxInfo> b = decoder_->decodeTensor(
        image_h, image_w, input_height_, input_width_,
        anchors_.data() + num_anchors_ * detection_count * 2, num_anchors_,
        data, grid_w, grid_h);
      all_boxes.insert(all_boxes.end(), b.begin(), b.end());
      detection_count++;
    } else if (chan == tlr_size) {
      std::vector<BBoxInfo> b = decoder_->decodeTLRTensor(
        image_h, image_w, input_height_, input_width_,
        anchors_.data() + num_anchors_ * detection_count * 2, num_anchors_,
        data, grid_w, grid_h);
      all_boxes.insert(all_boxes.end(), b.begin(), b.end());
      detection_count++;
    }
  }

  all_boxes = decoder_->nonMaximumSuppression(nms_thresh_, all_boxes);
}

bool ComlopsDetector::detect(const cv::Mat & image, std::vector<BBoxInfo> & out_boxes)
{
  preprocess(image);
  if (!doInference()) {
    return false;
  }
  decodeOutputs(image.rows, image.cols, out_boxes);
  return true;
}

}  // namespace autoware::traffic_light::whole_image_detector
