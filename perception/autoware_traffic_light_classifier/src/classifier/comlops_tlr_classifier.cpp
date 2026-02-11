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

#include "comlops_tlr_classifier.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace autoware::traffic_light
{

using tier4_perception_msgs::msg::TrafficLightElement;

// TLR output layout per anchor: bbox(4) + obj(1) + color(3) + type(6) + angle(2) = 16
static constexpr int TLR_NUM_ANCHORS = 3;
static constexpr int TLR_CHANS_PER_ANCHOR = 16;
static constexpr int TLR_COLOR_START = 5;
static constexpr int TLR_TYPE_START = 8;
static constexpr int TLR_NUM_TYPES = 6;
static constexpr int TLR_NUM_COLORS = 3;
static constexpr int TLR_COS_INDEX = 14;
static constexpr int TLR_SIN_INDEX = 15;

CoMLOpsTLRClassifier::CoMLOpsTLRClassifier(rclcpp::Node * node_ptr) : node_ptr_(node_ptr)
{
  image_pub_ = image_transport::create_publisher(
    node_ptr_, "~/output/debug/image", rclcpp::QoS{1}.get_rmw_qos_profile());

  const std::string model_path = node_ptr_->declare_parameter<std::string>("model_path");
  const std::string precision = node_ptr_->declare_parameter<std::string>("precision");
  score_threshold_ =
    static_cast<float>(node_ptr_->declare_parameter<double>("score_threshold", 0.2));
  max_batch_size_ = node_ptr_->declare_parameter<int>("max_batch_size", 64);
  const int default_input_h = node_ptr_->declare_parameter<int>("input_height", 64);
  const int default_input_w = node_ptr_->declare_parameter<int>("input_width", 64);

  autoware::tensorrt_common::TrtCommonConfig config(
    model_path, precision, "", (1ULL << 30U), -1, false);
  trt_common_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(config);

  nvinfer1::Dims input_dims = trt_common_->getInputDims(0);
  input_c_ = input_dims.d[1] > 0 ? input_dims.d[1] : 3;
  input_height_ = input_dims.d[2] > 0 ? input_dims.d[2] : default_input_h;
  input_width_ = input_dims.d[3] > 0 ? input_dims.d[3] : default_input_w;

  const int min_batch = 1;
  const int opt_batch = std::min(32, max_batch_size_);
  const int max_batch = max_batch_size_;

  nvinfer1::Dims dim_min, dim_opt, dim_max;
  dim_min.nbDims = 4;
  dim_min.d[0] = min_batch;
  dim_min.d[1] = input_c_;
  dim_min.d[2] = input_height_;
  dim_min.d[3] = input_width_;
  dim_opt.nbDims = 4;
  dim_opt.d[0] = opt_batch;
  dim_opt.d[1] = input_c_;
  dim_opt.d[2] = input_height_;
  dim_opt.d[3] = input_width_;
  dim_max.nbDims = 4;
  dim_max.d[0] = max_batch;
  dim_max.d[1] = input_c_;
  dim_max.d[2] = input_height_;
  dim_max.d[3] = input_width_;

  auto profile_dims = std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>();
  profile_dims->push_back(
    autoware::tensorrt_common::ProfileDims(0, dim_min, dim_opt, dim_max));

  if (!trt_common_->setup(std::move(profile_dims), nullptr)) {
    throw std::runtime_error("CoMLOpsTLRClassifier: Failed to setup TensorRT engine");
  }

  nvinfer1::Dims output_dims = trt_common_->getOutputDims(0);
  const int out_c = output_dims.d[1] > 0 ? output_dims.d[1] : 48;
  output_grid_h_ = output_dims.d[2] > 0 ? output_dims.d[2] : 8;
  output_grid_w_ = output_dims.d[3] > 0 ? output_dims.d[3] : 8;

  const size_t input_vol = static_cast<size_t>(max_batch_size_) * input_c_ * input_height_ *
                            input_width_;
  input_d_ = autoware::cuda_utils::make_unique<float[]>(input_vol);

  const size_t output_vol = static_cast<size_t>(max_batch_size_) * out_c * output_grid_h_ *
                            output_grid_w_;
  output_d_ = autoware::cuda_utils::make_unique<float[]>(output_vol);
  output_h_ = autoware::cuda_utils::make_unique_host<float[]>(output_vol, cudaHostAllocPortable);
}

void CoMLOpsTLRClassifier::preprocess(const std::vector<cv::Mat> & images)
{
  const float scale = 1.0f / 255.0f;
  const cv::Size input_size(input_width_, input_height_);
  cv::Mat blob = cv::dnn::blobFromImages(
    images, scale, input_size, cv::Scalar(0, 0, 0), false, false, CV_32F); // Skip swap RB channels
  if (!blob.isContinuous()) {
    blob = blob.clone();
  }
  input_h_.assign(blob.ptr<float>(), blob.ptr<float>() + blob.total());
  const size_t copy_size = input_h_.size() * sizeof(float);
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(input_d_.get(), input_h_.data(), copy_size, cudaMemcpyHostToDevice, *stream_));
}

bool CoMLOpsTLRClassifier::doInference(size_t batch_size)
{
  nvinfer1::Dims input_dims = trt_common_->getInputDims(0);
  input_dims.d[0] = static_cast<int32_t>(batch_size);
  if (!trt_common_->setInputShape(0, input_dims)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "CoMLOpsTLR: setInputShape failed");
    return false;
  }

  const int out_c = 48;
  const size_t out_vol =
    static_cast<size_t>(batch_size) * out_c * output_grid_h_ * output_grid_w_;
  std::vector<void *> buffers = {input_d_.get(), output_d_.get()};
  if (!trt_common_->setTensorsAddresses(buffers)) {
    return false;
  }
  if (!trt_common_->enqueueV3(*stream_)) {
    return false;
  }
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output_h_.get(), output_d_.get(), out_vol * sizeof(float), cudaMemcpyDeviceToHost, *stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
  return true;
}

void CoMLOpsTLRClassifier::decodeTlrOutput(
  size_t batch_size, std::vector<int> & colors, std::vector<int> & types,
  std::vector<float> & confidences)
{
  colors.resize(batch_size, 0);
  types.resize(batch_size, 0);
  confidences.resize(batch_size, 0.0f);

  const int grid_size = output_grid_h_ * output_grid_w_;
  const int out_c = TLR_NUM_ANCHORS * TLR_CHANS_PER_ANCHOR;  // 48
  const size_t batch_stride = static_cast<size_t>(out_c) * grid_size;

  for (size_t b = 0; b < batch_size; ++b) {
    const float * out = output_h_.get() + b * batch_stride;
    float best_score = 0.0f;
    int best_color = 0;
    int best_type = 0;

    for (int y = 0; y < output_grid_h_; ++y) {
      for (int x = 0; x < output_grid_w_; ++x) {
        const int cell = y * output_grid_w_ + x;
        for (int a = 0; a < TLR_NUM_ANCHORS; ++a) {
          const int base = (a * TLR_CHANS_PER_ANCHOR) * grid_size + cell;
          const float obj = out[base + 4 * grid_size];
          if (obj < score_threshold_) continue;

          float max_type_prob = 0.0f;
          int type_idx = 0;
          for (int t = 0; t < TLR_NUM_TYPES; ++t) {
            float p = out[base + (TLR_TYPE_START + t) * grid_size];
            if (p > max_type_prob) {
              max_type_prob = p;
              type_idx = t;
            }
          }
          float max_color_prob = 0.0f;
          int color_idx = 0;
          for (int c = 0; c < TLR_NUM_COLORS; ++c) {
            float p = out[base + (TLR_COLOR_START + c) * grid_size];
            if (p > max_color_prob) {
              max_color_prob = p;
              color_idx = c;
            }
          }
          const float score = obj * max_type_prob;
          if (score > best_score) {
            best_score = score;
            best_color = color_idx;
            best_type = type_idx;
          }
        }
      }
    }
    colors[b] = best_color;
    types[b] = best_type;
    confidences[b] = best_score;
  }
}

void CoMLOpsTLRClassifier::toTrafficLightElements(
  int color_index, int type_index, float confidence,
  tier4_perception_msgs::msg::TrafficLight & traffic_signal)
{
  TrafficLightElement element;
  element.confidence = confidence;

  if (color_index == 0) {
    element.color = TrafficLightElement::GREEN;
  } else if (color_index == 1) {
    element.color = TrafficLightElement::AMBER;
  } else if (color_index == 2) {
    element.color = TrafficLightElement::RED;
  } else {
    element.color = TrafficLightElement::UNKNOWN;
  }

  switch (type_index) {
    case 0:
      element.shape = TrafficLightElement::CIRCLE;
      break;
    case 1:
      element.shape = TrafficLightElement::UP_ARROW;
      break;
    case 2:
      element.shape = TrafficLightElement::LEFT_ARROW;
      break;
    case 3:
      element.shape = TrafficLightElement::CIRCLE;
      break;
    case 4:
      element.shape = TrafficLightElement::CIRCLE;
      break;
    case 5:
      element.shape = TrafficLightElement::CROSS;
      break;
    default:
      element.shape = TrafficLightElement::UNKNOWN;
  }

  traffic_signal.elements.push_back(element);
}

void CoMLOpsTLRClassifier::outputDebugImage(
  cv::Mat & debug_image, const tier4_perception_msgs::msg::TrafficLight & traffic_signal)
{
  std::string label;
  float probability = 0.0f;
  for (std::size_t i = 0; i < traffic_signal.elements.size(); i++) {
    const auto & light = traffic_signal.elements.at(i);
    if (light.color == TrafficLightElement::GREEN) label += "green";
    else if (light.color == TrafficLightElement::AMBER) label += "yellow";
    else if (light.color == TrafficLightElement::RED) label += "red";
    else label += "unknown";
    if (light.shape == TrafficLightElement::CIRCLE) label += "-circle";
    else if (light.shape == TrafficLightElement::UP_ARROW) label += "-arrow";
    else if (light.shape == TrafficLightElement::CROSS) label += "-cross";
    else label += "";
    probability = light.confidence;
    if (i < traffic_signal.elements.size() - 1) label += ",";
  }
  const int expand_w = 200;
  const int expand_h =
    std::max(static_cast<int>((expand_w * debug_image.rows) / debug_image.cols), 1);
  cv::resize(debug_image, debug_image, cv::Size(expand_w, expand_h));
  cv::Mat text_img(cv::Size(expand_w, 50), CV_8UC3, cv::Scalar(0, 0, 0));
  std::string text = label + " " + std::to_string(probability);
  cv::putText(
    text_img, text, cv::Point(5, 25), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
  cv::vconcat(debug_image, text_img, debug_image);
  const auto debug_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", debug_image).toImageMsg();
  image_pub_.publish(debug_image_msg);
}

bool CoMLOpsTLRClassifier::getTrafficSignals(
  const std::vector<cv::Mat> & images,
  tier4_perception_msgs::msg::TrafficLightArray & traffic_signals)
{
  if (images.size() != traffic_signals.signals.size()) {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "CoMLOpsTLR: image count (%zu) != signal count (%zu)", images.size(),
      traffic_signals.signals.size());
    return false;
  }

  size_t signal_i = 0;
  std::vector<cv::Mat> batch;

  for (size_t image_i = 0; image_i < images.size(); image_i++) {
    batch.push_back(images[image_i]);
    const size_t current_batch_size = batch.size();
    const bool flush = (current_batch_size >= static_cast<size_t>(max_batch_size_)) ||
                       (image_i + 1 == images.size());

    if (!flush) continue;

    preprocess(batch);
    if (!doInference(current_batch_size)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "CoMLOpsTLR: inference failed");
      return false;
    }

    std::vector<int> colors, types;
    std::vector<float> confidences;
    decodeTlrOutput(current_batch_size, colors, types, confidences);

    for (size_t i = 0; i < current_batch_size; i++) {
      toTrafficLightElements(
        colors[i], types[i], confidences[i], traffic_signals.signals[signal_i]);
      if (image_pub_.getNumSubscribers() > 0) {
        outputDebugImage(batch[i], traffic_signals.signals[signal_i]);
      }
      signal_i++;
    }
    batch.clear();
  }
  return true;
}

}  // namespace autoware::traffic_light
