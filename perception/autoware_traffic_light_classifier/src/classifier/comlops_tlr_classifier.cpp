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
#include <string>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{

using MsgTE = tier4_perception_msgs::msg::TrafficLightElement;

namespace
{

constexpr float kPi = 3.14159265358979f;
constexpr int kDebugImageWidth = 200;
constexpr int kDebugTextHeight = 50;

// TLR output layout per anchor (bbox(4) + obj(1) + color(3) + type(6) + angle(2) = 16)
constexpr int TLR_NUM_ANCHORS = 3;
constexpr int TLR_CHANS_PER_ANCHOR = 16;
constexpr int TLR_X_INDEX = 0, TLR_Y_INDEX = 1, TLR_W_INDEX = 2, TLR_H_INDEX = 3;
constexpr int TLR_OBJ_INDEX = 4;
constexpr int TLR_COLOR_START = 5, TLR_TYPE_START = 8;
constexpr int TLR_NUM_TYPES = 6, TLR_NUM_COLORS = 3;
constexpr int TLR_COS_INDEX = 14, TLR_SIN_INDEX = 15;
constexpr float TLR_SCALE_X_Y = 2.0f;
constexpr float TLR_BBOX_OFFSET = 0.5f * (TLR_SCALE_X_Y - 1.0f);
constexpr float TLR_ANCHORS[TLR_NUM_ANCHORS * 2] = {1.0f, 1.0f, 2.0f, 2.0f, 4.0f, 4.0f};

ArrowDirection angleToArrowDirection(float angle_rad)
{
  if (angle_rad >= -kPi / 8.0f && angle_rad < kPi / 8.0f) return ArrowDirection::UP_ARROW;
  if (angle_rad >= kPi / 8.0f && angle_rad < 3.0f * kPi / 8.0f) return ArrowDirection::UP_RIGHT_ARROW;
  if (angle_rad >= 3.0f * kPi / 8.0f && angle_rad < 5.0f * kPi / 8.0f) return ArrowDirection::RIGHT_ARROW;
  if (angle_rad >= 5.0f * kPi / 8.0f && angle_rad < 7.0f * kPi / 8.0f) return ArrowDirection::DOWN_RIGHT_ARROW;
  if (angle_rad >= 7.0f * kPi / 8.0f || angle_rad < -7.0f * kPi / 8.0f) return ArrowDirection::DOWN_ARROW;
  if (angle_rad >= -7.0f * kPi / 8.0f && angle_rad < -5.0f * kPi / 8.0f) return ArrowDirection::DOWN_LEFT_ARROW;
  if (angle_rad >= -5.0f * kPi / 8.0f && angle_rad < -3.0f * kPi / 8.0f) return ArrowDirection::LEFT_ARROW;
  return ArrowDirection::UP_LEFT_ARROW;
}

}  // namespace

static float boxIou(const BBox & bbox1, const BBox & bbox2)
{
  auto overlap1D = [](float x1min, float x1max, float x2min, float x2max) -> float {
    if (x1min > x2min) {
      std::swap(x1min, x2min);
      std::swap(x1max, x2max);
    }
    return x1max < x2min ? 0.0f : std::min(x1max, x2max) - x2min;
  };
  const float overlap_x = overlap1D(bbox1.x1, bbox1.x2, bbox2.x1, bbox2.x2);
  const float overlap_y = overlap1D(bbox1.y1, bbox1.y2, bbox2.y1, bbox2.y2);
  const float area1 = (bbox1.x2 - bbox1.x1) * (bbox1.y2 - bbox1.y1);
  const float area2 = (bbox2.x2 - bbox2.x1) * (bbox2.y2 - bbox2.y1);
  const float overlap_2d = overlap_x * overlap_y;
  const float u = area1 + area2 - overlap_2d;
  return (u == 0.0f) ? 0.0f : overlap_2d / u;
}

static void runNms(
  std::vector<BBoxInfo> & detections, float iou_threshold,
  std::vector<BBoxInfo> & out)
{
  out.clear();
  if (detections.empty()) return;
  std::stable_sort(detections.begin(), detections.end(),
    [](const BBoxInfo & b1, const BBoxInfo & b2) { return b1.prob > b2.prob; });
  for (const auto & i : detections) {
    bool keep = true;
    for (const auto & j : out) {
      if (keep) {
        const float overlap = boxIou(i.box, j.box);
        keep = (overlap <= iou_threshold);
      } else {
        break;
      }
    }
    if (keep) out.push_back(i);
  }
}

CoMLOpsTLRClassifier::CoMLOpsTLRClassifier(rclcpp::Node * node_ptr) : node_ptr_(node_ptr)
{
  image_pub_ = image_transport::create_publisher(
    node_ptr_, "~/output/debug/image", rclcpp::QoS{1}.get_rmw_qos_profile());

  const std::string model_path = node_ptr_->declare_parameter<std::string>("model_path");
  const std::string precision = node_ptr_->declare_parameter<std::string>("precision");
  score_threshold_ =
    static_cast<float>(node_ptr_->declare_parameter<double>("score_threshold", 0.2));
  nms_threshold_ =
    static_cast<float>(node_ptr_->declare_parameter<double>("nms_threshold", 0.5));
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

  const int32_t nb_io = trt_common_->getNbIOTensors();
  const int32_t num_outputs = nb_io - 1;
  if (num_outputs < 1) {
    throw std::runtime_error("CoMLOpsTLRClassifier: engine has no output bindings");
  }

  output_d_.resize(num_outputs);
  output_h_.resize(num_outputs);

  for (int32_t o = 0; o < num_outputs; ++o) {
    nvinfer1::Dims dims = trt_common_->getOutputDims(o);
    const int32_t batch_dim = dims.d[0] > 0 ? dims.d[0] : max_batch_size_;
    dims.d[0] = batch_dim >= 1 ? batch_dim : 1;
    if (o == 0) {
      out_c_ = dims.d[1] > 0 ? dims.d[1] : 48;
      output_grid_h_ = dims.d[2] > 0 ? dims.d[2] : 8;
      output_grid_w_ = dims.d[3] > 0 ? dims.d[3] : 8;
      dims.d[1] = out_c_;
      dims.d[2] = output_grid_h_;
      dims.d[3] = output_grid_w_;
    }
    for (int32_t j = 1; j < dims.nbDims; ++j) {
      if (dims.d[j] <= 0) dims.d[j] = 1;
    }
    const size_t vol = std::accumulate(
      dims.d, dims.d + dims.nbDims, static_cast<size_t>(1), std::multiplies<size_t>());
    output_d_[o] = autoware::cuda_utils::make_unique<float[]>(vol);
    output_h_[o] = autoware::cuda_utils::make_unique_host<float[]>(vol, cudaHostAllocPortable);
  }

  const size_t input_vol = static_cast<size_t>(max_batch_size_) * input_c_ * input_height_ *
                            input_width_;
  input_d_ = autoware::cuda_utils::make_unique<float[]>(input_vol);
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
    dims.d[0] = static_cast<int32_t>(batch_size);
    for (int32_t j = 1; j < dims.nbDims; ++j) {
      if (dims.d[j] <= 0) dims.d[j] = 1;
    }
    const size_t output_size =
      std::accumulate(dims.d, dims.d + dims.nbDims, static_cast<size_t>(1), std::multiplies<size_t>());
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      output_h_.at(i).get(), output_d_.at(i).get(), output_size * sizeof(float),
      cudaMemcpyDeviceToHost, *stream_));
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
  return true;
}

void CoMLOpsTLRClassifier::decodeTlrOutput(
  size_t batch_size, std::vector<std::vector<BBoxInfo>> & detections_per_roi)
{
  detections_per_roi.resize(batch_size);

  const int grid_size = output_grid_h_ * output_grid_w_;
  const size_t batch_stride =
    static_cast<size_t>(out_c_) * static_cast<size_t>(output_grid_h_) * output_grid_w_;
  const float * out_base = output_h_.at(0).get();
  for (size_t b = 0; b < batch_size; ++b) {
    detections_per_roi[b].clear();

    // 1) Decode all raw detections (bbox + obj + color + type + angle) as in TrtLightnet::decodeTLRTensor.
    std::vector<BBoxInfo> raw;
    const float * out = out_base + b * batch_stride;
    const float grid_w_f = static_cast<float>(output_grid_w_);
    const float grid_h_f = static_cast<float>(output_grid_h_);

    for (int y = 0; y < output_grid_h_; ++y) {
      for (int x = 0; x < output_grid_w_; ++x) {
        const int cell = y * output_grid_w_ + x;
        for (int a = 0; a < TLR_NUM_ANCHORS; ++a) {
          const int base = (a * TLR_CHANS_PER_ANCHOR) * grid_size + cell;

          const float objectness = out[base + TLR_OBJ_INDEX * grid_size];
          if (objectness < score_threshold_) continue;

          // Decode bounding box (Scaled-YOLOv4): bx = x + scale_x_y*tx - offset; bw = pw*(tw*2)^2
          const float pw = TLR_ANCHORS[a * 2];
          const float ph = TLR_ANCHORS[a * 2 + 1];
          const float tx = out[base + TLR_X_INDEX * grid_size];
          const float ty = out[base + TLR_Y_INDEX * grid_size];
          const float tw = out[base + TLR_W_INDEX * grid_size];
          const float th = out[base + TLR_H_INDEX * grid_size];
          const float bx = x + TLR_SCALE_X_Y * tx - TLR_BBOX_OFFSET;
          const float by = y + TLR_SCALE_X_Y * ty - TLR_BBOX_OFFSET;
          const float bw = pw * std::pow(tw * 2.0f, 2.0f);
          const float bh = ph * std::pow(th * 2.0f, 2.0f);

          // Decode class (type) probabilities and take max
          float max_type_prob = 0.0f;
          int type_idx = 0;
          for (int t = 0; t < TLR_NUM_TYPES; ++t) {
            const float p = out[base + (TLR_TYPE_START + t) * grid_size];
            if (p > max_type_prob) {
              max_type_prob = p;
              type_idx = t;
            }
          }
          // Decode sub-class (color) probabilities and take max
          float max_color_prob = 0.0f;
          int color_idx = 0;
          for (int c = 0; c < TLR_NUM_COLORS; ++c) {
            const float p = out[base + (TLR_COLOR_START + c) * grid_size];
            if (p > max_color_prob) {
              max_color_prob = p;
              color_idx = c;
            }
          }

          const float score = objectness * max_type_prob;
          if (score < score_threshold_) continue;

          const float cos_val = out[base + TLR_COS_INDEX * grid_size];
          const float sin_val = out[base + TLR_SIN_INDEX * grid_size];

          // Convert decoded box (grid space) to normalized [0,1] (x1,y1,x2,y2)
          const float cx = bx / grid_w_f;
          const float cy = by / grid_h_f;
          const float half_w = (bw * 0.5f) / grid_w_f;
          const float half_h = (bh * 0.5f) / grid_h_f;
          float x1 = cx - half_w;
          float y1 = cy - half_h;
          float x2 = cx + half_w;
          float y2 = cy + half_h;
          x1 = std::max(0.0f, std::min(1.0f, x1));
          y1 = std::max(0.0f, std::min(1.0f, y1));
          x2 = std::max(0.0f, std::min(1.0f, x2));
          y2 = std::max(0.0f, std::min(1.0f, y2));

          BBoxInfo info;
          info.box.x1 = x1;
          info.box.y1 = y1;
          info.box.x2 = x2;
          info.box.y2 = y2;
          info.label = type_idx;
          info.classId = type_idx;
          info.prob = score;
          info.isHierarchical = true;
          info.subClassId = color_idx;
          info.sin = sin_val;
          info.cos = cos_val;
          info.keypoint.clear();
          raw.push_back(info);
        }
      }
    }

    // 2) NMS: keep highest-score per (classId, subClassId), suppress overlapping boxes.
    runNms(raw, nms_threshold_, detections_per_roi[b]);
  }
}


void CoMLOpsTLRClassifier::outputDebugImage(
  cv::Mat & debug_image, const tier4_perception_msgs::msg::TrafficLight & traffic_signal,
  const std::vector<TrafficLightElement> * unique_elements)
{
  const int img_w = debug_image.cols;
  const int img_h = debug_image.rows;

  if (unique_elements && !unique_elements->empty()) {
    // Image is RGB: (R, G, B)
    static const cv::Scalar colors[] = {
      cv::Scalar(0, 255, 0),    // green  (R, G, B)
      cv::Scalar(255, 255, 0),  // yellow/amber
      cv::Scalar(255, 0, 0),    // red
    };
    for (const auto & d : *unique_elements) {
      const int x1 = static_cast<int>(d.box.x1 * img_w);
      const int y1 = static_cast<int>(d.box.y1 * img_h);
      const int x2 = static_cast<int>(d.box.x2 * img_w);
      const int y2 = static_cast<int>(d.box.y2 * img_h);
      const int color_idx = std::min(2, std::max(0, static_cast<int>(d.color)));
      cv::rectangle(debug_image, cv::Point(x1, y1), cv::Point(x2, y2), colors[color_idx], 2);
      const std::string prob_str = std::to_string(static_cast<int>(d.confidence * 100)) + "%";
      cv::putText(
        debug_image, prob_str, cv::Point(x1, std::max(12, y1 - 2)), cv::FONT_HERSHEY_SIMPLEX,
        0.4, colors[color_idx], 1);
    }
  }

  // One token per lamp, comma-separated: green | left,red | red,right,straight | red,up_left | etc.
  std::string label;
  // check if traffic_signal.traffic_light_type is pedestrian (=1)
  bool is_pedestrian = traffic_signal.traffic_light_type == 1;
  float probability = 0.0f;
  const auto colorStr = [](uint8_t c) -> std::string {
    if (c == MsgTE::GREEN) return "green";
    if (c == MsgTE::AMBER) return "yellow";
    if (c == MsgTE::RED) return "red";
    return "unknown";
  };
  const auto shapeStr = [](uint8_t s) -> std::string {
    if (s == MsgTE::UP_ARROW) return "straight";
    if (s == MsgTE::DOWN_ARROW) return "down";
    if (s == MsgTE::LEFT_ARROW) return "left";
    if (s == MsgTE::RIGHT_ARROW) return "right";
    if (s == MsgTE::UP_LEFT_ARROW) return "up_left";
    if (s == MsgTE::UP_RIGHT_ARROW) return "up_right";
    if (s == MsgTE::DOWN_LEFT_ARROW) return "down_left";
    if (s == MsgTE::DOWN_RIGHT_ARROW) return "down_right";
    return "unknown";
  };
  
  for (std::size_t i = 0; i < traffic_signal.elements.size(); i++) {
    const auto & light = traffic_signal.elements.at(i);
    if (i > 0) label += ",";
    const std::string c = colorStr(light.color);
    const std::string sh = shapeStr(light.shape);
    bool is_arrow = (light.shape != MsgTE::CIRCLE &&
                     light.shape != MsgTE::CROSS &&
                     light.shape != MsgTE::UNKNOWN);
    if (is_arrow) {
      label += sh;  // one token per lamp: "left" "right" "straight" "up_left" "up_right"
    } else if(is_pedestrian) {
      label += "ped_" + c;  // one token per lamp: "ped_green" "ped_red" "ped_yellow" "ped_unknown"
    } else {
      label += c;  // one token per lamp: "green" "red" "yellow" "unknown"
    }
    probability = light.confidence;
  }
  const int expand_h =
    std::max(static_cast<int>((kDebugImageWidth * debug_image.rows) / debug_image.cols), 1);
  cv::resize(debug_image, debug_image, cv::Size(kDebugImageWidth, expand_h));
  cv::Mat text_img(cv::Size(kDebugImageWidth, kDebugTextHeight), CV_8UC3, cv::Scalar(0, 0, 0));
  const std::string text = label + " " + std::to_string(probability);
  cv::putText(
    text_img, text, cv::Point(5, 25), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
  cv::vconcat(debug_image, text_img, debug_image);
  const auto debug_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", debug_image).toImageMsg();
  image_pub_.publish(debug_image_msg);
}

static void cvtBBoxInfo2TrafficLightElement(const BBoxInfo & d, TrafficLightElement & element)
{
  element.color = static_cast<Color>(std::min(2, std::max(0, d.subClassId)));
  element.confidence = d.prob;
  element.box = d.box;
  element.shape = static_cast<Shape>(d.classId);
  if (element.shape == Shape::ARROW) {
    element.arrow_direction = angleToArrowDirection(std::atan2(d.sin, d.cos));
    element.color = Color::GREEN;
  }
} 

void CoMLOpsTLRClassifier::updateTrafficSignals(const std::vector<TrafficLightElement> & unique_elements, tier4_perception_msgs::msg::TrafficLight & traffic_signal)
{
  traffic_signal.elements.clear();
  MsgTE element;
  for (const auto & e : unique_elements) {
    element.confidence = e.confidence;
    switch (e.shape) {
      case Shape::CIRCLE:
        element.shape = MsgTE::CIRCLE;
        switch (e.color) {
          case Color::GREEN: element.color = MsgTE::GREEN; break;
          case Color::AMBER: element.color = MsgTE::AMBER; break;
          case Color::RED: element.color = MsgTE::RED; break;
          default: element.color = MsgTE::UNKNOWN; break;
        }
        break;
      case Shape::ARROW:
        element.color = MsgTE::GREEN;
        switch (e.arrow_direction) {
          case ArrowDirection::UP_ARROW: element.shape = MsgTE::UP_ARROW; break;
          case ArrowDirection::DOWN_ARROW: element.shape = MsgTE::DOWN_ARROW; break;
          case ArrowDirection::LEFT_ARROW: element.shape = MsgTE::LEFT_ARROW; break;
          case ArrowDirection::RIGHT_ARROW: element.shape = MsgTE::RIGHT_ARROW; break;
          case ArrowDirection::UP_LEFT_ARROW: element.shape = MsgTE::UP_LEFT_ARROW; break;
          case ArrowDirection::UP_RIGHT_ARROW: element.shape = MsgTE::UP_RIGHT_ARROW; break;
          case ArrowDirection::DOWN_LEFT_ARROW: element.shape = MsgTE::DOWN_LEFT_ARROW; break;
          case ArrowDirection::DOWN_RIGHT_ARROW: element.shape = MsgTE::DOWN_RIGHT_ARROW; break;
          default: element.shape = MsgTE::UNKNOWN; break;
        }
        break;
      case Shape::PED:
        element.shape = MsgTE::CROSS;
        switch (e.color) {
          case Color::GREEN: element.color = MsgTE::GREEN; break;
          case Color::AMBER: element.color = MsgTE::AMBER; break;
          case Color::RED: element.color = MsgTE::RED; break;
          default: element.color = MsgTE::UNKNOWN; break;
        }
        break;
      default:
        element.color = MsgTE::UNKNOWN;
        element.shape = MsgTE::UNKNOWN;
        break;
    }
    traffic_signal.elements.push_back(element);
  }
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

    std::vector<std::vector<BBoxInfo>> detections_per_roi;
    decodeTlrOutput(current_batch_size, detections_per_roi);
    size_t total_detections = 0;
    for (size_t i = 0; i < current_batch_size; ++i) {
      const size_t sig_idx = signal_i + i;
      total_detections += detections_per_roi[i].size();

      std::vector<TrafficLightElement> traffic_light_elements;
      for (const auto & d : detections_per_roi[i]) {
        TrafficLightElement element;
        cvtBBoxInfo2TrafficLightElement(d, element);
        traffic_light_elements.push_back(element);
      }

      std::vector<TrafficLightElement> unique_traffic_light_elements;
      for (const auto & el : traffic_light_elements) {
        auto it = std::find_if(
          unique_traffic_light_elements.begin(), unique_traffic_light_elements.end(),
          [&](const TrafficLightElement & e) {
            return e.shape == el.shape && e.arrow_direction == el.arrow_direction;
          });
        if (it == unique_traffic_light_elements.end()) {
          unique_traffic_light_elements.push_back(el);
        } else {
          it->confidence = std::max(it->confidence, el.confidence);
        }
      }

      updateTrafficSignals(unique_traffic_light_elements, traffic_signals.signals[sig_idx]);

      if (image_pub_.getNumSubscribers() > 0) {
        cv::Mat debug_img = batch[i].clone();
        outputDebugImage(debug_img, traffic_signals.signals[sig_idx], &unique_traffic_light_elements);
      }
    }
    signal_i += current_batch_size;
    RCLCPP_INFO(
      node_ptr_->get_logger(), "batch combined signal: %zu detections", total_detections);
    batch.clear();
  }
  return true;
}

}  // namespace autoware::traffic_light
