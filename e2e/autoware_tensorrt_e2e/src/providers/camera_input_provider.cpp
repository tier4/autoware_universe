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

#include "autoware/tensorrt_e2e/providers/camera_input_provider.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // for ROS 2 Jazzy or newer
#else
#include <cv_bridge/cv_bridge.h>  // for ROS 2 Humble or older
#endif

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

CameraInputProvider::CameraInputProvider(rclcpp::Node & node, tf2_ros::Buffer & tf_buffer)
: node_(node), tf_buffer_(tf_buffer)
{
  num_cameras_ = node_.declare_parameter<int64_t>("camera.num_cameras", 1);
  input_width_ = node_.declare_parameter<int64_t>("camera.input_width", 1920);
  input_height_ = node_.declare_parameter<int64_t>("camera.input_height", 1280);
  transport_ = node_.declare_parameter<std::string>("camera.image_transport", "compressed");
  ego_frame_ = node_.declare_parameter<std::string>("camera.ego_frame", "base_link");
  images_tensor_name_ =
    node_.declare_parameter<std::string>("camera.images_tensor", "camera_images");
  intrinsics_tensor_name_ =
    node_.declare_parameter<std::string>("camera.intrinsics_tensor", "camera_intrinsics");
  extrinsics_tensor_name_ =
    node_.declare_parameter<std::string>("camera.extrinsics_tensor", "camera2ego");
  sync_tolerance_ms_ = node_.declare_parameter<double>("camera.sync_tolerance_ms", 100.0);
  max_delay_ms_ = node_.declare_parameter<double>("camera.max_delay_ms", 200.0);
  allow_dropped_cameras_ = node_.declare_parameter<bool>("camera.allow_dropped_cameras", false);
  anchor_camera_index_ = node_.declare_parameter<int64_t>("camera.anchor_camera_index", 0);

  const auto mean = node_.declare_parameter<std::vector<double>>(
    "camera.normalization_mean", std::vector<double>{123.675, 116.28, 103.53});
  const auto std_dev = node_.declare_parameter<std::vector<double>>(
    "camera.normalization_std", std::vector<double>{58.395, 57.12, 57.375});
  if (mean.size() != 3 || std_dev.size() != 3) {
    throw std::runtime_error("camera.normalization_mean/std must each have 3 elements (RGB)");
  }
  for (size_t i = 0; i < 3; ++i) {
    if (std_dev[i] <= 0.0) {
      throw std::runtime_error("camera.normalization_std entries must be positive");
    }
    preprocess_config_.mean[i] = static_cast<float>(mean[i]);
    preprocess_config_.inverse_std[i] = static_cast<float>(1.0 / std_dev[i]);
  }

  if (num_cameras_ < 1) {
    throw std::runtime_error("camera.num_cameras must be >= 1");
  }
  if (anchor_camera_index_ < 0 || anchor_camera_index_ >= num_cameras_) {
    throw std::runtime_error("camera.anchor_camera_index must be in [0, camera.num_cameras)");
  }

  latest_images_.resize(num_cameras_);
  latest_camera_infos_.resize(num_cameras_);
  cached_extrinsics_.resize(num_cameras_);
}

CameraInputProvider::~CameraInputProvider()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

std::vector<std::string> CameraInputProvider::claim_inputs(
  const std::vector<TensorSpec> & engine_inputs)
{
  std::vector<std::string> claimed;

  const TensorSpec * images_spec = find_spec(engine_inputs, images_tensor_name_);
  if (!images_spec) {
    throw std::runtime_error(
      "The camera input provider is enabled but the model has no input tensor named '" +
      images_tensor_name_ + "' (set camera.images_tensor to match the model)");
  }

  // Accept [1, N, 3, H, W] or [N, 3, H, W].
  const auto & shape = images_spec->shape;
  const bool has_batch = shape.size() == 5;
  if (!(shape.size() == 4 || (has_batch && shape[0] == 1))) {
    throw std::runtime_error(
      "Model input '" + images_tensor_name_ + "' has shape " + shape_to_string(shape) +
      "; expected [1, N, 3, H, W] or [N, 3, H, W]");
  }
  const size_t base = has_batch ? 1 : 0;
  if (shape[base] != num_cameras_ || shape[base + 1] != 3) {
    throw std::runtime_error(
      "Model input '" + images_tensor_name_ + "' has shape " + shape_to_string(shape) +
      ", which does not match camera.num_cameras=" + std::to_string(num_cameras_));
  }
  images_shape_ = shape;
  preprocess_config_.num_cameras = static_cast<int32_t>(num_cameras_);
  preprocess_config_.input_width = static_cast<int32_t>(input_width_);
  preprocess_config_.input_height = static_cast<int32_t>(input_height_);
  preprocess_config_.output_height = static_cast<int32_t>(shape[base + 2]);
  preprocess_config_.output_width = static_cast<int32_t>(shape[base + 3]);
  claimed.push_back(images_tensor_name_);

  const TensorSpec * intrinsics_spec = find_spec(engine_inputs, intrinsics_tensor_name_);
  if (intrinsics_spec) {
    if (intrinsics_spec->num_elements() != num_cameras_ * 9) {
      throw std::runtime_error(
        "Model input '" + intrinsics_tensor_name_ + "' has shape " +
        shape_to_string(intrinsics_spec->shape) + "; expected [1, " +
        std::to_string(num_cameras_) + ", 3, 3]");
    }
    intrinsics_shape_ = intrinsics_spec->shape;
    intrinsics_claimed_ = true;
    claimed.push_back(intrinsics_tensor_name_);
  }

  const TensorSpec * extrinsics_spec = find_spec(engine_inputs, extrinsics_tensor_name_);
  if (extrinsics_spec) {
    if (extrinsics_spec->num_elements() != num_cameras_ * 16) {
      throw std::runtime_error(
        "Model input '" + extrinsics_tensor_name_ + "' has shape " +
        shape_to_string(extrinsics_spec->shape) + "; expected [1, " +
        std::to_string(num_cameras_) + ", 4, 4]");
    }
    extrinsics_shape_ = extrinsics_spec->shape;
    extrinsics_claimed_ = true;
    claimed.push_back(extrinsics_tensor_name_);
  }

  allocate_gpu_buffers();
  create_subscriptions();

  return claimed;
}

void CameraInputProvider::allocate_gpu_buffers()
{
  const size_t input_bytes =
    static_cast<size_t>(num_cameras_) * input_width_ * input_height_ * 3;
  const size_t resized_bytes = static_cast<size_t>(num_cameras_) *
                               preprocess_config_.output_width *
                               preprocess_config_.output_height * 3;
  const size_t output_floats = resized_bytes;  // Same element count, CHW float instead of HWC u8.

  pinned_input_ =
    autoware::cuda_utils::make_unique_host<uint8_t[]>(input_bytes, cudaHostAllocDefault);
  d_input_ = autoware::cuda_utils::make_unique<uint8_t[]>(input_bytes);
  d_resized_ = autoware::cuda_utils::make_unique<uint8_t[]>(resized_bytes);
  d_output_ = autoware::cuda_utils::make_unique<float[]>(output_floats);
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
}

void CameraInputProvider::create_subscriptions()
{
  for (int64_t i = 0; i < num_cameras_; ++i) {
    const std::string image_topic = "~/input/camera" + std::to_string(i) + "/image";
    image_subs_.push_back(image_transport::create_subscription(
      &node_, node_.get_node_topics_interface()->resolve_topic_name(image_topic),
      [this, i](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_images_[i] = msg;
      },
      transport_, rmw_qos_profile_sensor_data));

    if (intrinsics_claimed_) {
      const std::string info_topic = "~/input/camera" + std::to_string(i) + "/camera_info";
      camera_info_subs_.push_back(node_.create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS{},
        [this, i](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_camera_infos_[i] = msg;
        }));
    }
  }
}

bool CameraInputProvider::take_synchronized_images(
  const rclcpp::Time & now, std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & images,
  std::string & error) const
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    images = latest_images_;
  }

  const auto & anchor = images[anchor_camera_index_];
  if (!anchor) {
    error = "No image received yet on anchor camera " + std::to_string(anchor_camera_index_);
    return false;
  }

  const rclcpp::Time anchor_stamp(anchor->header.stamp);
  const double anchor_delay_ms = (now - anchor_stamp).seconds() * 1e3;
  if (anchor_delay_ms > max_delay_ms_) {
    error = "Anchor camera image is stale (" + std::to_string(anchor_delay_ms) + " ms > " +
            std::to_string(max_delay_ms_) + " ms)";
    return false;
  }

  for (int64_t i = 0; i < num_cameras_; ++i) {
    if (i == anchor_camera_index_) {
      continue;
    }
    const bool synchronized =
      images[i] &&
      std::abs((rclcpp::Time(images[i]->header.stamp) - anchor_stamp).seconds() * 1e3) <=
        sync_tolerance_ms_;
    if (synchronized) {
      continue;
    }
    if (!allow_dropped_cameras_) {
      error = "Camera " + std::to_string(i) +
              (images[i] ? " is out of sync with the anchor camera" : " has no image yet");
      return false;
    }
    images[i] = nullptr;  // Zero-filled downstream, following the VAD drop-handling precedent.
  }
  return true;
}

bool CameraInputProvider::build_images_tensor(
  const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & images, TensorMap & inputs,
  std::string & error)
{
  const size_t single_bytes = static_cast<size_t>(input_width_) * input_height_ * 3;

  for (int64_t i = 0; i < num_cameras_; ++i) {
    uint8_t * dst = pinned_input_.get() + static_cast<size_t>(i) * single_bytes;
    if (!images[i]) {
      std::memset(dst, 0, single_bytes);
      continue;
    }

    cv_bridge::CvImageConstPtr cv_image;
    try {
      cv_image = cv_bridge::toCvShare(images[i], "bgr8");
    } catch (const cv_bridge::Exception & e) {
      error = "Failed to convert image of camera " + std::to_string(i) + ": " + e.what();
      return false;
    }

    const cv::Mat & mat = cv_image->image;
    if (mat.cols != input_width_ || mat.rows != input_height_) {
      error = "Camera " + std::to_string(i) + " image is " + std::to_string(mat.cols) + "x" +
              std::to_string(mat.rows) + ", but camera.input_width/height is " +
              std::to_string(input_width_) + "x" + std::to_string(input_height_);
      return false;
    }
    if (mat.isContinuous()) {
      std::memcpy(dst, mat.data, single_bytes);
    } else {
      for (int64_t row = 0; row < input_height_; ++row) {
        std::memcpy(dst + row * input_width_ * 3, mat.ptr(row), input_width_ * 3);
      }
    }
  }

  const size_t total_bytes = static_cast<size_t>(num_cameras_) * single_bytes;
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_input_.get(), pinned_input_.get(), total_bytes, cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(launch_camera_resize_kernel(
    d_input_.get(), d_resized_.get(), preprocess_config_, stream_));
  CHECK_CUDA_ERROR(launch_camera_normalize_kernel(
    d_resized_.get(), d_output_.get(), preprocess_config_, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  inputs[images_tensor_name_] = Tensor::from_device(images_shape_, d_output_.get());
  return true;
}

bool CameraInputProvider::build_intrinsics_tensor(TensorMap & inputs, std::string & error) const
{
  std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> camera_infos;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_infos = latest_camera_infos_;
  }

  const double scale_x =
    static_cast<double>(preprocess_config_.output_width) / static_cast<double>(input_width_);
  const double scale_y =
    static_cast<double>(preprocess_config_.output_height) / static_cast<double>(input_height_);

  std::vector<float> data;
  data.reserve(num_cameras_ * 9);
  for (int64_t i = 0; i < num_cameras_; ++i) {
    if (!camera_infos[i]) {
      error = "No camera_info received yet for camera " + std::to_string(i);
      return false;
    }
    const auto & k = camera_infos[i]->k;  // Row-major 3x3.
    // Rescale the intrinsics to the model input resolution.
    data.insert(
      data.end(), {static_cast<float>(k[0] * scale_x), static_cast<float>(k[1] * scale_x),
                   static_cast<float>(k[2] * scale_x), static_cast<float>(k[3] * scale_y),
                   static_cast<float>(k[4] * scale_y), static_cast<float>(k[5] * scale_y),
                   static_cast<float>(k[6]), static_cast<float>(k[7]), static_cast<float>(k[8])});
  }

  inputs[intrinsics_tensor_name_] = Tensor::from_host(intrinsics_shape_, std::move(data));
  return true;
}

bool CameraInputProvider::build_extrinsics_tensor(
  const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & images, TensorMap & inputs,
  std::string & error)
{
  std::vector<float> data;
  data.reserve(num_cameras_ * 16);
  for (int64_t i = 0; i < num_cameras_; ++i) {
    if (!cached_extrinsics_[i]) {
      if (!images[i]) {
        error = "Cannot resolve the optical frame of dropped camera " + std::to_string(i);
        return false;
      }
      try {
        const auto tf = tf_buffer_.lookupTransform(
          ego_frame_, images[i]->header.frame_id, tf2::TimePointZero);
        const Eigen::Matrix4d camera_to_ego = tf2::transformToEigen(tf).matrix();
        std::array<float, 16> flat{};
        for (int64_t row = 0; row < 4; ++row) {
          for (int64_t col = 0; col < 4; ++col) {
            flat[row * 4 + col] = static_cast<float>(camera_to_ego(row, col));
          }
        }
        cached_extrinsics_[i] = flat;
      } catch (const tf2::TransformException & e) {
        error = "TF lookup failed for camera " + std::to_string(i) + " (" +
                images[i]->header.frame_id + " -> " + ego_frame_ + "): " + e.what();
        return false;
      }
    }
    data.insert(data.end(), cached_extrinsics_[i]->begin(), cached_extrinsics_[i]->end());
  }

  inputs[extrinsics_tensor_name_] = Tensor::from_host(extrinsics_shape_, std::move(data));
  return true;
}

bool CameraInputProvider::collect(
  [[maybe_unused]] const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs,
  std::string & error)
{
  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> images;
  if (!take_synchronized_images(now, images, error)) {
    return false;
  }
  // Extrinsics before images: the TF cache needs the image frame_id, and failing early avoids
  // wasted GPU work.
  if (extrinsics_claimed_ && !build_extrinsics_tensor(images, inputs, error)) {
    return false;
  }
  if (intrinsics_claimed_ && !build_intrinsics_tensor(inputs, error)) {
    return false;
  }
  if (!build_images_tensor(images, inputs, error)) {
    return false;
  }
  return true;
}

}  // namespace autoware::tensorrt_e2e
