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

#include "autoware/tensorrt_e2e/providers/latentdrive_input_provider.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // for ROS 2 Jazzy or newer
#else
#include <cv_bridge/cv_bridge.h>  // for ROS 2 Humble or older
#endif

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace latentdrive
{

bool preprocess_frame(
  const cv::Mat & bgr, const int64_t width, const int64_t height, const std::array<float, 3> & mean,
  const std::array<float, 3> & inverse_std, float * out_chw)
{
  if (bgr.empty() || bgr.channels() != 3) {
    return false;
  }

  // Vehicle preprocessing: keep the full width, centre-crop the height to width / 2, then
  // resize to the model input. Crop and resize come before the channel swap because bilinear
  // resampling is per channel, so the result is the same and the work is done on fewer pixels.
  cv::Mat image = bgr;
  const int target_h = image.cols / 2;
  if (image.rows > target_h) {
    const int y0 = (image.rows - target_h) / 2;
    image = image(cv::Rect(0, y0, image.cols, target_h));
  }
  if (image.rows != height || image.cols != width) {
    cv::resize(
      image, image, cv::Size(static_cast<int>(width), static_cast<int>(height)), 0, 0,
      cv::INTER_LINEAR);
  }
  cv::Mat rgb;
  cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
  cv::Mat rgb_f32;
  rgb.convertTo(rgb_f32, CV_32FC3, 1.0 / 255.0);

  // HWC -> CHW with the per-channel normalization.
  const size_t plane = static_cast<size_t>(height) * static_cast<size_t>(width);
  for (int c = 0; c < 3; ++c) {
    float * out_plane = out_chw + c * plane;
    for (int64_t y = 0; y < height; ++y) {
      const float * row = rgb_f32.ptr<float>(static_cast<int>(y));
      float * out = out_plane + y * width;
      for (int64_t x = 0; x < width; ++x) {
        out[x] = (row[x * 3 + c] - mean[c]) * inverse_std[c];
      }
    }
  }
  return true;
}

std::optional<std::vector<size_t>> select_frame_slots(
  const std::vector<double> & stamps, const int64_t num_frames, const double interval,
  const double tolerance)
{
  if (stamps.empty() || num_frames < 1) {
    return std::nullopt;
  }
  const double newest = stamps.back();
  std::vector<size_t> slots;
  slots.reserve(num_frames);
  for (int64_t k = 0; k < num_frames; ++k) {
    const double target = newest - static_cast<double>(num_frames - 1 - k) * interval;
    size_t best = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < stamps.size(); ++i) {
      const double distance = std::abs(stamps[i] - target);
      if (distance < best_distance) {
        best_distance = distance;
        best = i;
      }
    }
    if (best_distance > tolerance) {
      return std::nullopt;
    }
    slots.push_back(best);
  }
  return slots;
}

Eigen::Vector2d subgoal_along(
  const std::vector<Eigen::Vector2d> & reference, const Eigen::Vector2d & position,
  const double ahead_m)
{
  if (reference.empty()) {
    throw std::invalid_argument("subgoal_along needs at least one reference point");
  }
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < reference.size(); ++i) {
    const double distance = (reference[i] - position).norm();
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }
  double travelled = 0.0;
  size_t j = nearest;
  for (; j + 1 < reference.size() && travelled < ahead_m; ++j) {
    travelled += (reference[j + 1] - reference[j]).norm();
  }
  return reference[j];
}

}  // namespace latentdrive

namespace
{
constexpr int64_t STATUS_DIM = 6;
constexpr int64_t LOG_THROTTLE_INTERVAL_MS = 5000;
/// Frames older than the clip span plus this margin are dropped on push.
constexpr double PRUNE_MARGIN_S = 0.2;
/// A stamp this far behind the previous one means the source restarted (a looping bag).
constexpr double STAMP_JUMP_BACK_S = 1.0;
}  // namespace

LatentDriveInputProvider::LatentDriveInputProvider(rclcpp::Node & node) : node_(node)
{
  video_tensor_name_ = node_.declare_parameter<std::string>("latentdrive.video_tensor", "video");
  status_tensor_name_ = node_.declare_parameter<std::string>("latentdrive.status_tensor", "status");
  transport_ = node_.declare_parameter<std::string>("latentdrive.image_transport", "compressed");
  frame_interval_s_ = node_.declare_parameter<double>("latentdrive.frame_interval_seconds", 0.5);
  frame_tolerance_s_ =
    node_.declare_parameter<double>("latentdrive.frame_interval_tolerance_seconds", 0.15);
  max_delay_ms_ = node_.declare_parameter<double>("latentdrive.max_delay_ms", 200.0);
  subgoal_ahead_m_ = node_.declare_parameter<double>("latentdrive.subgoal_ahead_m", 50.0);
  subgoal_divisor_ = node_.declare_parameter<double>("latentdrive.status_subgoal_divisor", 10.0);
  tick_log_ = node_.declare_parameter<bool>("latentdrive.tick_log", false);

  // ImageNet statistics on the [0, 1] scale, as the training pipeline applies them after /255.
  const auto mean = node_.declare_parameter<std::vector<double>>(
    "latentdrive.normalization_mean", std::vector<double>{0.485, 0.456, 0.406});
  const auto std_dev = node_.declare_parameter<std::vector<double>>(
    "latentdrive.normalization_std", std::vector<double>{0.229, 0.224, 0.225});
  if (mean.size() != 3 || std_dev.size() != 3) {
    throw std::runtime_error("latentdrive.normalization_mean/std must each have 3 elements (RGB)");
  }
  for (size_t i = 0; i < 3; ++i) {
    if (std_dev[i] <= 0.0) {
      throw std::runtime_error("latentdrive.normalization_std entries must be positive");
    }
    mean_[i] = static_cast<float>(mean[i]);
    inverse_std_[i] = static_cast<float>(1.0 / std_dev[i]);
  }
  if (frame_interval_s_ <= 0.0 || frame_tolerance_s_ < 0.0) {
    throw std::runtime_error(
      "latentdrive.frame_interval_seconds must be positive and "
      "latentdrive.frame_interval_tolerance_seconds non-negative");
  }
  if (subgoal_divisor_ <= 0.0) {
    throw std::runtime_error("latentdrive.status_subgoal_divisor must be positive");
  }
}

std::vector<std::string> LatentDriveInputProvider::claim_inputs(
  const std::vector<TensorSpec> & engine_inputs)
{
  const TensorSpec * video_spec = find_spec(engine_inputs, video_tensor_name_);
  if (!video_spec) {
    throw std::runtime_error(
      "The latentdrive input provider is enabled but the model has no input tensor named '" +
      video_tensor_name_ + "' (set latentdrive.video_tensor to match the model)");
  }
  // Accept [1, 3, T, H, W] or [3, T, H, W].
  const auto & shape = video_spec->shape;
  const bool has_batch = shape.size() == 5;
  if (!(shape.size() == 4 || (has_batch && shape[0] == 1)) || shape[has_batch ? 1 : 0] != 3) {
    throw std::runtime_error(
      "Model input '" + video_tensor_name_ + "' has shape " + shape_to_string(shape) +
      "; expected [1, 3, T, H, W] or [3, T, H, W]");
  }
  const size_t base = has_batch ? 2 : 1;
  video_shape_ = shape;
  num_frames_ = shape[base];
  height_ = shape[base + 1];
  width_ = shape[base + 2];
  if (num_frames_ < 1 || height_ < 1 || width_ < 1) {
    throw std::runtime_error(
      "Model input '" + video_tensor_name_ + "' has shape " + shape_to_string(shape) +
      "; every dimension must be positive");
  }

  const TensorSpec * status_spec = find_spec(engine_inputs, status_tensor_name_);
  if (!status_spec) {
    throw std::runtime_error(
      "The latentdrive input provider is enabled but the model has no input tensor named '" +
      status_tensor_name_ + "' (set latentdrive.status_tensor to match the model)");
  }
  if (status_spec->num_elements() != STATUS_DIM) {
    throw std::runtime_error(
      "Model input '" + status_tensor_name_ + "' has shape " + shape_to_string(status_spec->shape) +
      "; expected [1, " + std::to_string(STATUS_DIM) +
      "] (subgoal x/y, velocity x/y, acceleration x/y)");
  }
  status_shape_ = status_spec->shape;

  video_buffer_.assign(static_cast<size_t>(video_spec->num_elements()), 0.0f);

  const std::string image_topic = "~/input/camera0/image";
  image_sub_ = image_transport::create_subscription(
    &node_, node_.get_node_topics_interface()->resolve_topic_name(image_topic),
    [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) { on_image(msg); }, transport_,
    rmw_qos_profile_sensor_data);
  trajectory_sub_ = node_.create_subscription<Trajectory>(
    "~/input/reference_trajectory", rclcpp::QoS(1), [this](const Trajectory::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_trajectory_ = msg;
    });
  pub_subgoal_ =
    node_.create_publisher<geometry_msgs::msg::PointStamped>("~/debug/latentdrive/subgoal", 1);
  pub_frame_age_ = node_.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/latentdrive/frame_age_ms", 10);
  pub_ego_pose_ =
    node_.create_publisher<geometry_msgs::msg::PoseStamped>("~/debug/latentdrive/ego_pose", 1);

  return {video_tensor_name_, status_tensor_name_};
}

void LatentDriveInputProvider::on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImageConstPtr cv_image;
  try {
    cv_image = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_.get_logger(), *node_.get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Failed to convert the camera image: " << e.what());
    return;
  }

  // Each frame is preprocessed exactly once, here, so the planning tick only stacks them.
  auto chw = std::make_shared<std::vector<float>>(static_cast<size_t>(3 * height_ * width_));
  if (!latentdrive::preprocess_frame(
        cv_image->image, width_, height_, mean_, inverse_std_, chw->data())) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Camera image is empty or not 3-channel; skipping it");
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp);
  autoware_internal_debug_msgs::msg::Float64Stamped frame_age;
  frame_age.stamp = msg->header.stamp;
  frame_age.data = (node_.get_clock()->now() - stamp).seconds() * 1e3;
  pub_frame_age_->publish(frame_age);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!frames_.empty() && (frames_.back().stamp - stamp).seconds() > STAMP_JUMP_BACK_S) {
    RCLCPP_WARN(
      node_.get_logger(), "Camera stamp jumped back %.2f s; restarting the frame window",
      (frames_.back().stamp - stamp).seconds());
    frames_.clear();
  }
  // Keep the deque ordered even if a frame arrives slightly late.
  auto position = frames_.end();
  while (position != frames_.begin() && std::prev(position)->stamp > stamp) {
    --position;
  }
  frames_.insert(position, Frame{stamp, std::move(chw)});

  const double span_s =
    static_cast<double>(num_frames_ - 1) * frame_interval_s_ + frame_tolerance_s_ + PRUNE_MARGIN_S;
  const rclcpp::Time newest = frames_.back().stamp;
  while (!frames_.empty() && (newest - frames_.front().stamp).seconds() > span_s) {
    frames_.pop_front();
  }
}

bool LatentDriveInputProvider::build_video_tensor(
  const rclcpp::Time & now, TensorMap & inputs, std::string & error)
{
  std::vector<Frame> frames;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frames.assign(frames_.begin(), frames_.end());
  }
  if (frames.empty()) {
    error = "No camera image received yet";
    return false;
  }

  const double delay_ms = (now - frames.back().stamp).seconds() * 1e3;
  if (delay_ms > max_delay_ms_) {
    error = "Newest camera image is stale (" + std::to_string(delay_ms) + " ms > " +
            std::to_string(max_delay_ms_) + " ms)";
    return false;
  }

  std::vector<double> stamps;
  stamps.reserve(frames.size());
  for (const auto & frame : frames) {
    stamps.push_back(frame.stamp.seconds());
  }
  const auto slots =
    latentdrive::select_frame_slots(stamps, num_frames_, frame_interval_s_, frame_tolerance_s_);
  if (!slots) {
    error = "Frame window not ready: need " + std::to_string(num_frames_) + " frames " +
            std::to_string(frame_interval_s_) + " s apart, have " + std::to_string(frames.size()) +
            " spanning " + std::to_string(stamps.back() - stamps.front()) + " s";
    return false;
  }

  last_frame_ages_ms_.clear();
  for (const size_t index : *slots) {
    last_frame_ages_ms_.push_back((now - frames[index].stamp).seconds() * 1e3);
  }

  // Layout [3, T, H, W]: channel-major, so each frame contributes one plane per channel.
  const size_t plane = static_cast<size_t>(height_) * static_cast<size_t>(width_);
  for (int64_t t = 0; t < num_frames_; ++t) {
    const std::vector<float> & chw = *frames[(*slots)[t]].chw;
    for (int64_t c = 0; c < 3; ++c) {
      std::memcpy(
        video_buffer_.data() + (c * num_frames_ + t) * plane, chw.data() + c * plane,
        plane * sizeof(float));
    }
  }
  inputs[video_tensor_name_] = Tensor::from_host(video_shape_, video_buffer_);
  return true;
}

bool LatentDriveInputProvider::build_status_tensor(
  const EgoFrame & ego, TensorMap & inputs, std::string & error)
{
  Trajectory::ConstSharedPtr trajectory;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    trajectory = latest_trajectory_;
  }
  if (!trajectory) {
    error = "No reference trajectory received yet";
    return false;
  }
  if (trajectory->points.empty()) {
    error = "Reference trajectory has no points";
    return false;
  }

  std::vector<Eigen::Vector2d> reference;
  reference.reserve(trajectory->points.size());
  for (const auto & point : trajectory->points) {
    reference.emplace_back(point.pose.position.x, point.pose.position.y);
  }
  const auto & ego_position = ego.reference_odometry.pose.pose.position;
  const Eigen::Vector2d subgoal_map = latentdrive::subgoal_along(
    reference, Eigen::Vector2d(ego_position.x, ego_position.y), subgoal_ahead_m_);
  const Eigen::Vector4d subgoal_ego =
    ego.map_to_ego * Eigen::Vector4d(subgoal_map.x(), subgoal_map.y(), 0.0, 1.0);

  const auto & velocity = ego.odometry.twist.twist.linear;
  float acceleration_x = 0.0f;
  float acceleration_y = 0.0f;
  if (ego.acceleration) {
    acceleration_x = static_cast<float>(ego.acceleration->accel.accel.linear.x);
    acceleration_y = static_cast<float>(ego.acceleration->accel.accel.linear.y);
  } else if (!warned_no_acceleration_) {
    RCLCPP_WARN(
      node_.get_logger(),
      "No acceleration received; the status tensor carries zero acceleration until it arrives");
    warned_no_acceleration_ = true;
  }

  // The training-time layout: the subgoal is divided by `d`, the rest passes through.
  last_status_ = {
    static_cast<float>(subgoal_ego.x() / subgoal_divisor_),
    static_cast<float>(subgoal_ego.y() / subgoal_divisor_),
    static_cast<float>(velocity.x),
    static_cast<float>(velocity.y),
    acceleration_x,
    acceleration_y};
  inputs[status_tensor_name_] =
    Tensor::from_host(status_shape_, {last_status_.begin(), last_status_.end()});

  geometry_msgs::msg::PointStamped subgoal_msg;
  subgoal_msg.header.stamp = ego.stamp;
  subgoal_msg.header.frame_id = "map";
  subgoal_msg.point.x = subgoal_map.x();
  subgoal_msg.point.y = subgoal_map.y();
  subgoal_msg.point.z = ego_position.z;
  pub_subgoal_->publish(subgoal_msg);

  // The pose the plan is anchored on: in RViz, an arrow that trails the ego model means the
  // display lags the planner; an arrow on the ego with the trajectory behind it means the plan
  // itself starts behind.
  geometry_msgs::msg::PoseStamped ego_pose_msg;
  ego_pose_msg.header.stamp = ego.stamp;
  ego_pose_msg.header.frame_id = "map";
  ego_pose_msg.pose = ego.reference_odometry.pose.pose;
  pub_ego_pose_->publish(ego_pose_msg);
  return true;
}

bool LatentDriveInputProvider::collect(
  const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs, std::string & error)
{
  // Status before video: it is cheap, and a missing reference should not cost the 6 MB stack.
  const bool ok = build_status_tensor(ego, inputs, error) && build_video_tensor(now, inputs, error);
  if (tick_log_) {
    if (ok) {
      std::ostringstream line;
      line << "[LatentDrive-debug] tick OK | frames before tick:";
      for (const double age : last_frame_ages_ms_) {
        line << " " << std::lround(age);
      }
      line << " ms | v=" << std::fixed << std::setprecision(1) << last_status_[2]
           << " m/s | subgoal=(" << last_status_[0] * subgoal_divisor_ << ", "
           << last_status_[1] * subgoal_divisor_ << ") m";
      RCLCPP_INFO(node_.get_logger(), "%s", line.str().c_str());
    } else {
      RCLCPP_INFO(node_.get_logger(), "[LatentDrive-debug] tick SKIPPED: %s", error.c_str());
    }
  }
  return ok;
}

}  // namespace autoware::tensorrt_e2e
