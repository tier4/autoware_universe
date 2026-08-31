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

#include "autoware/tensorrt_e2e/providers/bev_feature_input_provider.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/diffusion_planner/utils/utils.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace dp = autoware::diffusion_planner;

namespace
{
constexpr int64_t LOG_THROTTLE_INTERVAL_MS = 5000;

std::array<double, 4> pose_from_odometry(const nav_msgs::msg::Odometry & odometry)
{
  const Eigen::Matrix4d pose_matrix = dp::utils::pose_to_matrix4d(odometry.pose.pose);
  const auto [cos_yaw, sin_yaw] =
    dp::utils::rotation_matrix_to_cos_sin(pose_matrix.block<3, 3>(0, 0));
  // Double throughout: map coordinates are ~1e5 m, and the warp needs the metre-scale pose
  // DIFFERENCE — a float cast here quantizes slow-speed inter-frame displacements away.
  return {
    odometry.pose.pose.position.x, odometry.pose.pose.position.y,
    static_cast<double>(cos_yaw), static_cast<double>(sin_yaw)};
}

}  // namespace

BevFeatureInputProvider::BevFeatureInputProvider(rclcpp::Node & node) : node_(node)
{
  history_tensor_name_ =
    node_.declare_parameter<std::string>("bev_feature.history_tensor", "bev_feature_history");
  max_delay_ms_ = node_.declare_parameter<double>("bev_feature.max_delay_ms", 200.0);

  cache_config_.frames = node_.declare_parameter<int64_t>("bev_feature.frames", 3);
  cache_config_.interval_seconds =
    node_.declare_parameter<double>("bev_feature.interval_seconds", 0.1);
  cache_config_.interval_tolerance_seconds =
    node_.declare_parameter<double>("bev_feature.interval_tolerance_seconds", 0.02);
  cache_config_.bev_half_extent_m =
    node_.declare_parameter<double>("bev_feature.bev_half_extent_m", 122.4);
  const auto warmup = node_.declare_parameter<std::string>("bev_feature.warmup", "wait");
  if (warmup != "wait" && warmup != "duplicate_current") {
    throw std::runtime_error(
      "bev_feature.warmup must be 'wait' or 'duplicate_current', got '" + warmup + "'");
  }
  cache_config_.duplicate_current_on_warmup = warmup == "duplicate_current";

  extractor_config_.onnx_path =
    node_.declare_parameter<std::string>("bev_feature.extractor.onnx_path", "");
  extractor_config_.plugins_path =
    node_.declare_parameter<std::string>("bev_feature.extractor.plugins_path", "");
  extractor_config_.precision =
    node_.declare_parameter<std::string>("bev_feature.extractor.precision", "fp16");
  extractor_config_.feature_tensor =
    node_.declare_parameter<std::string>("bev_feature.extractor.feature_tensor", "bev_feature");
  extractor_config_.cloud_capacity =
    node_.declare_parameter<int64_t>("bev_feature.extractor.cloud_capacity", 2000000);
  extractor_config_.max_points_per_voxel =
    node_.declare_parameter<int64_t>("bev_feature.extractor.max_points_per_voxel", 10);
  extractor_config_.voxels_num = node_.declare_parameter<std::vector<int64_t>>(
    "bev_feature.extractor.voxels_num", std::vector<int64_t>{1, 128000, 256000});
  const auto point_cloud_range = node_.declare_parameter<std::vector<double>>(
    "bev_feature.extractor.point_cloud_range",
    std::vector<double>{-122.4, -122.4, -3.0, 122.4, 122.4, 5.0});
  const auto voxel_size = node_.declare_parameter<std::vector<double>>(
    "bev_feature.extractor.voxel_size", std::vector<double>{0.17, 0.17, 0.2});
  extractor_config_.point_cloud_range.assign(point_cloud_range.begin(), point_cloud_range.end());
  extractor_config_.voxel_size.assign(voxel_size.begin(), voxel_size.end());
  extractor_config_.use_intensity =
    node_.declare_parameter<bool>("bev_feature.extractor.use_intensity", false);

  const auto contract_path =
    node_.declare_parameter<std::string>("bev_feature.contract_path", "");
  if (!contract_path.empty()) {
    load_contract(contract_path);
  }
}

BevFeatureInputProvider::~BevFeatureInputProvider()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

void BevFeatureInputProvider::load_contract(const std::string & contract_path)
{
  std::ifstream file(contract_path);
  if (!file) {
    throw std::runtime_error("Could not open the deployment contract: " + contract_path);
  }
  nlohmann::json contract;
  file >> contract;

  // The contract JSON shipped with the model is the source of truth for the temporal-cache
  // semantics; ROS parameters remain the fallback for fields it does not carry.
  if (contract.contains("temporal_cache")) {
    const auto & cache = contract.at("temporal_cache");
    if (cache.contains("frames")) {
      cache_config_.frames = cache.at("frames").get<int64_t>();
    }
    if (cache.contains("interval_seconds")) {
      cache_config_.interval_seconds = cache.at("interval_seconds").get<double>();
    }
    if (cache.contains("alignment") && cache.at("alignment").contains("bev_half_extent_m")) {
      cache_config_.bev_half_extent_m =
        cache.at("alignment").at("bev_half_extent_m").get<double>();
    }
  }
  if (contract.contains("bevfusion") && contract.at("bevfusion").contains("output_tensor")) {
    extractor_config_.feature_tensor =
      contract.at("bevfusion").at("output_tensor").get<std::string>();
  }
  RCLCPP_INFO(
    node_.get_logger(),
    "Loaded deployment contract %s (frames=%ld, interval=%.3fs, half_extent=%.1fm)",
    contract_path.c_str(), cache_config_.frames, cache_config_.interval_seconds,
    cache_config_.bev_half_extent_m);
}

std::vector<std::string> BevFeatureInputProvider::claim_inputs(
  const std::vector<TensorSpec> & engine_inputs)
{
  const TensorSpec * spec = find_spec(engine_inputs, history_tensor_name_);
  if (!spec) {
    throw std::runtime_error(
      "The BEV feature input provider is enabled but the model has no input tensor named '" +
      history_tensor_name_ + "' (set bev_feature.history_tensor to match the model)");
  }

  // Accept [1, K, C, H, W] or [K, C, H, W].
  const auto & shape = spec->shape;
  const bool has_batch = shape.size() == 5;
  if (!(shape.size() == 4 || (has_batch && shape[0] == 1))) {
    throw std::runtime_error(
      "Model input '" + history_tensor_name_ + "' has shape " + shape_to_string(shape) +
      "; expected [1, K, C, H, W] or [K, C, H, W]");
  }
  const size_t base = has_batch ? 1 : 0;
  if (shape[base] != cache_config_.frames) {
    throw std::runtime_error(
      "Model input '" + history_tensor_name_ + "' has " + std::to_string(shape[base]) +
      " temporal frames, but the cache is configured for " + std::to_string(cache_config_.frames));
  }
  history_shape_ = shape;

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
  try {
    extractor_ = std::make_unique<TrtBevFeatureExtractor>(extractor_config_, stream_);
  } catch (const std::exception & e) {
    throw std::runtime_error(
      "Failed to create the BEV feature extractor (bev_feature.extractor.onnx_path: '" +
      extractor_config_.onnx_path + "'): " + e.what());
  }
  if (
    extractor_->channels() != shape[base + 1] || extractor_->height() != shape[base + 2] ||
    extractor_->width() != shape[base + 3]) {
    throw std::runtime_error(
      "The BEV feature extractor produces [" + std::to_string(extractor_->channels()) + ", " +
      std::to_string(extractor_->height()) + ", " + std::to_string(extractor_->width()) +
      "] maps, but the planner expects '" + history_tensor_name_ + "' with shape " +
      shape_to_string(shape));
  }
  cache_ = std::make_unique<TemporalBevCache>(
    cache_config_, extractor_->channels(), extractor_->height(), extractor_->width());

  pointcloud_sub_ = node_.create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{},
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_pointcloud_ = msg;
    });

  return {history_tensor_name_};
}

bool BevFeatureInputProvider::collect(
  const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs, std::string & error)
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud = latest_pointcloud_;
  }
  if (!cloud) {
    error = "No point cloud received yet";
    return false;
  }

  const rclcpp::Time cloud_stamp(cloud->header.stamp);
  const double delay_ms = (now - cloud_stamp).seconds() * 1e3;
  if (delay_ms > max_delay_ms_) {
    error = "Point cloud is stale (" + std::to_string(delay_ms) + " ms > " +
            std::to_string(max_delay_ms_) + " ms)";
    return false;
  }

  // The extractor runs once per new LiDAR frame; between frames the assembled history is
  // reused (it stays anchored at the newest frame's ego pose, as in the reference cache).
  if (!last_extracted_stamp_ || cloud_stamp != *last_extracted_stamp_) {
    const float * feature = extractor_->extract(*cloud, error);
    if (!feature) {
      return false;
    }
    // The BEV feature lives in the base_link frame of its source LiDAR frame; the pose is
    // sampled from the newest odometry (the stamps differ by at most one sensor period).
    const auto insert_result =
      cache_->insert(feature, pose_from_odometry(ego.odometry), cloud_stamp, stream_);
    if (insert_result == TemporalBevCache::InsertResult::kGapReset) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), LOG_THROTTLE_INTERVAL_MS,
        "LiDAR timestamps went backwards (time jump or bag loop); BEV feature cache reset");
    }
    last_extracted_stamp_ = cloud_stamp;
    history_ptr_ = nullptr;
  }

  if (!cache_->ready()) {
    error = "BEV feature history incomplete (" + std::to_string(cache_->cached_frames()) +
            " maps cached, need " + std::to_string(cache_->frames()) +
            " at the contract interval)";
    return false;
  }

  if (!history_ptr_) {
    history_ptr_ = cache_->build_history(stream_);
  }
  inputs[history_tensor_name_] = Tensor::from_device(history_shape_, history_ptr_);
  return true;
}

}  // namespace autoware::tensorrt_e2e
