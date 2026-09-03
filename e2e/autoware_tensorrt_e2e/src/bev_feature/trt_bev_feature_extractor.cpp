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

#include "autoware/tensorrt_e2e/bev_feature/trt_bev_feature_extractor.hpp"

#include "autoware/tensorrt_e2e/types.hpp"

#include <autoware/bevfusion/preprocess/point_type.hpp>
#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <NvInfer.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_e2e
{

using autoware::bevfusion::BEVFusionConfig;
using autoware::bevfusion::InputPointType;
using autoware::tensorrt_common::NetworkIO;
using autoware::tensorrt_common::ProfileDims;
using autoware::tensorrt_common::Profiler;
using autoware::tensorrt_common::TrtCommon;
using autoware::tensorrt_common::TrtCommonConfig;

namespace
{

BEVFusionConfig make_lidar_only_config(const TrtBevFeatureExtractor::Config & config)
{
  if (config.voxels_num.size() != 3) {
    throw std::runtime_error("bev_feature.extractor.voxels_num must have 3 elements [min,opt,max]");
  }
  if (config.point_cloud_range.size() != 6) {
    throw std::runtime_error("bev_feature.extractor.point_cloud_range must have 6 elements");
  }
  if (config.voxel_size.size() != 3) {
    throw std::runtime_error("bev_feature.extractor.voxel_size must have 3 elements");
  }
  // Empty image-backbone paths select the lidar-only mode; camera and postprocess parameters
  // are unused on this path and passed as neutral values.
  return BEVFusionConfig(
    config.plugins_path, "", "", "",
    /*out_size_factor=*/8, config.cloud_capacity, config.max_points_per_voxel, config.voxels_num,
    config.point_cloud_range, config.voxel_size, /*d_bound=*/{}, /*x_bound=*/{}, /*y_bound=*/{},
    /*z_bound=*/{}, /*num_cameras=*/0, /*raw_image_height=*/0, /*raw_image_width=*/0,
    /*img_aug_scale_x=*/0.0f, /*img_aug_scale_y=*/0.0f, /*roi_height=*/0, /*roi_width=*/0,
    /*features_height=*/0, /*features_width=*/0, /*num_depth_features=*/0,
    /*image_feature_channel=*/0, /*num_proposals=*/0, /*circle_nms_dist_threshold=*/0.0f,
    /*yaw_norm_thresholds=*/{}, /*score_threshold=*/0.0f, config.use_intensity);
}

}  // namespace

TrtBevFeatureExtractor::TrtBevFeatureExtractor(const Config & config, cudaStream_t stream)
: bevfusion_config_(make_lidar_only_config(config)),
  feature_tensor_(config.feature_tensor),
  stream_(stream)
{
  preprocess_ =
    std::make_unique<autoware::bevfusion::PreprocessCuda>(bevfusion_config_, stream_, true);

  points_d_ = autoware::cuda_utils::make_unique<float[]>(
    static_cast<size_t>(bevfusion_config_.cloud_capacity_) *
    bevfusion_config_.num_point_feature_size_);
  voxel_features_d_ = autoware::cuda_utils::make_unique<float[]>(
    static_cast<size_t>(bevfusion_config_.max_num_voxels_) *
    bevfusion_config_.max_points_per_voxel_ * bevfusion_config_.num_point_feature_size_);
  voxel_coords_d_ = autoware::cuda_utils::make_unique<int32_t[]>(
    static_cast<size_t>(bevfusion_config_.max_num_voxels_) * 3);
  num_points_per_voxel_d_ = autoware::cuda_utils::make_unique<int32_t[]>(
    static_cast<size_t>(bevfusion_config_.max_num_voxels_));

  // The single sweep is already in the target frame: identity transform, zero time lag.
  const std::vector<float> identity = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                       0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  identity_transform_d_ = autoware::cuda_utils::make_unique<float[]>(identity.size());
  CHECK_CUDA_ERROR(cudaMemcpy(
    identity_transform_d_.get(), identity.data(), identity.size() * sizeof(float),
    cudaMemcpyHostToDevice));

  init_engine(config);
}

void TrtBevFeatureExtractor::init_engine(const Config & config)
{
  const auto trt_config = TrtCommonConfig(
    config.onnx_path, config.precision, config.engine_path, config.max_workspace_size);

  // Same lidar-branch IO contract as autoware_bevfusion: dynamic voxel count with a
  // min/opt/max optimization profile.
  const int64_t max_ppv = bevfusion_config_.max_points_per_voxel_;
  const int64_t feature_size = bevfusion_config_.num_point_feature_size_;
  const auto voxel_dims = [max_ppv, feature_size](const int64_t count) {
    return nvinfer1::Dims{3, {count, max_ppv, feature_size}};
  };

  std::vector<ProfileDims> profile_dims;
  profile_dims.emplace_back(
    "voxels", voxel_dims(config.voxels_num[0]), voxel_dims(config.voxels_num[1]),
    voxel_dims(config.voxels_num[2]));
  profile_dims.emplace_back(
    "num_points_per_voxel", nvinfer1::Dims{1, {config.voxels_num[0]}},
    nvinfer1::Dims{1, {config.voxels_num[1]}}, nvinfer1::Dims{1, {config.voxels_num[2]}});
  profile_dims.emplace_back(
    "coors", nvinfer1::Dims{2, {config.voxels_num[0], 3}},
    nvinfer1::Dims{2, {config.voxels_num[1], 3}}, nvinfer1::Dims{2, {config.voxels_num[2], 3}});

  trt_common_ = std::make_unique<TrtCommon>(
    trt_config, std::make_shared<Profiler>(), std::vector<std::string>{config.plugins_path});
  auto profile_dims_ptr = std::make_unique<std::vector<ProfileDims>>(profile_dims);
  if (!trt_common_->setup(std::move(profile_dims_ptr))) {
    throw std::runtime_error(
      "Failed to setup the BEV feature extractor engine from " + config.onnx_path);
  }

  // The feature output must have static dimensions (the sparse voxel count only affects the
  // engine's internal tensors; the scattered BEV grid is fixed by the model).
  const nvinfer1::Dims feature_dims = trt_common_->getTensorShape(feature_tensor_.c_str());
  std::vector<int64_t> dims;
  for (int32_t i = 0; i < feature_dims.nbDims; ++i) {
    dims.push_back(feature_dims.d[i]);
  }
  const bool rank4 = dims.size() == 4 && dims[0] == 1;
  const bool rank3 = dims.size() == 3;
  if (!(rank3 || rank4) || std::any_of(dims.begin(), dims.end(), [](int64_t d) { return d < 1; })) {
    throw std::runtime_error(
      "BEV feature extractor output '" + feature_tensor_ +
      "' must have a static [1, C, H, W] or [C, H, W] shape; got " + shape_to_string(dims));
  }
  const size_t base = rank4 ? 1 : 0;
  channels_ = dims[base];
  height_ = dims[base + 1];
  width_ = dims[base + 2];

  feature_d_ = autoware::cuda_utils::make_unique<float[]>(
    static_cast<size_t>(channels_) * height_ * width_);

  trt_common_->setTensorAddress("voxels", voxel_features_d_.get());
  trt_common_->setTensorAddress("num_points_per_voxel", num_points_per_voxel_d_.get());
  trt_common_->setTensorAddress("coors", voxel_coords_d_.get());
  trt_common_->setTensorAddress(feature_tensor_.c_str(), feature_d_.get());
}

bool TrtBevFeatureExtractor::validate_cloud_layout(
  const sensor_msgs::msg::PointCloud2 & cloud, std::string & error) const
{
  if (cloud.point_step != sizeof(InputPointType)) {
    error = "Point cloud point_step is " + std::to_string(cloud.point_step) + ", expected " +
            std::to_string(sizeof(InputPointType)) +
            " (Autoware PointXYZIRC layout, as consumed by BEVFusion)";
    return false;
  }
  const auto has_field = [&cloud](const std::string & name, const uint32_t offset) {
    return std::any_of(
      cloud.fields.begin(), cloud.fields.end(),
      [&](const sensor_msgs::msg::PointField & field) {
        return field.name == name && field.offset == offset;
      });
  };
  if (!has_field("x", 0) || !has_field("y", 4) || !has_field("z", 8) ||
      !has_field("intensity", 12)) {
    error = "Point cloud fields do not match the Autoware PointXYZIRC layout";
    return false;
  }
  return true;
}

const float * TrtBevFeatureExtractor::extract(
  const cuda_blackboard::CudaPointCloud2 & cloud, std::string & error)
{
  if (!validate_cloud_layout(cloud, error)) {
    return nullptr;
  }

  size_t num_points = static_cast<size_t>(cloud.width) * cloud.height;
  if (num_points == 0) {
    error = "Point cloud is empty";
    return nullptr;
  }
  num_points =
    std::min(num_points, static_cast<size_t>(bevfusion_config_.cloud_capacity_));

  // The cloud already lives on the device (cuda_blackboard); no staging copy.
  CHECK_CUDA_ERROR(preprocess_->generateSweepPoints_launch(
    reinterpret_cast<const InputPointType *>(cloud.data.get()), num_points,
    /*time_lag=*/0.0f, identity_transform_d_.get(), points_d_.get()));

  const auto num_voxels = static_cast<int64_t>(preprocess_->generateVoxels(
    points_d_.get(), static_cast<unsigned int>(num_points), voxel_features_d_.get(),
    voxel_coords_d_.get(), num_points_per_voxel_d_.get()));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  last_num_voxels_ = num_voxels;
  // Same policy as autoware_bevfusion: below the profile minimum the frame cannot run;
  // above the maximum it runs on a clipped voxel set and diagnostics say so.
  last_voxels_within_range_ = num_voxels < static_cast<int64_t>(bevfusion_config_.max_num_voxels_);
  if (num_voxels < bevfusion_config_.min_num_voxels_) {
    error = "Too few voxels (" + std::to_string(num_voxels) +
            ") for the optimization profile minimum (" +
            std::to_string(bevfusion_config_.min_num_voxels_) + ")";
    return nullptr;
  }
  const auto clamped_voxels =
    std::min(num_voxels, static_cast<int64_t>(bevfusion_config_.max_num_voxels_));

  const int64_t max_ppv = bevfusion_config_.max_points_per_voxel_;
  const int64_t feature_size = bevfusion_config_.num_point_feature_size_;
  trt_common_->setInputShape(
    "voxels", nvinfer1::Dims{3, {clamped_voxels, max_ppv, feature_size}});
  trt_common_->setInputShape("num_points_per_voxel", nvinfer1::Dims{1, {clamped_voxels}});
  trt_common_->setInputShape("coors", nvinfer1::Dims{2, {clamped_voxels, 3}});

  if (!trt_common_->enqueueV3(stream_)) {
    error = "Failed to enqueue the BEV feature extractor engine";
    return nullptr;
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  return feature_d_.get();
}

}  // namespace autoware::tensorrt_e2e
