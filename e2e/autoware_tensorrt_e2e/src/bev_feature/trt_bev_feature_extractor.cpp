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
  // Empty image-backbone paths select the lidar-only mode; the camera parameters are
  // unused on this path and passed as neutral values. The postprocess parameters are
  // real whenever the graph carries the detection head -- `PostprocessCuda` decodes
  // proposals out of exactly this config.
  const auto & detection = config.detection;
  return BEVFusionConfig(
    config.plugins_path, "", "", "", detection.out_size_factor, config.cloud_capacity,
    config.max_points_per_voxel, config.voxels_num, config.point_cloud_range, config.voxel_size,
    /*d_bound=*/{}, /*x_bound=*/{}, /*y_bound=*/{},
    /*z_bound=*/{}, /*num_cameras=*/0, /*raw_image_height=*/0, /*raw_image_width=*/0,
    /*img_aug_scale_x=*/0.0f, /*img_aug_scale_y=*/0.0f, /*roi_height=*/0, /*roi_width=*/0,
    /*features_height=*/0, /*features_width=*/0, /*num_depth_features=*/0,
    /*image_feature_channel=*/0, detection.num_proposals, detection.circle_nms_dist_threshold,
    detection.yaw_norm_thresholds, detection.score_threshold, config.use_intensity);
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
  bound_outputs_.push_back(feature_tensor_);

  init_detection(config);
  bind_unread_outputs();
}

namespace
{
size_t element_size(const nvinfer1::DataType dtype)
{
  switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
    case nvinfer1::DataType::kINT32:
      return 4;
    case nvinfer1::DataType::kHALF:
    case nvinfer1::DataType::kBF16:
      return 2;
    case nvinfer1::DataType::kINT64:
      return 8;
    case nvinfer1::DataType::kINT8:
    case nvinfer1::DataType::kUINT8:
    case nvinfer1::DataType::kBOOL:
    case nvinfer1::DataType::kFP8:
      return 1;
    default:
      return 0;
  }
}
}  // namespace

void TrtBevFeatureExtractor::bind_unread_outputs()
{
  // enqueueV3 requires an address for EVERY output the engine declares, not just the
  // ones this node reads. The extractor graph carries a detection head whether or not
  // the deployment asked for boxes, so with the head switched off its three outputs
  // were left unbound and every inference failed with "Neither address or allocator is
  // set for output tensor bbox_pred" -- no BEV feature, no trajectory, and nothing in
  // the node's own logs to say why. Give the leftovers scratch buffers and ignore them.
  const int32_t num_io = trt_common_->getNbIOTensors();
  for (int32_t i = 0; i < num_io; ++i) {
    const char * name = trt_common_->getIOTensorName(i);
    if (trt_common_->getTensorIOMode(name) != nvinfer1::TensorIOMode::kOUTPUT) {
      continue;
    }
    if (
      std::find(bound_outputs_.begin(), bound_outputs_.end(), std::string(name)) !=
      bound_outputs_.end()) {
      continue;
    }

    const nvinfer1::Dims dims = trt_common_->getTensorShape(name);
    size_t count = 1;
    for (int32_t d = 0; d < dims.nbDims; ++d) {
      // A dynamic extent would make the size unknowable here; the extractor's profile
      // is static, so treat one as a graph this node cannot safely run.
      if (dims.d[d] < 0) {
        throw std::runtime_error(
          "The BEV feature extractor's output '" + std::string(name) +
          "' has a dynamic extent; its size cannot be bound ahead of inference");
      }
      count *= static_cast<size_t>(dims.d[d]);
    }
    const auto dtype = trt_common_->getTensorDataType(name);
    const size_t bytes = count * (dtype ? element_size(*dtype) : 0);
    if (bytes == 0) {
      throw std::runtime_error(
        "The BEV feature extractor's output '" + std::string(name) +
        "' has an unsupported data type; it cannot be given a buffer");
    }

    unread_output_scratch_.push_back(autoware::cuda_utils::make_unique<uint8_t[]>(bytes));
    trt_common_->setTensorAddress(name, unread_output_scratch_.back().get());
    bound_outputs_.emplace_back(name);
  }
}

void TrtBevFeatureExtractor::init_detection(const Config & config)
{
  const auto & detection = config.detection;
  if (!detection.enabled) {
    return;
  }

  // A graph exported before the head was added is still a valid extractor: say the head
  // is absent and run without it, rather than failing a deployment that never asked for
  // boxes. A graph that HAS the tensors but disagrees on their shape is a real mismatch.
  std::vector<std::string> engine_tensors;
  const int32_t num_io = trt_common_->getNbIOTensors();
  engine_tensors.reserve(static_cast<size_t>(num_io));
  for (int32_t i = 0; i < num_io; ++i) {
    engine_tensors.emplace_back(trt_common_->getIOTensorName(i));
  }
  const auto has_tensor = [&engine_tensors](const std::string & name) {
    return std::find(engine_tensors.begin(), engine_tensors.end(), name) != engine_tensors.end();
  };
  const bool complete = has_tensor(detection.bbox_tensor) && has_tensor(detection.score_tensor) &&
                        has_tensor(detection.label_tensor);
  if (!complete) {
    return;
  }

  if (detection.num_proposals <= 0) {
    throw std::runtime_error(
      "bev_feature.detection is enabled but num_proposals is " +
      std::to_string(detection.num_proposals));
  }
  const auto class_count = static_cast<int64_t>(detection.yaw_norm_thresholds.size());
  if (class_count <= 0) {
    throw std::runtime_error(
      "bev_feature.detection.yaw_norm_thresholds is empty; the decode kernel indexes it by "
      "class and would read out of bounds");
  }

  // The decode kernel reads bbox_pred as [num_box_values, num_proposals] and the other two
  // as [num_proposals]; a graph whose head was exported with a different proposal count
  // would be read at the wrong stride.
  const auto expect = [this](const std::string & name, const std::vector<int64_t> & want) {
    const nvinfer1::Dims dims = trt_common_->getTensorShape(name.c_str());
    std::vector<int64_t> got;
    for (int32_t i = 0; i < dims.nbDims; ++i) {
      got.push_back(dims.d[i]);
    }
    if (got != want) {
      throw std::runtime_error(
        "The BEV feature extractor's detection output '" + name + "' has shape " +
        shape_to_string(got) + ", expected " + shape_to_string(want) +
        " (bev_feature.detection.num_proposals disagrees with the graph)");
    }
  };
  const int64_t proposals = detection.num_proposals;
  const int64_t box_values = bevfusion_config_.num_box_values_;
  expect(detection.bbox_tensor, {box_values, proposals});
  expect(detection.score_tensor, {proposals});
  expect(detection.label_tensor, {proposals});

  bbox_pred_d_ =
    autoware::cuda_utils::make_unique<float[]>(static_cast<size_t>(box_values) * proposals);
  score_d_ = autoware::cuda_utils::make_unique<float[]>(static_cast<size_t>(proposals));
  label_pred_d_ = autoware::cuda_utils::make_unique<int64_t[]>(static_cast<size_t>(proposals));

  trt_common_->setTensorAddress(detection.bbox_tensor.c_str(), bbox_pred_d_.get());
  trt_common_->setTensorAddress(detection.score_tensor.c_str(), score_d_.get());
  trt_common_->setTensorAddress(detection.label_tensor.c_str(), label_pred_d_.get());
  bound_outputs_.push_back(detection.bbox_tensor);
  bound_outputs_.push_back(detection.score_tensor);
  bound_outputs_.push_back(detection.label_tensor);

  postprocess_ =
    std::make_unique<autoware::bevfusion::PostprocessCuda>(bevfusion_config_, stream_);
  last_detections_.reserve(static_cast<size_t>(proposals));
  detection_enabled_ = true;
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

  if (detection_enabled_) {
    // autoware_bevfusion's own decode: TransFusion coder, score cut, circle NMS. The
    // boxes come back on the host, already sorted by score.
    last_detections_.clear();
    CHECK_CUDA_ERROR(postprocess_->generateDetectedBoxes3D_launch(
      label_pred_d_.get(), bbox_pred_d_.get(), score_d_.get(), last_detections_, stream_));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  }
  return feature_d_.get();
}

}  // namespace autoware::tensorrt_e2e
