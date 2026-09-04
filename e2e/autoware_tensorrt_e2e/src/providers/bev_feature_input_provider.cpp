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
#include "autoware/tensorrt_e2e/input_provider_registry.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/diffusion_planner/utils/utils.hpp>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
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

// Fields of the network description are declared the way autoware_bevfusion declares
// its ml_package fields: read-only and without a default, so a model directory that
// lacks one fails at startup instead of voxelizing for some other network.
rcl_interfaces::msg::ParameterDescriptor network_field()
{
  return rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);
}

}  // namespace

BevFeatureInputProvider::BevFeatureInputProvider(rclcpp::Node & node) : node_(node)
{
  history_tensor_name_ =
    node_.declare_parameter<std::string>("bev_feature.history_tensor", "bev_feature_history");
  max_delay_ms_ = node_.declare_parameter<double>("bev_feature.max_delay_ms", 200.0);

  cache_config_.frames = node_.declare_parameter<int64_t>("bev_feature.frames", 3);
  cache_config_.interval_seconds =
    node_.declare_parameter<double>("bev_feature.interval_seconds", 0.2);
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

  // Host-side settings keep defaults; they describe the deployment, not the network.
  extractor_config_.onnx_path =
    node_.declare_parameter<std::string>("bev_feature.extractor.onnx_path", "");
  extractor_config_.engine_path =
    node_.declare_parameter<std::string>("bev_feature.extractor.engine_path", "");
  extractor_config_.plugins_path =
    node_.declare_parameter<std::string>("bev_feature.extractor.plugins_path", "");
  extractor_config_.precision =
    node_.declare_parameter<std::string>("bev_feature.extractor.precision", "fp16");
  extractor_config_.feature_tensor =
    node_.declare_parameter<std::string>("bev_feature.extractor.feature_tensor", "bev_feature");
  extractor_config_.cloud_capacity =
    node_.declare_parameter<int64_t>("bev_feature.extractor.cloud_capacity", 2000000);
  // One workspace setting for both engines; declared by the node before any provider.
  if (node_.has_parameter("trt_workspace_mib")) {
    extractor_config_.max_workspace_size =
      static_cast<size_t>(node_.get_parameter("trt_workspace_mib").as_int()) * 1024ULL *
      1024ULL;
  }

  // Network description: no defaults (see network_field()).
  extractor_config_.max_points_per_voxel = node_.declare_parameter<int64_t>(
    "bev_feature.extractor.max_points_per_voxel", network_field());
  extractor_config_.voxels_num = node_.declare_parameter<std::vector<int64_t>>(
    "bev_feature.extractor.voxels_num", network_field());
  const auto point_cloud_range = node_.declare_parameter<std::vector<double>>(
    "bev_feature.extractor.point_cloud_range", network_field());
  const auto voxel_size =
    node_.declare_parameter<std::vector<double>>("bev_feature.extractor.voxel_size", network_field());
  extractor_config_.point_cloud_range.assign(point_cloud_range.begin(), point_cloud_range.end());
  extractor_config_.voxel_size.assign(voxel_size.begin(), voxel_size.end());
  extractor_config_.use_intensity =
    node_.declare_parameter<bool>("bev_feature.extractor.use_intensity", network_field());

  declare_detection_params();
}

void BevFeatureInputProvider::declare_detection_params()
{
  // The detection head is optional: a model whose extractor graph predates it simply
  // leaves this off, and the provider behaves exactly as before.
  detection_requested_ = node_.declare_parameter<bool>("bev_feature.detection.enabled", false);
  if (!detection_requested_) {
    return;
  }

  // Network description -- the head's own, written into the ml_package file by the
  // exporter, so no defaults here.
  detection_config_.class_names = node_.declare_parameter<std::vector<std::string>>(
    "bev_feature.detection.class_names", network_field());
  detection_config_.score_thresholds = node_.declare_parameter<std::vector<double>>(
    "bev_feature.detection.score_thresholds", network_field());
  extractor_config_.detection.num_proposals =
    node_.declare_parameter<int64_t>("bev_feature.detection.num_proposals", network_field());
  extractor_config_.detection.out_size_factor =
    node_.declare_parameter<int64_t>("bev_feature.detection.out_size_factor", network_field());

  // Host behaviour -- the same knobs, with the same names and defaults, that
  // autoware_bevfusion keeps in its deployment file rather than its ml_package.
  extractor_config_.detection.circle_nms_dist_threshold = static_cast<float>(
    node_.declare_parameter<double>("bev_feature.detection.circle_nms_dist_threshold", 0.5));
  detection_config_.iou_nms_search_distance_2d =
    node_.declare_parameter<double>("bev_feature.detection.iou_nms_search_distance_2d", 10.0);
  detection_config_.iou_nms_threshold =
    node_.declare_parameter<double>("bev_feature.detection.iou_nms_threshold", 0.1);
  extractor_config_.detection.yaw_norm_thresholds =
    node_.declare_parameter<std::vector<double>>(
      "bev_feature.detection.yaw_norm_thresholds", std::vector<double>{});
  detection_config_.allow_remapping_by_area_matrix =
    node_.declare_parameter<std::vector<int64_t>>(
      "bev_feature.detection.allow_remapping_by_area_matrix", std::vector<int64_t>{});
  detection_config_.min_area_matrix = node_.declare_parameter<std::vector<double>>(
    "bev_feature.detection.min_area_matrix", std::vector<double>{});
  detection_config_.max_area_matrix = node_.declare_parameter<std::vector<double>>(
    "bev_feature.detection.max_area_matrix", std::vector<double>{});

  const size_t num_classes = detection_config_.class_names.size();
  if (extractor_config_.detection.yaw_norm_thresholds.empty()) {
    // The decode kernel indexes this by class; a zero floor keeps every box, which is
    // what autoware_bevfusion configures for the classes it does not gate.
    extractor_config_.detection.yaw_norm_thresholds.assign(num_classes, 0.0);
  } else if (extractor_config_.detection.yaw_norm_thresholds.size() != num_classes) {
    throw std::runtime_error(
      "bev_feature.detection.yaw_norm_thresholds has " +
      std::to_string(extractor_config_.detection.yaw_norm_thresholds.size()) +
      " entries but the head has " + std::to_string(num_classes) + " classes");
  }

  // PostprocessCuda holds ONE score threshold, the head states one per class: the device
  // filter runs at the lowest and DetectionPostprocessor applies the per-class cut, so
  // the two together are the head's own thresholds and nothing is dropped early.
  extractor_config_.detection.score_threshold =
    detection_config_.score_thresholds.empty()
      ? 0.0f
      : static_cast<float>(*std::min_element(
          detection_config_.score_thresholds.begin(), detection_config_.score_thresholds.end()));
  extractor_config_.detection.enabled = true;
}

BevFeatureInputProvider::~BevFeatureInputProvider()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
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

  if (detection_requested_) {
    if (extractor_->detection_enabled()) {
      detection_postprocessor_ = std::make_unique<DetectionPostprocessor>(detection_config_);
      detected_objects_pub_ =
        node_.create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
          "~/output/detected_objects", rclcpp::QoS(1));
      RCLCPP_INFO_STREAM(
        node_.get_logger(), "BEV feature extractor detection head enabled: "
                              << detection_config_.class_names.size() << " classes, "
                              << extractor_config_.detection.num_proposals << " proposals");
    } else {
      // Configured on, but this graph has no head. That is a stale artifact next to a
      // current config, and silently publishing nothing would look like a dead detector.
      RCLCPP_WARN(
        node_.get_logger(),
        "bev_feature.detection is enabled but the extractor graph (%s) has no '%s'/'%s'/'%s' "
        "outputs; no objects will be published",
        extractor_config_.onnx_path.c_str(), extractor_config_.detection.bbox_tensor.c_str(),
        extractor_config_.detection.score_tensor.c_str(),
        extractor_config_.detection.label_tensor.c_str());
    }
  }

  pointcloud_sub_ = std::make_unique<
    cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
    node_, "~/input/pointcloud",
    [this](std::shared_ptr<const cuda_blackboard::CudaPointCloud2> msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_pointcloud_ = std::move(msg);
    });

  return {history_tensor_name_};
}

bool BevFeatureInputProvider::collect(
  const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs, std::string & error)
{
  std::shared_ptr<const cuda_blackboard::CudaPointCloud2> cloud;
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
    // The boxes belong to this cloud, so they are published once per LiDAR frame rather
    // than once per planning tick -- republishing the same detection at 10 Hz would give
    // a consumer a false sense of a fresh measurement.
    if (detection_postprocessor_) {
      const auto objects = detection_postprocessor_->build(
        extractor_->last_detections(), cloud->header);
      last_detected_object_count_ = objects.objects.size();
      detected_objects_pub_->publish(objects);
    }
    last_extracted_stamp_ = cloud_stamp;
    history_ptr_ = nullptr;
  }

  if (!cache_->ready()) {
    error = "BEV feature history incomplete (" + std::to_string(cache_->cached_frames()) +
            " maps cached, need " + std::to_string(cache_->frames()) +
            " at the configured interval)";
    return false;
  }

  if (!history_ptr_) {
    history_ptr_ = cache_->build_history(stream_);
  }
  inputs[history_tensor_name_] = Tensor::from_device(history_shape_, history_ptr_);
  return true;
}

TENSORRT_E2E_REGISTER_INPUT_PROVIDER(
  "bev_feature", [](rclcpp::Node & node, tf2_ros::Buffer &) {
    return std::make_unique<BevFeatureInputProvider>(node);
  });

}  // namespace autoware::tensorrt_e2e
