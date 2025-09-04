// Copyright 2025 Shin-kyoto.
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

#ifndef AUTOWARE_TENSORRT_VAD_VAD_TRT_HPP_
#define AUTOWARE_TENSORRT_VAD_VAD_TRT_HPP_

#include <optional>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>
#include <array>
#include <cuda_runtime.h>
#include <NvInfer.h>
#include <dlfcn.h>
#include "networks/net.hpp"
#include "networks/backbone.hpp"
#include "networks/head.hpp"
#include <map>

#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include "ros_vad_logger.hpp"
#include "vad_config.hpp"

namespace autoware::tensorrt_vad
{

/**
 * @brief Structure representing predicted trajectory
 */
struct PredictedTrajectory {
    std::array<std::array<float, 2>, 6> trajectory;  // 6 time steps × 2 coordinates (x, y)
    float confidence;                                 // Confidence of this trajectory
    
    PredictedTrajectory() : confidence(0.0f) {
        // Initialize trajectory coordinates to 0
        for (int32_t i = 0; i < 6; ++i) {
            trajectory[i][0] = 0.0f;
            trajectory[i][1] = 0.0f;
        }
    }
};

/**
 * @brief each map polyline type and its points
 */
struct MapPolyline {
    std::string type;                               // polyline type ("divider", "ped_crossing", "boundary")
    std::vector<std::vector<float>> points;         // polyline points (each point has [x, y])

    MapPolyline() = default;
    
    MapPolyline(const std::string& map_type, const std::vector<std::vector<float>>& map_points)
        : type(map_type), points(map_points) {}
};

/**
 * @brief Structure representing bounding box and its predicted trajectory
 */
struct BBox {
    std::array<float, 10> bbox;                      // [c_x, c_y, w, l, c_z, h, sin(theta), cos(theta), v_x, v_y]
    float confidence;                                // Object confidence
    int32_t object_class;                           // Object class (0-9)
    std::array<PredictedTrajectory, 6> trajectories; // 6 predicted trajectories
    
    BBox() : confidence(0.0f), object_class(-1) {
        // Initialize bbox coordinates to 0
        for (int32_t i = 0; i < 10; ++i) {
            bbox[i] = 0.0f;
        }
    }
};

// VAD inference input data structure
struct VadInputData
{
  // Camera image data (multi-camera support)
  std::vector<float> camera_images_;

  // Shift information (img_metas.0[shift])
  std::vector<float> shift_;

  // Transform matrix from coordinate used in VAD inference to camera image coordinate system (img_metas.0[lidar2img])
  std::vector<float> vad_base2img_;

  // CAN-BUS data (vehicle state information: velocity, angular velocity, etc.) (img_metas.0[can_bus])
  std::vector<float> can_bus_;

  // Command index (for trajectory selection)
  int32_t command_{2};
};

// VAD inference output data structure
struct VadOutputData
{
  // Predicted trajectory (6 2D coordinate points, expressed as cumulative coordinates)
  // planning[0,1] = 1st point (x,y), planning[2,3] = 2nd point (x,y), ...
  std::vector<float> predicted_trajectory_{};  // size: 12 (6 points * 2 coordinates)

  // Map of predicted trajectories for multiple commands
  // key: command index (int32_t), value: trajectory (std::vector<float>)
  std::map<int32_t, std::vector<float>> predicted_trajectories_{};

  // map polylines (each polyline has map_type and points)
  std::vector<MapPolyline> map_polylines_{};

  // Predicted objects
  std::vector<BBox> predicted_objects_{};
};

// Post-processing functions

// Helper functions for map prediction processing
std::vector<std::vector<float>> process_map_class_scores(const std::vector<float>& cls_preds_flat, const VadConfig& vad_config);
std::vector<std::vector<std::vector<float>>> process_map_points(const std::vector<float>& pts_preds_flat, const VadConfig& vad_config);
std::vector<MapPolyline>
select_confident_map_polylines(
    const std::vector<std::vector<float>>& cls_scores,
    const std::vector<std::vector<std::vector<float>>>& pts_preds,
    const VadConfig& vad_config);

std::vector<MapPolyline> postprocess_map_preds(
    const std::vector<float>& all_map_cls_preds_flat,
    const std::vector<float>& all_map_pts_preds_flat,
    const VadConfig& vad_config);

// Helper functions for trajectory prediction processing
std::vector<std::vector<float>> postprocess_class_scores(const std::vector<float>& all_cls_scores_flat, const VadConfig& vad_config);

std::vector<std::vector<std::vector<std::vector<float>>>> postprocess_traj_preds(
    const std::vector<float>& all_traj_preds_flat, const VadConfig& vad_config);

std::vector<std::vector<float>> postprocess_traj_cls_scores(
    const std::vector<float>& all_traj_cls_scores_flat, const VadConfig& vad_config);

std::vector<std::vector<float>> postprocess_bbox_preds(
    const std::vector<float>& all_bbox_preds_flat, const VadConfig& vad_config);

std::vector<BBox> postprocess_bboxes(
    const std::vector<float>& all_cls_scores_flat,
    const std::vector<float>& all_traj_preds_flat,
    const std::vector<float>& all_traj_cls_scores_flat,
    const std::vector<float>& all_bbox_preds_flat,
    const VadConfig& vad_config);

// Helper function to parse external input configuration
inline std::pair<std::string, std::string> parse_external_inputs(const std::pair<std::string, std::map<std::string, std::string>>& input_pair) {
  const auto& ext_map = input_pair.second;
  return {ext_map.at("net"), ext_map.at("name")};
}

// VAD model class - Handles inference using CUDA/TensorRT
template<typename LoggerType>
class VadModel
{
public:
  VadModel(
    const VadConfig& vad_config,
    const autoware::tensorrt_common::TrtCommonConfig& backbone_config,
    const autoware::tensorrt_common::TrtCommonConfig& head_config,
    const autoware::tensorrt_common::TrtCommonConfig& head_no_prev_config,
    std::shared_ptr<LoggerType> logger)
    : stream_(nullptr), is_first_frame_(true), 
      vad_config_(vad_config), logger_(std::move(logger)), head_trt_config_(head_config)
  {
    // Logger accepts only classes that inherit from VadLogger
    static_assert(std::is_base_of_v<VadLogger, LoggerType>, 
      "LoggerType must be VadLogger or derive from VadLogger.");    
    
    cudaStreamCreate(&stream_);

    nets_ = init_engines(vad_config_.nets_config, vad_config_, backbone_config, head_config, head_no_prev_config);
  }

  // Destructor
  ~VadModel()
  {
    if (stream_) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
    
    // Cleanup nets
    nets_.clear();
  }

  // Main inference API
  [[nodiscard]] std::optional<VadOutputData> infer(const VadInputData & vad_input_data) {
    // Change head name based on whether it's the first frame
    std::string head_name;
    if (is_first_frame_) {
      head_name = "head_no_prev";
    } else {
      head_name = "head";
    }

    // Load to bindings
    load_inputs(vad_input_data, head_name);

    // Enqueue backbone and head
    enqueue(head_name);

    // Save prev_bev
    saved_prev_bev_ = save_prev_bev(head_name);

    // Convert output to VadOutputData
    VadOutputData output = postprocess(head_name, vad_input_data.command_);

    // If it's the first frame, release "head_no_prev" and load "head"
    if (is_first_frame_) {
      release_network("head_no_prev");
      load_head();
      is_first_frame_ = false;
    }
    
    return output;
  }

  // Member variables
  cudaStream_t stream_;
  std::unordered_map<std::string, std::shared_ptr<Net>> nets_;

  // Storage for previous BEV features
  std::shared_ptr<Tensor> saved_prev_bev_;
  bool is_first_frame_;

  // Configuration information storage
  VadConfig vad_config_;

  std::shared_ptr<VadLogger> logger_;

private:
  autoware::tensorrt_common::TrtCommonConfig head_trt_config_;

  std::unordered_map<std::string, std::shared_ptr<Net>> init_engines(
      const std::vector<NetConfig>& nets_config,
      const VadConfig& vad_config,
      const autoware::tensorrt_common::TrtCommonConfig& backbone_config,
      const autoware::tensorrt_common::TrtCommonConfig& head_config,
      const autoware::tensorrt_common::TrtCommonConfig& head_no_prev_config) {
    
    std::unordered_map<std::string, std::shared_ptr<Net>> nets;
    
    for (const auto& engine : nets_config) {
      std::unordered_map<std::string, std::shared_ptr<Tensor>> external_bindings;
      // reuse memory
      for (const auto& input_pair : engine.inputs) {
        const std::string& k = input_pair.first;
        auto [external_network, external_input_name] = parse_external_inputs(input_pair);
        logger_->info(k + " <- " + external_network + "[" + external_input_name + "]");
        external_bindings[k] = nets[external_network]->bindings[external_input_name];
      }

      if (engine.name == "backbone") {
        nets[engine.name] = std::make_shared<Backbone>(
          vad_config, backbone_config, vad_config_.plugins_path, logger_);
        nets[engine.name]->set_input_tensor(external_bindings);
      } else if (engine.name == "head_no_prev") {
        nets[engine.name] = std::make_shared<Head>(
          vad_config, head_no_prev_config, NetworkType::HEAD_NO_PREV, vad_config_.plugins_path, logger_);
        nets[engine.name]->set_input_tensor(external_bindings);
      } else if (engine.name == "head") {
        nets[engine.name] = std::make_shared<Head>(
          vad_config, head_config, NetworkType::HEAD, vad_config_.plugins_path, logger_);
      }
    }
    
    return nets;
  }

  // Helper functions used in infer function
  void load_inputs(const VadInputData& vad_input_data, const std::string& head_name) {
    nets_["backbone"]->bindings["img"]->load(vad_input_data.camera_images_, stream_);
    nets_[head_name]->bindings["img_metas.0[shift]"]->load(vad_input_data.shift_, stream_);
    nets_[head_name]->bindings["img_metas.0[lidar2img]"]->load(vad_input_data.vad_base2img_, stream_);
    nets_[head_name]->bindings["img_metas.0[can_bus]"]->load(vad_input_data.can_bus_, stream_);

    if (head_name == "head") {
      nets_["head"]->bindings["prev_bev"] = saved_prev_bev_;
    }
  }

  void enqueue(const std::string& head_name) {
    nets_["backbone"]->Enqueue(stream_);
    nets_[head_name]->Enqueue(stream_);
    cudaStreamSynchronize(stream_);
  }

  std::shared_ptr<Tensor> save_prev_bev(const std::string& head_name) {
    auto bev_embed = nets_[head_name]->bindings["out.bev_embed"];
    auto prev_bev = std::make_shared<Tensor>("prev_bev", bev_embed->dim, bev_embed->dtype, logger_);
    cudaMemcpyAsync(prev_bev->ptr, bev_embed->ptr, bev_embed->nbytes(), 
                    cudaMemcpyDeviceToDevice, stream_);
    return prev_bev;
  }

  void release_network(const std::string& network_name) {
    if (nets_.find(network_name) != nets_.end()) {
      // First clear bindings
      nets_[network_name]->bindings.clear();
      cudaStreamSynchronize(stream_);
      
      // Then release Net object
      nets_[network_name].reset();
      nets_.erase(network_name);
      cudaStreamSynchronize(stream_);
    }
  }

  void load_head() {
    auto head_engine = std::find_if(vad_config_.nets_config.begin(), vad_config_.nets_config.end(),
        [](const NetConfig& engine) { return engine.name == "head"; });
    
    if (head_engine == vad_config_.nets_config.end()) {
      logger_->error("Head engine configuration not found");
      return;
    }
    
    std::unordered_map<std::string, std::shared_ptr<Tensor>> external_bindings;
    for (const auto& input_pair : head_engine->inputs) {
      const std::string& k = input_pair.first;
      auto [external_network, external_input_name] = parse_external_inputs(input_pair);
      logger_->info(k + " <- " + external_network + "[" + external_input_name + "]");
      external_bindings[k] = nets_[external_network]->bindings[external_input_name];
    }

    nets_["head"]->set_input_tensor(external_bindings);
  }

  VadOutputData postprocess(const std::string& head_name, int32_t cmd) {
    std::vector<float> ego_fut_preds = nets_[head_name]->bindings["out.ego_fut_preds"]->cpu<float>();
    std::vector<float> map_all_cls_preds_flat = nets_[head_name]->bindings["out.map_all_cls_scores"]->cpu<float>();
    std::vector<float> map_all_pts_preds_flat = nets_[head_name]->bindings["out.map_all_pts_preds"]->cpu<float>();
    std::vector<float> all_traj_preds_flat = nets_[head_name]->bindings["out.all_traj_preds"]->cpu<float>();
    std::vector<float> all_traj_cls_scores_flat = nets_[head_name]->bindings["out.all_traj_cls_scores"]->cpu<float>();
    std::vector<float> all_bbox_preds_flat = nets_[head_name]->bindings["out.all_bbox_preds"]->cpu<float>();
    std::vector<float> all_cls_scores_flat = nets_[head_name]->bindings["out.all_cls_scores"]->cpu<float>();

    // Process detected objects using postprocess_bboxes and apply confidence thresholding
    auto filtered_bboxes = postprocess_bboxes(
        all_cls_scores_flat, all_traj_preds_flat, all_traj_cls_scores_flat, all_bbox_preds_flat, vad_config_);

    std::vector<MapPolyline> map_polylines = postprocess_map_preds(
        map_all_cls_preds_flat, map_all_pts_preds_flat, vad_config_);
    
    // Extract planning for the given command
    std::vector<float> planning(
        ego_fut_preds.begin() + cmd * 12,
        ego_fut_preds.begin() + (cmd + 1) * 12
    );
    
    // cumsum to build trajectory in 3d space
    for (int32_t i = 1; i < 6; i++) {
      planning[i * 2] += planning[(i-1) * 2];
      planning[i * 2 + 1] += planning[(i-1) * 2 + 1];
    }
    
    // Extract all trajectories for all 3 commands
    std::map<int32_t, std::vector<float>> all_trajectories;
    for (int32_t command_idx = 0; command_idx < 3; command_idx++) {
      std::vector<float> trajectory(
          ego_fut_preds.begin() + command_idx * 12,
          ego_fut_preds.begin() + (command_idx + 1) * 12
      );
      
      // cumsum to build trajectory in 3d space
      for (int32_t i = 1; i < 6; i++) {
        trajectory[i * 2] += trajectory[(i-1) * 2];
        trajectory[i * 2 + 1] += trajectory[(i-1) * 2 + 1];
      }
      
      all_trajectories[command_idx] = trajectory;
    }
    
    return VadOutputData{planning, all_trajectories, map_polylines, filtered_bboxes};
  }
};

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE_TENSORRT_VAD_VAD_TRT_HPP_
