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
#include <cuda_runtime.h>
#include <NvInfer.h>
#include <dlfcn.h>
#include "net.h"
#include <map>

#include <autoware/tensorrt_common/tensorrt_common.hpp>

namespace autoware::tensorrt_vad
{

// ロガーインターフェース
class VadLogger {
public:
  virtual ~VadLogger() = default;
  
  // 各ログレベルのメソッドを純粋仮想関数として定義
  virtual void debug(const std::string& message) = 0;
  virtual void info(const std::string& message) = 0;
  virtual void warn(const std::string& message) = 0;
  virtual void error(const std::string& message) = 0;
};

// Loggerクラス（VadModel内で使用）
class Logger : public nvinfer1::ILogger {
private:
    std::shared_ptr<VadLogger> custom_logger_;

public:
    Logger(std::shared_ptr<VadLogger> logger) : custom_logger_(logger) {}
    
    void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override {
        // Only print error messages
        if (severity == nvinfer1::ILogger::Severity::kERROR && custom_logger_) {
            custom_logger_->error(std::string(msg));
        }
    }
};

// VAD推論の入力データ構造
struct VadInputData
{
  // カメラ画像データ（複数カメラ対応）
  std::vector<float> camera_images_;

  // シフト情報（img_metas.0[shift]）
  std::vector<float> shift_;

  // LiDAR座標系からカメラ画像座標系への変換行列（img_metas.0[lidar2img]）
  std::vector<float> lidar2img_;

  // CAN-BUSデータ（車両状態情報：速度、角速度など）(img_metas.0[can_bus])
  std::vector<float> can_bus_;

  // コマンドインデックス（軌道選択用）
  int32_t command_{2};
};

// VAD推論の出力データ構造
struct VadOutputData
{
  // 予測された軌道（6つの2D座標点、累積座標として表現）
  // planning[0,1] = 1st point (x,y), planning[2,3] = 2nd point (x,y), ...
  std::vector<float> predicted_trajectory_{};  // size: 12 (6 points * 2 coordinates)

  // 複数のコマンドに対応した予測軌道のマップ
  // key: command index (int32_t), value: trajectory (std::vector<float>)
  std::map<int32_t, std::vector<float>> predicted_trajectories_{};

  // // 検出されたオブジェクト
  // std::vector<std::vector<float>> detected_objects_{};

  // // コマンドインデックス（選択された軌道のインデックス）
  // int32_t selected_command_index_{2};
};

// 後処理関数

std::vector<std::vector<std::vector<std::vector<float>>>> postprocess_traj_preds(
    const std::vector<float>& all_traj_preds_flat);

std::vector<std::vector<float>> postprocess_traj_cls_scores(
    const std::vector<float>& all_traj_cls_scores_flat);

std::vector<std::vector<float>> postprocess_bbox_preds(
    const std::vector<float>& all_bbox_preds_flat);

// config for Net class
struct NetConfig
{
  std::string name;
  std::map<std::string, std::map<std::string, std::string>> inputs;
};

// config for VadModel class
struct VadModelConfig
{
  std::string plugins_path;
  std::vector<NetConfig> nets_config;
};

// NetworkIO configuration parameters (Task 3.1)
struct VadConfig
{
  int32_t num_cameras;
  int32_t bev_h, bev_w;
  int32_t bev_feature_dim;
  int32_t num_decoder_layers;
  int32_t prediction_num_queries;
  int32_t prediction_num_classes;
  int32_t prediction_bbox_pred_dim;
  int32_t prediction_trajectory_modes;
  int32_t prediction_timesteps;
  int32_t planning_ego_commands;
  int32_t planning_timesteps;
  int32_t can_bus_dim;
  int32_t target_image_width;
  int32_t target_image_height;
  int32_t downsample_factor;
  int32_t map_num_queries;
  int32_t map_num_class;
  int32_t map_points_per_polylines;
};

// VADモデルクラス - CUDA/TensorRTを用いた推論を担当
template<typename LoggerType>
class VadModel
{
public:
  VadModel(
    const VadModelConfig& config,
    const VadConfig& vad_config,
    const autoware::tensorrt_common::TrtCommonConfig& backbone_config,
    const autoware::tensorrt_common::TrtCommonConfig& head_config,
    const autoware::tensorrt_common::TrtCommonConfig& head_no_prev_config,
    std::shared_ptr<LoggerType> logger)
    : stream_(nullptr), initialized_(false), is_first_frame_(true), config_(config), logger_(std::move(logger))
  {
    // loggerはVadLoggerを継承したclassのみ受け取る
    static_assert(std::is_base_of_v<VadLogger, LoggerType>, 
      "LoggerType must be VadLogger or derive from VadLogger.");    
    // 初期化を実行
    runtime_ = create_runtime();
    
    if (!load_plugin(config.plugins_path)) {
      logger_->error("Failed to load plugin");
      return;
    }
    
    cudaStreamCreate(&stream_);

    // TensorRTエンジン初期化
    auto [backbone_trt, head_trt_tmp, head_no_prev_trt] = init_tensorrt(vad_config, backbone_config, head_config, head_no_prev_config);
    if (!backbone_trt || !head_trt_tmp || !head_no_prev_trt) {
      logger_->error("Failed to initialize TensorRT engines");
      initialized_ = false;
      return;
    }
    head_trt_ = std::move(head_trt_tmp);
    nets_ = init_engines(config.nets_config, std::move(backbone_trt), std::move(head_no_prev_trt));
    initialized_ = true;
  }

  // デストラクタ
  ~VadModel()
  {
    if (stream_) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
    
    // netsのクリーンアップ
    nets_.clear();
    
    initialized_ = false;
  }

  // TensorRT初期化API（事前ビルド方式）
  std::tuple<
    std::unique_ptr<autoware::tensorrt_common::TrtCommon>,
    std::unique_ptr<autoware::tensorrt_common::TrtCommon>,
    std::unique_ptr<autoware::tensorrt_common::TrtCommon>
  > init_tensorrt(
      const VadConfig& vad_config,
      const autoware::tensorrt_common::TrtCommonConfig& backbone_config,
      const autoware::tensorrt_common::TrtCommonConfig& head_config,
      const autoware::tensorrt_common::TrtCommonConfig& head_no_prev_config) {
    logger_->info("Initializing TensorRT with pre-build strategy");
    // NetworkIO arrays generation
    auto [backbone_network_io, head_network_io, head_no_prev_network_io] = generate_network_io_configs(vad_config);

    // Build all engines first (this takes time but done only once)
    logger_->info("Building all TensorRT engines (this may take several minutes)...");

    // 1. Build backbone engine (permanent)
    logger_->info("Building backbone engine...");
    auto backbone_trt = build_backbone_engine(backbone_config, backbone_network_io);
    if (!backbone_trt) {
      logger_->error("Failed to build backbone engine");
      return {nullptr, nullptr, nullptr};
    }

    // 2. Build head engine
    logger_->info("Building head engine...");
    auto head_trt = build_head_engine(head_config, head_network_io);
    if (!head_trt) {
      logger_->error("Failed to build head engine");
      return {nullptr, nullptr, nullptr};
    }
    // After building, unload head engine to save GPU memory
    logger_->info("Releasing head engine to save GPU memory");

    // 3. Build head_no_prev engine
    logger_->info("Building head_no_prev engine...");
    auto head_no_prev_trt = build_head_no_prev_engine(head_no_prev_config, head_no_prev_network_io);
    if (!head_no_prev_trt) {
      logger_->error("Failed to build head_no_prev engine");
      return {nullptr, nullptr, nullptr};
    }

    logger_->info("TensorRT pre-build initialization completed successfully");
    logger_->info("Ready for inference with dynamic head engine loading");
    return {std::move(backbone_trt), std::move(head_trt), std::move(head_no_prev_trt)};
  }

  // エンジンビルド専用メソッド
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> build_backbone_engine(
      const autoware::tensorrt_common::TrtCommonConfig& backbone_config,
      const std::vector<tensorrt_common::NetworkIO>& backbone_network_io) {
    logger_->info("Building backbone engine...");
    try {
      auto backbone_trt = std::make_unique<autoware::tensorrt_common::TrtCommon>(
        backbone_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
        std::vector<std::string>{config_.plugins_path});
      auto backbone_network_io_ptr = std::make_unique<std::vector<tensorrt_common::NetworkIO>>(backbone_network_io);
      if (!backbone_trt->setup(nullptr, std::move(backbone_network_io_ptr))) {
        logger_->error("Failed to setup backbone TrtCommon");
        return nullptr;
      }
      logger_->info("Backbone engine built successfully");
      return backbone_trt;
    } catch (const std::exception& e) {
      logger_->error("Exception building backbone engine: " + std::string(e.what()));
      return nullptr;
    }
  }

  std::unique_ptr<autoware::tensorrt_common::TrtCommon> build_head_engine(
      const autoware::tensorrt_common::TrtCommonConfig& head_config,
      const std::vector<tensorrt_common::NetworkIO>& head_network_io) {
    logger_->info("Building head engine...");
    try {
      auto head_trt = std::make_unique<autoware::tensorrt_common::TrtCommon>(
        head_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
        std::vector<std::string>{config_.plugins_path});
      auto head_network_io_ptr = std::make_unique<std::vector<tensorrt_common::NetworkIO>>(head_network_io);
      if (!head_trt->setup(nullptr, std::move(head_network_io_ptr))) {
        logger_->error("Failed to setup head TrtCommon");
        return nullptr;
      }
      logger_->info("Head engine built successfully");
      return head_trt;
    } catch (const std::exception& e) {
      logger_->error("Exception building head engine: " + std::string(e.what()));
      return nullptr;
    }
  }

  std::unique_ptr<autoware::tensorrt_common::TrtCommon> build_head_no_prev_engine(
      const autoware::tensorrt_common::TrtCommonConfig& head_no_prev_config,
      const std::vector<tensorrt_common::NetworkIO>& head_no_prev_network_io) {
    logger_->info("Building head_no_prev engine...");
    try {
      auto head_no_prev_trt = std::make_unique<autoware::tensorrt_common::TrtCommon>(
        head_no_prev_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
        std::vector<std::string>{config_.plugins_path});
      auto head_no_prev_network_io_ptr = std::make_unique<std::vector<tensorrt_common::NetworkIO>>(head_no_prev_network_io);
      if (!head_no_prev_trt->setup(nullptr, std::move(head_no_prev_network_io_ptr))) {
        logger_->error("Failed to setup head_no_prev TrtCommon");
        return nullptr;
      }
      logger_->info("Head_no_prev engine built successfully");
      return head_no_prev_trt;
    } catch (const std::exception& e) {
      logger_->error("Exception building head_no_prev engine: " + std::string(e.what()));
      return nullptr;
    }
  }

  // 動的エンジンロード用メソッド（軽量版 - エンジンファイルから直接ロード）
  bool load_head_engine(std::unique_ptr<autoware::tensorrt_common::TrtCommon>& head_trt,
                        const autoware::tensorrt_common::TrtCommonConfig& head_config,
                        const std::vector<tensorrt_common::NetworkIO>& head_network_io) {
    if (head_trt) {
      logger_->debug("Head engine already loaded");
      return true;
    }
    logger_->info("Loading head engine from built engine file");
    try {
      head_trt = std::make_unique<autoware::tensorrt_common::TrtCommon>(
        head_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
        std::vector<std::string>{config_.plugins_path});
      auto head_network_io_ptr = std::make_unique<std::vector<tensorrt_common::NetworkIO>>(head_network_io);
      if (!head_trt->setup(nullptr, std::move(head_network_io_ptr))) {
        logger_->error("Failed to setup head TrtCommon");
        head_trt.reset();
        return false;
      }
      logger_->info("Head engine loaded successfully");
      return true;
    } catch (const std::exception& e) {
      logger_->error("Exception loading head engine: " + std::string(e.what()));
      head_trt.reset();
      return false;
    }
  }

  bool load_head_no_prev_engine(std::unique_ptr<autoware::tensorrt_common::TrtCommon>& head_no_prev_trt,
                                const autoware::tensorrt_common::TrtCommonConfig& head_no_prev_config,
                                const std::vector<tensorrt_common::NetworkIO>& head_no_prev_network_io) {
    if (head_no_prev_trt) {
      logger_->debug("Head_no_prev engine already loaded");
      return true;
    }
    logger_->info("Loading head_no_prev engine from built engine file");
    try {
      head_no_prev_trt = std::make_unique<autoware::tensorrt_common::TrtCommon>(
        head_no_prev_config, std::make_shared<autoware::tensorrt_common::Profiler>(),
        std::vector<std::string>{config_.plugins_path});
      auto head_no_prev_network_io_ptr = std::make_unique<std::vector<tensorrt_common::NetworkIO>>(head_no_prev_network_io);
      if (!head_no_prev_trt->setup(nullptr, std::move(head_no_prev_network_io_ptr))) {
        logger_->error("Failed to setup head_no_prev TrtCommon");
        head_no_prev_trt.reset();
        return false;
      }
      logger_->info("Head_no_prev engine loaded successfully");
      return true;
    } catch (const std::exception& e) {
      logger_->error("Exception loading head_no_prev engine: " + std::string(e.what()));
      head_no_prev_trt.reset();
      return false;
    }
  }

  void unload_head_engine(std::unique_ptr<autoware::tensorrt_common::TrtCommon>& head_trt) {
    if (head_trt) {
      logger_->info("Unloading head engine to free GPU memory");
      head_trt.reset();
    }
  }

  void unload_head_no_prev_engine(std::unique_ptr<autoware::tensorrt_common::TrtCommon>& head_no_prev_trt) {
    if (head_no_prev_trt) {
      logger_->info("Unloading head_no_prev engine to free GPU memory");
      head_no_prev_trt.reset();
    }
  }

  // メイン推論API
  [[nodiscard]] std::optional<VadOutputData> infer(const VadInputData & vad_input) {
    // 最初のフレームかどうかでheadの名前を変更
    std::string head_name;
    if (is_first_frame_) {
      head_name = "head_no_prev";
    } else {
      head_name = "head";
    }

    // bindingsにload
    load_inputs(vad_input, head_name);

    // backboneとheadをenqueue
    enqueue(head_name);

    // prev_bevを保存
    saved_prev_bev_ = save_prev_bev(head_name);

    // VadOutputDataに出力を変換
    VadOutputData output = postprocess(head_name, vad_input.command_);

    // 最初のフレームなら"head_no_prev"をリリースして"head"をload
    if (is_first_frame_) {
      release_network("head_no_prev");
      load_head();
      is_first_frame_ = false;
    }
    
    return output;
  }

  // メンバ変数
  std::unique_ptr<nvinfer1::IRuntime, std::function<void(nvinfer1::IRuntime*)>> runtime_;
  cudaStream_t stream_;
  std::unordered_map<std::string, std::shared_ptr<nv::Net>> nets_;
  bool initialized_;

  // 前回のBEV特徴量保存用
  std::shared_ptr<nv::Tensor> saved_prev_bev_;
  bool is_first_frame_;

  // 設定情報の保存
  VadModelConfig config_;

  // ロガーインスタンス
  std::shared_ptr<VadLogger> logger_;

private:
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> head_trt_;
  std::unique_ptr<nvinfer1::IRuntime, std::function<void(nvinfer1::IRuntime*)>> create_runtime() {
    static std::unique_ptr<Logger> logger_instance = std::make_unique<Logger>(logger_);
    auto runtime_deleter = []([[maybe_unused]] nvinfer1::IRuntime *runtime) {};
    std::unique_ptr<nvinfer1::IRuntime, decltype(runtime_deleter)> runtime{
        nvinfer1::createInferRuntime(*logger_instance), runtime_deleter};
    return runtime;
  }

  // NetworkIO設定を生成するメソッド
  std::tuple<
    std::vector<tensorrt_common::NetworkIO>,
    std::vector<tensorrt_common::NetworkIO>,
    std::vector<tensorrt_common::NetworkIO>
  > generate_network_io_configs(VadConfig vad_config) {
    logger_->info("Generating NetworkIO configurations");
    std::vector<tensorrt_common::NetworkIO> backbone_network_io;
    std::vector<tensorrt_common::NetworkIO> head_network_io;
    std::vector<tensorrt_common::NetworkIO> head_no_prev_network_io;

    // Backbone Network IO Configuration
    int32_t downsampled_image_height = vad_config.target_image_height / vad_config.downsample_factor;
    int32_t downsampled_image_width = vad_config.target_image_width / vad_config.downsample_factor;
    nvinfer1::Dims camera_input_dims{4, {vad_config.num_cameras, 3, vad_config.target_image_height, vad_config.target_image_width}};
    nvinfer1::Dims backbone_output_dims{5, {vad_config.num_cameras, 1, vad_config.bev_feature_dim, downsampled_image_height, downsampled_image_width}};

    backbone_network_io.emplace_back("img", camera_input_dims);
    backbone_network_io.emplace_back("out.0", backbone_output_dims);

    // Common dimensions for head networks
    nvinfer1::Dims mlvl_dims{5, {1, vad_config.num_cameras, vad_config.bev_feature_dim, downsampled_image_height, downsampled_image_width}};
    nvinfer1::Dims can_bus_dims{2, {1, vad_config.can_bus_dim}};
    nvinfer1::Dims lidar2img_dims{3, {vad_config.num_cameras, 4, 4}};
    nvinfer1::Dims shift_dims{2, {1, 2}};
    nvinfer1::Dims prev_bev_dims{3, {vad_config.bev_h * vad_config.bev_w, 1, vad_config.bev_feature_dim}};
    nvinfer1::Dims ego_fut_preds_dims{4, {1, vad_config.planning_ego_commands, vad_config.planning_timesteps, 2}};
    nvinfer1::Dims traj_preds_dims{5, {3, 1, vad_config.prediction_num_queries, vad_config.prediction_trajectory_modes, vad_config.prediction_timesteps*2}};
    nvinfer1::Dims traj_cls_dims{4, {3, 1, vad_config.prediction_num_queries, vad_config.prediction_trajectory_modes}};
    nvinfer1::Dims bbox_preds_dims{4, {3, 1, vad_config.prediction_num_queries, vad_config.prediction_bbox_pred_dim}};
    nvinfer1::Dims all_cls_scores_dims{4, {3, 1, vad_config.prediction_num_queries, vad_config.prediction_num_classes}};
    nvinfer1::Dims map_all_cls_scores_dims{4, {3, 1, vad_config.map_num_queries, vad_config.map_num_class}};
    nvinfer1::Dims map_all_pts_preds_dims{5, {3, 1, vad_config.map_num_queries, vad_config.map_points_per_polylines, 2}};
    nvinfer1::Dims map_all_bbox_preds_dims{4, {3, 1, vad_config.map_num_queries, 4}};

    // Head Network IO Configuration (with previous BEV)
    head_network_io.emplace_back("mlvl_feats.0", mlvl_dims);
    head_network_io.emplace_back("img_metas.0[can_bus]", can_bus_dims);
    head_network_io.emplace_back("img_metas.0[lidar2img]", lidar2img_dims);
    head_network_io.emplace_back("img_metas.0[shift]", shift_dims);
    head_network_io.emplace_back("prev_bev", prev_bev_dims);
    head_network_io.emplace_back("out.bev_embed", prev_bev_dims);
    head_network_io.emplace_back("out.ego_fut_preds", ego_fut_preds_dims);
    head_network_io.emplace_back("out.all_traj_preds", traj_preds_dims);
    head_network_io.emplace_back("out.all_traj_cls_scores", traj_cls_dims);
    head_network_io.emplace_back("out.all_bbox_preds", bbox_preds_dims);
    head_network_io.emplace_back("out.all_cls_scores", all_cls_scores_dims);
    head_network_io.emplace_back("out.map_all_cls_scores", map_all_cls_scores_dims);
    head_network_io.emplace_back("out.map_all_pts_preds", map_all_pts_preds_dims);
    head_network_io.emplace_back("out.map_all_bbox_preds", map_all_bbox_preds_dims);

    // Head No Previous Network IO Configuration (without prev_bev input)
    head_no_prev_network_io.emplace_back("mlvl_feats.0", mlvl_dims);
    head_no_prev_network_io.emplace_back("img_metas.0[can_bus]", can_bus_dims);
    head_no_prev_network_io.emplace_back("img_metas.0[lidar2img]", lidar2img_dims);
    head_no_prev_network_io.emplace_back("img_metas.0[shift]", shift_dims);
    head_no_prev_network_io.emplace_back("out.bev_embed", prev_bev_dims);
    head_no_prev_network_io.emplace_back("out.ego_fut_preds", ego_fut_preds_dims);
    head_no_prev_network_io.emplace_back("out.all_traj_preds", traj_preds_dims);
    head_no_prev_network_io.emplace_back("out.all_traj_cls_scores", traj_cls_dims);
    head_no_prev_network_io.emplace_back("out.all_bbox_preds", bbox_preds_dims);
    head_no_prev_network_io.emplace_back("out.all_cls_scores", all_cls_scores_dims);
    head_no_prev_network_io.emplace_back("out.map_all_cls_scores", map_all_cls_scores_dims);
    head_no_prev_network_io.emplace_back("out.map_all_pts_preds", map_all_pts_preds_dims);
    head_no_prev_network_io.emplace_back("out.map_all_bbox_preds", map_all_bbox_preds_dims);

    logger_->info("NetworkIO configurations generated successfully");
    return {backbone_network_io, head_network_io, head_no_prev_network_io};
  }

  bool load_plugin(const std::string& plugin_dir) {
    void* h_ = dlopen(plugin_dir.c_str(), RTLD_NOW);
    logger_->info("loading plugin from: " + plugin_dir);
    if (!h_) {
      const char* error = dlerror();
      logger_->error("Failed to load library: " + std::string(error ? error : "unknown error"));
      return false;
    }
    return true;
  }

  std::unordered_map<std::string, std::shared_ptr<nv::Net>> init_engines(
      const std::vector<NetConfig>& nets_config,
      std::unique_ptr<autoware::tensorrt_common::TrtCommon> backbone_trt,
      std::unique_ptr<autoware::tensorrt_common::TrtCommon> head_no_prev_trt) {
    
    std::unordered_map<std::string, std::shared_ptr<nv::Net>> nets;
    
    for (const auto& engine : nets_config) {
      if (engine.name == "head") {
        continue;  // headは後で初期化
      }
      std::unordered_map<std::string, std::shared_ptr<nv::Tensor>> external_bindings;
      // reuse memory
      for (const auto& input_pair : engine.inputs) {
        const std::string& k = input_pair.first;
        const auto& ext_map = input_pair.second;      
        std::string ext_net = ext_map.at("net");
        std::string ext_name = ext_map.at("name");
        logger_->info(k + " <- " + ext_net + "[" + ext_name + "]");
        external_bindings[k] = nets[ext_net]->bindings[ext_name];
      }

      if (engine.name == "backbone") {
        nets[engine.name] = std::make_shared<nv::Net>(
          runtime_.get(), external_bindings, std::move(backbone_trt));
      } else if (engine.name == "head_no_prev") {
        nets[engine.name] = std::make_shared<nv::Net>(
          runtime_.get(), external_bindings, std::move(head_no_prev_trt));
      }
    }
    
    return nets;
  }

  // infer関数で使用するヘルパー関数
  void load_inputs(const VadInputData& vad_input, const std::string& head_name) {
    nets_["backbone"]->bindings["img"]->load(vad_input.camera_images_, stream_);
    nets_[head_name]->bindings["img_metas.0[shift]"]->load(vad_input.shift_, stream_);
    nets_[head_name]->bindings["img_metas.0[lidar2img]"]->load(vad_input.lidar2img_, stream_);
    nets_[head_name]->bindings["img_metas.0[can_bus]"]->load(vad_input.can_bus_, stream_);

    if (head_name == "head") {
      nets_["head"]->bindings["prev_bev"] = saved_prev_bev_;
    }
  }

  void enqueue(const std::string& head_name) {
    nets_["backbone"]->Enqueue(stream_);
    nets_[head_name]->Enqueue(stream_);
    cudaStreamSynchronize(stream_);
  }

  std::shared_ptr<nv::Tensor> save_prev_bev(const std::string& head_name) {
    auto bev_embed = nets_[head_name]->bindings["out.bev_embed"];
    auto prev_bev = std::make_shared<nv::Tensor>("prev_bev", bev_embed->dim, bev_embed->dtype);
    cudaMemcpyAsync(prev_bev->ptr, bev_embed->ptr, bev_embed->nbytes(), 
                    cudaMemcpyDeviceToDevice, stream_);
    return prev_bev;
  }

  void release_network(const std::string& network_name) {
    if (nets_.find(network_name) != nets_.end()) {
      // まずbindingsをクリア
      nets_[network_name]->bindings.clear();
      cudaStreamSynchronize(stream_);
      
      // 次にNetオブジェクトを解放
      nets_[network_name].reset();
      nets_.erase(network_name);
      cudaStreamSynchronize(stream_);
    }
  }

  void load_head() {
    auto head_engine = std::find_if(config_.nets_config.begin(), config_.nets_config.end(),
        [](const NetConfig& engine) { return engine.name == "head"; });
    
    if (head_engine == config_.nets_config.end()) {
      logger_->error("Head engine configuration not found");
      return;
    }
    
    std::unordered_map<std::string, std::shared_ptr<nv::Tensor>> external_bindings;
    for (const auto& input_pair : head_engine->inputs) {
      const std::string& k = input_pair.first;
      const auto& ext_map = input_pair.second;      
      std::string ext_net = ext_map.at("net");
      std::string ext_name = ext_map.at("name");
      logger_->info(k + " <- " + ext_net + "[" + ext_name + "]");
      external_bindings[k] = nets_[ext_net]->bindings[ext_name];
    }

    // head_trtを使ってNetを生成
    nets_["head"] = std::make_shared<nv::Net>(runtime_.get(), external_bindings, std::move(head_trt_));
  }

  VadOutputData postprocess(const std::string& head_name, int32_t cmd) {
    std::vector<float> ego_fut_preds = nets_[head_name]->bindings["out.ego_fut_preds"]->cpu<float>();
    std::vector<float> map_all_pts_preds_flat = nets_[head_name]->bindings["out.map_all_pts_preds"]->cpu<float>();
    std::vector<float> all_traj_preds_flat = nets_[head_name]->bindings["out.all_traj_preds"]->cpu<float>();
    std::vector<float> all_traj_cls_scores_flat = nets_[head_name]->bindings["out.all_traj_cls_scores"]->cpu<float>();
    std::vector<float> all_bbox_preds_flat = nets_[head_name]->bindings["out.all_bbox_preds"]->cpu<float>();
    
    // flat -> structured
    auto traj_preds = postprocess_traj_preds(all_traj_preds_flat);
    auto traj_cls_scores = postprocess_traj_cls_scores(all_traj_cls_scores_flat);
    auto bbox_preds = postprocess_bbox_preds(all_bbox_preds_flat);
    
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
    
    return VadOutputData{planning, all_trajectories};
  }
};

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE_TENSORRT_VAD_VAD_TRT_HPP_
