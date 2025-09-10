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

#include "autoware/tensorrt_vad/vad_model.hpp"
#include <cmath>
#include <stdexcept>
#include <array>
#include <unordered_map>

namespace autoware::tensorrt_vad {

// Sigmoid activation function
inline float sigmoid(float x) {
  return 1.0f / (1.0f + std::exp(-x));
}

std::vector<std::vector<std::vector<std::vector<float>>>> postprocess_traj_preds(
    const std::vector<float>& all_traj_preds_flat, const VadConfig& vad_config) {
  const int32_t num_objects = vad_config.prediction_num_queries;
  const int32_t num_fut_modes = vad_config.prediction_trajectory_modes;
  const int32_t num_fut_ts = vad_config.prediction_timesteps;
  const int32_t traj_coords = 2;
  
  // Use only final layer (index 2) data
  const int32_t final_layer_idx = vad_config.num_decoder_layers - 1; // = 2
  const int32_t layer_size = num_objects * num_fut_modes * num_fut_ts * traj_coords;
  const int32_t final_layer_offset = final_layer_idx * layer_size;
  
  std::vector<std::vector<std::vector<std::vector<float>>>> traj_preds;
  traj_preds.resize(num_objects);
  for (int32_t obj = 0; obj < num_objects; ++obj) {
    traj_preds[obj].resize(num_fut_modes);
    for (int32_t fut_mode = 0; fut_mode < num_fut_modes; ++fut_mode) {
      traj_preds[obj][fut_mode].resize(num_fut_ts);
      for (int32_t ts = 0; ts < num_fut_ts; ++ts) {
        traj_preds[obj][fut_mode][ts].resize(traj_coords);
        int32_t idx_within_layer = obj * num_fut_modes * num_fut_ts * traj_coords + 
                                   fut_mode * num_fut_ts * traj_coords + 
                                   ts * traj_coords;
        int32_t idx_flat = final_layer_offset + idx_within_layer;
        traj_preds[obj][fut_mode][ts][0] = all_traj_preds_flat[idx_flat];
        traj_preds[obj][fut_mode][ts][1] = all_traj_preds_flat[idx_flat + 1];
      }
      
      // cumsum to build trajectory in 3d space - calculate cumulative sum like planning
      for (int32_t ts = 1; ts < num_fut_ts; ++ts) {
        traj_preds[obj][fut_mode][ts][0] += traj_preds[obj][fut_mode][ts-1][0];  // Cumulative sum of x coordinates
        traj_preds[obj][fut_mode][ts][1] += traj_preds[obj][fut_mode][ts-1][1];  // Cumulative sum of y coordinates
      }
    }
  }
  return traj_preds;
}

std::vector<std::vector<float>> postprocess_traj_cls_scores(
    const std::vector<float>& all_traj_cls_scores_flat, const VadConfig& vad_config) {
  const int32_t num_layers = vad_config.num_decoder_layers;
  const int32_t num_objects = vad_config.prediction_num_queries;
  const int32_t num_fut_modes = vad_config.prediction_trajectory_modes;
  
  // Use only final layer (index 2) data
  const int32_t final_layer_idx = num_layers - 1; // = 2
  const int32_t layer_size = num_objects * num_fut_modes;
  const int32_t final_layer_offset = final_layer_idx * layer_size;
  
  std::vector<std::vector<float>> traj_cls_scores;
  traj_cls_scores.resize(num_objects);
  for (int32_t obj = 0; obj < num_objects; ++obj) {
    traj_cls_scores[obj].resize(num_fut_modes);
    for (int32_t fut_mode = 0; fut_mode < num_fut_modes; ++fut_mode) {
      int32_t idx_within_layer = obj * num_fut_modes + fut_mode;
      int32_t idx_flat = final_layer_offset + idx_within_layer;
      traj_cls_scores[obj][fut_mode] = sigmoid(all_traj_cls_scores_flat[idx_flat]);
    }
  }
  return traj_cls_scores;
}

std::vector<std::vector<float>> postprocess_bbox_preds(
    const std::vector<float>& all_bbox_preds_flat, const VadConfig& vad_config) {
  const int32_t num_objects = vad_config.prediction_num_queries;
  const int32_t bbox_features = vad_config.prediction_bbox_pred_dim;
  
  // Use only final layer (index 2) data
  const int32_t final_layer_idx = vad_config.num_decoder_layers - 1; // = 2
  const int32_t layer_size = num_objects * bbox_features;
  const int32_t final_layer_offset = final_layer_idx * layer_size;
  
  std::vector<std::vector<float>> bbox_preds;
  bbox_preds.resize(num_objects);
  for (int32_t obj = 0; obj < num_objects; ++obj) {
    bbox_preds[obj].resize(bbox_features);
    for (int32_t feat = 0; feat < bbox_features; ++feat) {
      int32_t idx_within_layer = obj * bbox_features + feat;
      int32_t idx_flat = final_layer_offset + idx_within_layer;
      bbox_preds[obj][feat] = all_bbox_preds_flat[idx_flat];
    }
  }
  return bbox_preds;
}

/**
 * @brief Parse flat array of object class predictions and convert to 2D score array
 * During inference, use only the final layer (-1st layer)
 */
std::vector<std::vector<float>> 
postprocess_class_scores(const std::vector<float>& all_cls_scores_flat, const VadConfig& vad_config) 
{
    const int32_t num_objects = vad_config.prediction_num_queries;
    const int32_t num_classes = vad_config.prediction_num_classes;
    
    // Use only final layer (index 2) data
    const int32_t final_layer_idx = vad_config.num_decoder_layers - 1; // = 2
    const int32_t layer_size = num_objects * num_classes;
    const int32_t final_layer_offset = final_layer_idx * layer_size;
    
    std::vector<std::vector<float>> cls_scores(num_objects, std::vector<float>(num_classes));

    for (int32_t obj = 0; obj < num_objects; ++obj) {
        for (int32_t c = 0; c < num_classes; ++c) {
            int32_t idx_within_layer = obj * num_classes + c;
            int32_t idx_flat = final_layer_offset + idx_within_layer;
            cls_scores[obj][c] = sigmoid(all_cls_scores_flat[idx_flat]);
        }
    }
    return cls_scores;
}

/**
 * @brief Perform overall object post-processing and generate BBox data
 * @param all_cls_scores_flat Flat array of object classification scores
 * @param all_traj_preds_flat Flat array of trajectory predictions
 * @param all_traj_cls_scores_flat Flat array of trajectory classification scores
 * @param all_bbox_preds_flat Flat array of bounding box predictions
 * @param vad_config VAD configuration containing thresholds and model parameters
 * @return Vector of structured BBox data
 */
std::vector<BBox> postprocess_bboxes(
    const std::vector<float>& all_cls_scores_flat,
    const std::vector<float>& all_traj_preds_flat,
    const std::vector<float>& all_traj_cls_scores_flat,
    const std::vector<float>& all_bbox_preds_flat,
    const VadConfig& vad_config)
{
    // Structure each prediction result
    auto obj_cls_scores = postprocess_class_scores(all_cls_scores_flat, vad_config);
    auto traj_preds = postprocess_traj_preds(all_traj_preds_flat, vad_config);
    auto traj_cls_scores = postprocess_traj_cls_scores(all_traj_cls_scores_flat, vad_config);
    auto bbox_preds = postprocess_bbox_preds(all_bbox_preds_flat, vad_config);

    const int32_t num_objects = vad_config.prediction_num_queries;
    std::vector<BBox> bboxes;
    bboxes.reserve(num_objects);

    for (int32_t obj = 0; obj < num_objects; ++obj) {
        BBox bbox;
        
        // Bounding box parameters [c_x, c_y, w, l, c_z, h, sin(theta), cos(theta), v_x, v_y]
        for (int32_t i = 0; i < vad_config.prediction_bbox_pred_dim; ++i) {
            if (i == 2 || i == 3 || i == 5) {
                // Apply exp transformation to w, l, h
                bbox.bbox[i] = std::exp(bbox_preds[obj][i]);
            } else {
                bbox.bbox[i] = bbox_preds[obj][i];
            }
        }
        
        // Determine object class and confidence
        auto max_it = std::max_element(obj_cls_scores[obj].begin(), obj_cls_scores[obj].end());
        bbox.confidence = *max_it;
        bbox.object_class = std::distance(obj_cls_scores[obj].begin(), max_it);
        
        // Apply confidence filtering using bbox_class_names from VadConfig
        if (bbox.object_class >= 0 && bbox.object_class < static_cast<int32_t>(vad_config.bbox_class_names.size())) {
            std::string class_name = vad_config.bbox_class_names[bbox.object_class];
            auto threshold_it = vad_config.object_confidence_thresholds.find(class_name);
            if (threshold_it != vad_config.object_confidence_thresholds.end()) {
                float threshold = threshold_it->second;
                // Skip if threshold is not met
                if (bbox.confidence < threshold) {
                    continue;
                }
            } else {
                // Skip classes without threshold setting
                continue;
            }
        } else {
            // Skip unknown classes
            continue;
        }
        
        // Structure 6 predicted trajectories
        for (int32_t mode = 0; mode < vad_config.prediction_trajectory_modes; ++mode) {
            PredictedTrajectory pred_traj;
            pred_traj.confidence = traj_cls_scores[obj][mode];
            
            // Set 6 timestep trajectory data
            for (int32_t ts = 0; ts < vad_config.prediction_timesteps; ++ts) {
                pred_traj.trajectory[ts][0] = traj_preds[obj][mode][ts][0]; // x coordinate
                pred_traj.trajectory[ts][1] = traj_preds[obj][mode][ts][1]; // y coordinate
            }
            
            bbox.trajectories[mode] = pred_traj;
        }
        
        bboxes.push_back(bbox);
    }

    return bboxes;
}

} // namespace autoware::tensorrt_vad
