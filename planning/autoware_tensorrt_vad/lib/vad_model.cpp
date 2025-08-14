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

// Denormalize 2D points using PC range
inline std::vector<float> denormalize_2d_pts(const std::vector<float>& pts, 
                                            const std::vector<float>& detection_range) {
  if (pts.size() != 2) {
    throw std::invalid_argument("pts must have 2 elements: x, y");
  }
  if (detection_range.size() != 6) {
    throw std::invalid_argument("detection_range must have 6 elements");
  }
  
  std::vector<float> denormalized_pts = pts;
  
  // x = normalized_x * (x_max[3] - x_min[0]) + x_min[0]
  denormalized_pts[0] = pts[0] * (detection_range[3] - detection_range[0]) + detection_range[0];

  // y = normalized_y * (y_max[4] - y_min[1]) + y_min[1]
  denormalized_pts[1] = pts[1] * (detection_range[4] - detection_range[1]) + detection_range[1];
  
  return denormalized_pts;
}


/**
 * @brief Process confidence scores for map polylines from flat to 2D array
 * Uses only the final decoder layer output
 */
std::vector<std::vector<float>> 
process_map_class_scores(const std::vector<float>& cls_preds_flat,
                    const VadConfig& vad_config) 
{
    const int32_t num_query = vad_config.map_num_queries;
    const int32_t cls_out_channels = vad_config.map_num_classes;
    const int32_t num_layers = vad_config.num_decoder_layers;
    
    // Use output from last layer (index = num_layers - 1) only
    const int32_t final_layer_idx = num_layers - 1;
    const int32_t layer_size = num_query * cls_out_channels;
    const int32_t final_layer_offset = final_layer_idx * layer_size;
    
    std::vector<std::vector<float>> cls_scores(num_query, std::vector<float>(cls_out_channels));

    for (int32_t q = 0; q < num_query; ++q) {
        for (int32_t c = 0; c < cls_out_channels; ++c) {
            int32_t flat_idx = final_layer_offset + q * cls_out_channels + c;
            cls_scores[q][c] = sigmoid(cls_preds_flat[flat_idx]);
        }
    }
    return cls_scores;
}

/**
 * @brief Process map polylines from flat array to denormalized 3D array
 * Uses only the final decoder layer output
 */
std::vector<std::vector<std::vector<float>>> 
process_map_points(const std::vector<float>& pts_preds_flat,
               const VadConfig& vad_config) 
{
    const int32_t num_query = vad_config.map_num_queries;
    const int32_t fixed_num_pts = vad_config.map_points_per_polylines;
    const int32_t pts_coords = 2;
    const int32_t num_layers = vad_config.num_decoder_layers;
    
    // Use output from last layer (index = num_layers - 1) only
    const int32_t final_layer_idx = num_layers - 1;
    const int32_t layer_size = num_query * fixed_num_pts * pts_coords;
    const int32_t final_layer_offset = final_layer_idx * layer_size;
    
    std::vector<std::vector<std::vector<float>>> pts_preds(num_query, std::vector<std::vector<float>>(fixed_num_pts, std::vector<float>(pts_coords)));

    for (int32_t q = 0; q < num_query; ++q) {
        for (int32_t p = 0; p < fixed_num_pts; ++p) {
            std::vector<float> pt(pts_coords);
            for (int32_t d = 0; d < pts_coords; ++d) {
                int32_t flat_idx = final_layer_offset + q * fixed_num_pts * pts_coords + p * pts_coords + d;
                pt[d] = pts_preds_flat[flat_idx];
            }
            std::vector<float> denormalized_pt = denormalize_2d_pts(pt, vad_config.detection_range);
            for (int32_t d = 0; d < pts_coords; ++d) {
                pts_preds[q][p][d] = denormalized_pt[d];
            }
        }
    }
    return pts_preds;
}

/**
 * @brief Select confident predictions and filter by class-specific thresholds
 * @param cls_scores confidence scores [num_query, num_classes]
 * @param pts_preds predicted map polylines [num_query, num_points, 2]
 * @param vad_config VAD configuration containing class names and thresholds
 * @return vector of MapPolyline objects filtered by confidence scores
 */
std::vector<MapPolyline>
select_confident_map_polylines(
    const std::vector<std::vector<float>>& cls_scores,
    const std::vector<std::vector<std::vector<float>>>& pts_preds,
    const VadConfig& vad_config)
{
  std::vector<MapPolyline> map_polylines;
    map_polylines.reserve(cls_scores.size());

    for (size_t polyline_index = 0; polyline_index < cls_scores.size(); ++polyline_index) {
        auto max_it = std::max_element(cls_scores[polyline_index].begin(), cls_scores[polyline_index].begin() + vad_config.map_num_classes);
        float max_score = *max_it;
        int32_t max_class_idx = std::distance(cls_scores[polyline_index].begin(), max_it);

        // If max confidence score is greater than or equal to the threshold for the class, add the polyline
        std::string class_name = vad_config.map_class_names.at(max_class_idx);
        auto threshold = vad_config.map_confidence_thresholds.at(class_name);
        if (max_score >= threshold) {
            map_polylines.emplace_back(class_name, pts_preds[polyline_index]);
        }
    }

    return map_polylines;
}

/**
 * @brief Postprocess model inference results for map predictions
 */
std::vector<MapPolyline>
postprocess_map_preds(
    const std::vector<float>& map_all_cls_preds_flat,
    const std::vector<float>& map_all_pts_preds_flat,
    const VadConfig& vad_config) 
{
    // 1. get confidence scores for each polyline
    auto cls_scores = process_map_class_scores(map_all_cls_preds_flat, vad_config);

    // 2. get points in each polyline
    auto pts_preds = process_map_points(map_all_pts_preds_flat, vad_config);
    
    // 3. filter polylines based on confidence scores and create MapPolyline objects
    auto map_polylines = select_confident_map_polylines(cls_scores, pts_preds, vad_config);
    
    return map_polylines;
}

} // namespace autoware::tensorrt_vad
