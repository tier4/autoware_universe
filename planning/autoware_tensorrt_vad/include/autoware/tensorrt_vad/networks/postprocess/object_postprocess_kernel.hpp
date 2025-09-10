#ifndef AUTOWARE_TENSORRT_VAD_OBJECT_POSTPROCESS_KERNEL_HPP_
#define AUTOWARE_TENSORRT_VAD_OBJECT_POSTPROCESS_KERNEL_HPP_

#include <cuda_runtime.h>
#include <cstdint>
#include <vector>
#include <string>
#include <map>

/**
 * @struct ObjectPostprocessConfig
 * @brief Configuration parameters required for object CUDA postprocessing
 */
struct ObjectPostprocessConfig {
    // VAD configuration parameters
    int32_t prediction_num_queries;      // 300 (number of objects)
    int32_t prediction_num_classes;      // 10 (number of object classes)
    int32_t prediction_bbox_pred_dim;    // 10 (bbox prediction dimension)
    int32_t prediction_trajectory_modes; // 6 (number of trajectory modes)
    int32_t prediction_timesteps;        // 6 (number of prediction timesteps)
    int32_t num_decoder_layers;          // 3 (number of decoder layers)
    
    // Class names and confidence thresholds
    std::vector<std::string> bbox_class_names;
    std::map<std::string, float> object_confidence_thresholds;
    
    // Additional members for CUDA kernel (computed at runtime)
    int32_t bbox_class_count;
    float object_confidence_thresholds_array[16];  // Max 16 classes (adjustable)
    
    // Helper method to prepare kernel data
    void prepare_for_kernel() {
        bbox_class_count = static_cast<int32_t>(bbox_class_names.size());
        
        // Initialize confidence thresholds array
        for (int32_t i = 0; i < 16; ++i) {
            object_confidence_thresholds_array[i] = 0.0f;
        }
        
        // Copy confidence thresholds to flat array
        for (int32_t i = 0; i < static_cast<int32_t>(bbox_class_names.size()) && i < 16; ++i) {
            const std::string& class_name = bbox_class_names.at(i);
            auto it = object_confidence_thresholds.find(class_name);
            if (it != object_confidence_thresholds.end()) {
                object_confidence_thresholds_array[i] = it->second;
            }
        }
    }
};

/**
 * @brief Launch CUDA kernel to postprocess object predictions
 * @param all_cls_scores_flat Flat array of object classification scores
 * @param all_traj_preds_flat Flat array of trajectory predictions
 * @param all_traj_cls_scores_flat Flat array of trajectory classification scores
 * @param all_bbox_preds_flat Flat array of bounding box predictions
 * @param d_output_cls_scores Device buffer for output classification scores [num_queries, num_classes]
 * @param d_output_bbox_preds Device buffer for output bbox predictions [num_queries, bbox_pred_dim]
 * @param d_output_trajectories Device buffer for output trajectories [num_queries, traj_modes, timesteps, 2]
 * @param d_output_traj_scores Device buffer for output trajectory scores [num_queries, traj_modes]
 * @param d_output_valid_flags Device buffer for valid object flags [num_queries]
 * @param config Postprocessing configuration parameters
 * @param stream CUDA stream for asynchronous execution
 * @return cudaError_t CUDA error code
 */
cudaError_t launch_object_postprocess_kernel(
    const float* all_cls_scores_flat,
    const float* all_traj_preds_flat,
    const float* all_traj_cls_scores_flat,
    const float* all_bbox_preds_flat,
    float* d_output_cls_scores,
    float* d_output_bbox_preds,
    float* d_output_trajectories,
    float* d_output_traj_scores,
    int32_t* d_output_valid_flags,
    const ObjectPostprocessConfig& config,
    cudaStream_t stream);

#endif // AUTOWARE_TENSORRT_VAD_OBJECT_POSTPROCESS_KERNEL_HPP_
