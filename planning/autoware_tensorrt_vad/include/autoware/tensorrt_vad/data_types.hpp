#ifndef AUTOWARE_TENSORRT_VAD_DATA_TYPES_HPP_
#define AUTOWARE_TENSORRT_VAD_DATA_TYPES_HPP_

#include <vector>
#include <string>
#include <array>

namespace autoware::tensorrt_vad
{

/**
 * @brief Structure representing predicted trajectory
 */
struct PredictedTrajectory {
    std::vector<std::array<float, 2>> trajectory;    // Dynamic timesteps × 2 coordinates (x, y)
    float confidence;                                // Confidence of this trajectory
    
    // Default constructor
    PredictedTrajectory() : confidence(0.0f) {
        // Default to empty trajectory
    }
    
    // Constructor with specific number of timesteps
    explicit PredictedTrajectory(size_t timesteps) : confidence(0.0f) {
        trajectory.resize(timesteps);
        // Initialize trajectory coordinates to 0
        for (size_t i = 0; i < timesteps; ++i) {
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
    std::vector<PredictedTrajectory> trajectories;   // Dynamic number of predicted trajectories
    
    // Default constructor
    BBox() : confidence(0.0f), object_class(-1) {
        // Initialize bbox coordinates to 0
        for (int32_t i = 0; i < 10; ++i) {
            bbox[i] = 0.0f;
        }
    }
    
    // Constructor with specific number of trajectory modes and timesteps
    BBox(size_t trajectory_modes, size_t timesteps) : confidence(0.0f), object_class(-1) {
        // Initialize bbox coordinates to 0
        for (int32_t i = 0; i < 10; ++i) {
            bbox[i] = 0.0f;
        }
        
        // Initialize trajectories with specified modes and timesteps
        trajectories.reserve(trajectory_modes);
        for (size_t i = 0; i < trajectory_modes; ++i) {
            trajectories.emplace_back(timesteps);
        }
    }
};

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE_TENSORRT_VAD_DATA_TYPES_HPP_
