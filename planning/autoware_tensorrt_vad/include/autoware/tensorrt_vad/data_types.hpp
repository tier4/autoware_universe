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

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE_TENSORRT_VAD_DATA_TYPES_HPP_
