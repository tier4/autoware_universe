#ifndef AUTOWARE_TENSORRT_VAD_OUTPUT_CONVERTER_MAP_CONVERTER_HPP
#define AUTOWARE_TENSORRT_VAD_OUTPUT_CONVERTER_MAP_CONVERTER_HPP

#include "autoware/tensorrt_vad/converter.hpp"
#include "autoware/tensorrt_vad/vad_model.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/time.hpp>
#include <Eigen/Dense>

#include <vector>
#include <string>
#include <map>
#include <array>

namespace autoware::tensorrt_vad::vad_interface {

/**
 * @brief Converter for processing VAD map polylines into ROS visualization markers
 */
class OutputMapConverter : public Converter {
public:
  /**
   * @brief Constructor
   * @param coordinate_transformer Reference to coordinate transformer
   * @param config Reference to VAD interface configuration
   */
  OutputMapConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config);

  /**
   * @brief Process map polylines from VAD to visualization markers
   * @param vad_map_polylines Vector of map polylines from VAD output
   * @param stamp Timestamp for the message
   * @param base2map_transform Transformation matrix from base_link to map frame
   * @return visualization_msgs::msg::MarkerArray Visualization markers for RViz
   */
  visualization_msgs::msg::MarkerArray process_map_points(
    const std::vector<MapPolyline>& vad_map_polylines,
    const rclcpp::Time& stamp,
    const Eigen::Matrix4d& base2map_transform) const;

private:
  /**
   * @brief Create a single marker from a map polyline
   * @param map_polyline Single polyline with type and points
   * @param marker_id Unique marker ID
   * @param stamp Timestamp for the marker
   * @param base2map_transform Transformation matrix from base_link to map frame
   * @return visualization_msgs::msg::Marker Single marker for the polyline
   */
  visualization_msgs::msg::Marker create_polyline_marker(
    const MapPolyline& map_polyline,
    const int32_t marker_id,
    const rclcpp::Time& stamp,
    const Eigen::Matrix4d& base2map_transform) const;

  /**
   * @brief Get color for a specific map type
   * @param type Map polyline type (e.g., "divider", "boundary", etc.)
   * @return std::array<float, 3> RGB color values [0-1]
   */
  std::array<float, 3> get_color_for_type(const std::string& type) const;
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif // AUTOWARE_TENSORRT_VAD_OUTPUT_CONVERTER_MAP_CONVERTER_HPP
