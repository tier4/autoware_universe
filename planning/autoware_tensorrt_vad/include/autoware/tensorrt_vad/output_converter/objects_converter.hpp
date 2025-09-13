#ifndef AUTOWARE_TENSORRT_VAD_OUTPUT_CONVERTER_OBJECTS_CONVERTER_HPP
#define AUTOWARE_TENSORRT_VAD_OUTPUT_CONVERTER_OBJECTS_CONVERTER_HPP

#include "autoware/tensorrt_vad/converter.hpp"
#include "autoware/tensorrt_vad/vad_model.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/time.hpp>
#include <Eigen/Dense>

#include <vector>
#include <string>

namespace autoware::tensorrt_vad::vad_interface {

/**
 * @brief Converter for processing VAD bounding boxes into ROS predicted objects
 */
class OutputObjectsConverter : public Converter {
public:
  /**
   * @brief Constructor
   * @param coordinate_transformer Reference to coordinate transformer
   * @param config Reference to VAD interface configuration
   */
  OutputObjectsConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config);

  /**
   * @brief Process predicted objects from VAD to ROS format
   * @param bboxes Vector of bounding boxes from VAD output
   * @param stamp Timestamp for the message
   * @param base2map_transform Transformation matrix from base_link to map frame
   * @return autoware_perception_msgs::msg::PredictedObjects ROS message with predicted objects
   */
  autoware_perception_msgs::msg::PredictedObjects process_predicted_objects(
    const std::vector<BBox>& bboxes,
    const rclcpp::Time& stamp,
    const Eigen::Matrix4d& base2map_transform) const;

private:
  // TODO(Shin-kyoto): tentative implementation. 
  // offset for z is needed because VAD trained with nuScenes dataset outputs bbox in lidar coordinate, 
  // but autoware needs bbox in base_link coordinate.
  float z_offset_;

  /**
   * @brief Convert VAD object class to Autoware classification
   * @param object_class VAD object class index
   * @param confidence Classification confidence
   * @return autoware_perception_msgs::msg::ObjectClassification Autoware classification
   */
  autoware_perception_msgs::msg::ObjectClassification convert_classification(
    const int32_t object_class,
    const float confidence) const;

  /**
   * @brief Convert VAD bounding box to ROS Twist message
   * @param bbox Bounding box data
   * @return geometry_msgs::msg::Twist ROS Twist message
   */
  geometry_msgs::msg::Twist convert_velocity(
    const BBox& bbox) const;

  /**
   * @brief Convert VAD bounding box to ROS Shape message
   * @param bbox Bounding box data
   * @return autoware_perception_msgs::msg::Shape ROS Shape message
   */
  static autoware_perception_msgs::msg::Shape convert_shape(
    const BBox& bbox);

  /**
   * @brief Process predicted trajectory for an object
   * @param bbox Bounding box containing trajectory data
   * @param base2map_transform Transformation matrix from base_link to map frame
   * @param yaw Object orientation for fallback direction
   * @return std::vector<autoware_perception_msgs::msg::PredictedPath> Predicted paths
   */
  std::vector<autoware_perception_msgs::msg::PredictedPath> convert_predicted_paths(
    const BBox& bbox,
    const Eigen::Matrix4d& base2map_transform,
    const float yaw) const;

  /**
   * @brief Convert VAD bounding box to ROS Point message
   * @param bbox Bounding box data
   * @param base2map_transform Transformation matrix from base_link to map frame
   * @return geometry_msgs::msg::Point ROS Point message
   */
  geometry_msgs::msg::Point convert_position(
    const BBox& bbox,
    const Eigen::Matrix4d& base2map_transform) const;

  /**
   * @brief Calculate object orientation from trajectory or bbox
   * @param bbox Bounding box data
   * @param base2map_transform Transformation matrix from base_link to map frame
   * @return float Final yaw angle in map frame
   */
  float calculate_object_orientation(
    const BBox& bbox,
    const Eigen::Matrix4d& base2map_transform) const;

  /**
   * @brief Calculate predicted path yaw
   * @param bbox Bounding box data
   * @param base2map_transform Transformation matrix from base_link to map frame
   * @return std::optional<float> Predicted path yaw angle in map frame if valid, otherwise std::nullopt
   */
   std::optional<float> calculate_predicted_path_yaw(
    const BBox& bbox,
    const Eigen::Matrix4d& base2map_transform) const;

  };

} // namespace autoware::tensorrt_vad::vad_interface

#endif // AUTOWARE_TENSORRT_VAD_OUTPUT_CONVERTER_OBJECTS_CONVERTER_HPP
