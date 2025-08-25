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
   * @param transformer Reference to coordinate transformer
   * @param config Reference to VAD interface configuration
   */
  OutputObjectsConverter(const CoordinateTransformer& transformer, const VadInterfaceConfig& config);

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
    const Eigen::Matrix4f& base2map_transform) const;

private:
  /**
   * @brief Convert VAD object class to Autoware classification
   * @param object_class VAD object class index
   * @param confidence Classification confidence
   * @return autoware_perception_msgs::msg::ObjectClassification Autoware classification
   */
  autoware_perception_msgs::msg::ObjectClassification convert_classification(
    int32_t object_class,
    float confidence) const;

  /**
   * @brief Process predicted trajectory for an object
   * @param bbox Bounding box containing trajectory data
   * @param aw_z Object z-coordinate in Autoware frame
   * @param base2map_transform Transformation matrix from base_link to map frame
   * @param final_yaw Object orientation for fallback direction
   * @return std::vector<autoware_perception_msgs::msg::PredictedPath> Predicted paths
   */
  std::vector<autoware_perception_msgs::msg::PredictedPath> process_predicted_trajectories(
    const BBox& bbox,
    float aw_z,
    const Eigen::Matrix4f& base2map_transform,
    float final_yaw) const;

  /**
   * @brief Calculate object orientation from trajectory or bbox
   * @param bbox Bounding box data
   * @param aw_z Object z-coordinate in Autoware frame
   * @param base2map_transform Transformation matrix from base_link to map frame
   * @return float Final yaw angle in map frame
   */
  float calculate_object_orientation(
    const BBox& bbox,
    float aw_z,
    const Eigen::Matrix4f& base2map_transform) const;
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif // AUTOWARE_TENSORRT_VAD_OUTPUT_CONVERTER_OBJECTS_CONVERTER_HPP
