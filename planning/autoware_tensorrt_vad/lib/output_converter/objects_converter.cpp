#include "autoware/tensorrt_vad/output_converter/objects_converter.hpp"
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <cmath>

namespace autoware::tensorrt_vad::vad_interface {

OutputObjectsConverter::OutputObjectsConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config)
  : Converter(coordinate_transformer, config)
{
}

autoware_perception_msgs::msg::ObjectClassification OutputObjectsConverter::convert_classification(
  const int32_t object_class,
  const float confidence) const
{
  autoware_perception_msgs::msg::ObjectClassification classification;
  
  if (object_class >= 0 && object_class < static_cast<int32_t>(config_.class_mapping.size())) {
    // Get Autoware class name from class mapping array using VAD class index
    const std::string& autoware_class_name = config_.class_mapping[object_class];
    
    // Convert string to Autoware ObjectClassification enum
    if (autoware_class_name == "CAR") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
    } else if (autoware_class_name == "TRUCK") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::TRUCK;
    } else if (autoware_class_name == "BUS") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::BUS;
    } else if (autoware_class_name == "BICYCLE") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::BICYCLE;
    } else if (autoware_class_name == "MOTORCYCLE") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
    } else if (autoware_class_name == "PEDESTRIAN") {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
    } else {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    }
  } else {
    classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  }
  
  classification.probability = confidence;
  return classification;
}

std::optional<float> OutputObjectsConverter::calculate_predicted_path_yaw(
  const BBox& bbox,
  const float aw_z,
  const Eigen::Matrix4f& base2map_transform) const
{
  float max_confidence = 0.0f;
  std::optional<float> predicted_path_yaw = std::nullopt;

  for (int32_t mode = 0; mode < 6; ++mode) {
    const auto& pred_traj = bbox.trajectories[mode];
    if (pred_traj.confidence > max_confidence) {
      // Calculate direction from first 2 points
      float traj_vad_x1 = pred_traj.trajectory[0][0] + bbox.bbox[0];
      float traj_vad_y1 = pred_traj.trajectory[0][1] + bbox.bbox[1];
      float traj_vad_x2 = pred_traj.trajectory[1][0] + bbox.bbox[0];
      float traj_vad_y2 = pred_traj.trajectory[1][1] + bbox.bbox[1];

      auto [traj_aw_x1, traj_aw_y1, traj_aw_z1] = coordinate_transformer_.vad2aw_xyz(traj_vad_x1, traj_vad_y1, aw_z);
      auto [traj_aw_x2, traj_aw_y2, traj_aw_z2] = coordinate_transformer_.vad2aw_xyz(traj_vad_x2, traj_vad_y2, aw_z);

      Eigen::Vector4f pos1_base(traj_aw_x1, traj_aw_y1, traj_aw_z1, 1.0f);
      Eigen::Vector4f pos2_base(traj_aw_x2, traj_aw_y2, traj_aw_z2, 1.0f);
      Eigen::Vector4f pos1_map = base2map_transform * pos1_base;
      Eigen::Vector4f pos2_map = base2map_transform * pos2_base;

      float dx = pos2_map.x() - pos1_map.x();
      float dy = pos2_map.y() - pos1_map.y();
      if (std::sqrt(dx*dx + dy*dy) > 0.01) {
        predicted_path_yaw = std::atan2(dy, dx);
        max_confidence = pred_traj.confidence;
      }
    }
  }

  return predicted_path_yaw;
}

float OutputObjectsConverter::calculate_object_orientation(
  const BBox& bbox,
  const float aw_z,
  const Eigen::Matrix4f& base2map_transform) const
{
  float sin_theta = bbox.bbox[6];
  float cos_theta = bbox.bbox[7];
  float vad_yaw = std::atan2(sin_theta, cos_theta);

  // Get vehicle rotation from base2map_transform
  Eigen::Matrix3f rotation_matrix = base2map_transform.block<3, 3>(0, 0);
  float transform_yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
  float map_yaw = vad_yaw + transform_yaw;

  return calculate_predicted_path_yaw(bbox, aw_z, base2map_transform).value_or(map_yaw);
}

std::vector<autoware_perception_msgs::msg::PredictedPath> OutputObjectsConverter::process_predicted_trajectories(
  const BBox& bbox,
  const float aw_z,
  const Eigen::Matrix4f& base2map_transform,
  const float yaw) const
{
  std::vector<autoware_perception_msgs::msg::PredictedPath> predicted_paths;

  // Set predicted trajectories
  for (int32_t mode = 0; mode < 6; ++mode) {
    const auto& pred_traj = bbox.trajectories[mode];
    
    autoware_perception_msgs::msg::PredictedPath predicted_path;
    predicted_path.confidence = pred_traj.confidence;
    
    // Set time step (assuming 0.1 seconds per step for future trajectory)
    predicted_path.time_step.sec = 0;
    predicted_path.time_step.nanosec = 100000000;  // 0.1 seconds in nanoseconds
    
    // Transform each point of the trajectory
    for (int32_t ts = 0; ts < 6; ++ts) {
      geometry_msgs::msg::Pose pose;
      
      // Predicted trajectory is in relative coordinates (ego coordinate system), so transform to agent center
      float traj_vad_x = pred_traj.trajectory[ts][0] + bbox.bbox[0];  // Relative coordinates from agent center
      float traj_vad_y = pred_traj.trajectory[ts][1] + bbox.bbox[1];  // Relative coordinates from agent center
      auto [traj_aw_x, traj_aw_y, traj_aw_z] = coordinate_transformer_.vad2aw_xyz(traj_vad_x, traj_vad_y, aw_z);
      
      // Transform to map coordinate system
      Eigen::Vector4f traj_position_base(traj_aw_x, traj_aw_y, traj_aw_z, 1.0f);
      Eigen::Vector4f traj_position_map = base2map_transform * traj_position_base;
      
      pose.position.x = traj_position_map.x();
      pose.position.y = traj_position_map.y();
      pose.position.z = traj_position_map.z();
      
      // Calculate trajectory direction (direction to next point)
      // Default is object direction (using corrected yaw)
      float traj_yaw = yaw;
      
      if (ts < 5) {  // If next point exists
        float next_vad_x = pred_traj.trajectory[ts + 1][0] + bbox.bbox[0];
        float next_vad_y = pred_traj.trajectory[ts + 1][1] + bbox.bbox[1];
        auto [next_aw_x, next_aw_y, next_aw_z] = coordinate_transformer_.vad2aw_xyz(next_vad_x, next_vad_y, aw_z);
        
        // Transform next point to map coordinate system
        Eigen::Vector4f next_position_base(next_aw_x, next_aw_y, next_aw_z, 1.0f);
        Eigen::Vector4f next_position_map = base2map_transform * next_position_base;
        
        // Calculate direction vector in map coordinate system
        float dx = next_position_map.x() - traj_position_map.x();
        float dy = next_position_map.y() - traj_position_map.y();
        if (std::sqrt(dx*dx + dy*dy) > 0.01) {
          traj_yaw = std::atan2(dy, dx);
        }
      }
      
      pose.orientation = autoware_utils::create_quaternion_from_yaw(traj_yaw);
      
      predicted_path.path.push_back(pose);
    }
    
    // Add trajectory only if it contains points
    if (!predicted_path.path.empty()) {
      predicted_paths.push_back(predicted_path);
    }
  }

  return predicted_paths;
}

autoware_perception_msgs::msg::PredictedObjects OutputObjectsConverter::process_predicted_objects(
  const std::vector<BBox>& bboxes,
  const rclcpp::Time& stamp,
  const Eigen::Matrix4f& base2map_transform) const
{
  autoware_perception_msgs::msg::PredictedObjects predicted_objects;
  // Set header
  predicted_objects.header.stamp = stamp;
  predicted_objects.header.frame_id = "map";
  
  // Object class name mapping
  for (const auto& bbox : bboxes) {
    autoware_perception_msgs::msg::PredictedObject predicted_object;
    
    // Set object ID
    predicted_object.object_id = autoware_utils_uuid::generate_uuid();
    
    // Set existence probability
    predicted_object.existence_probability = bbox.confidence;
    
    // Set classification
    auto classification = convert_classification(bbox.object_class, bbox.confidence);
    predicted_object.classification.push_back(classification);

    // Set kinematics (position and orientation)
    // BBox format: [c_x, c_y, w, l, c_z, h, sin(theta), cos(theta), v_x, v_y]
    float z_offset = +1.9f;
    float vad_x = bbox.bbox[0];
    float vad_y = bbox.bbox[1];
    float vad_z = bbox.bbox[4] + z_offset;
    auto [aw_x, aw_y, aw_z] = coordinate_transformer_.vad2aw_xyz(vad_x, vad_y, vad_z);
    Eigen::Vector4f position_base(aw_x, aw_y, aw_z, 1.0f);
    Eigen::Vector4f position_map = base2map_transform * position_base;
    predicted_object.kinematics.initial_pose_with_covariance.pose.position.x = position_map.x();
    predicted_object.kinematics.initial_pose_with_covariance.pose.position.y = position_map.y();
    predicted_object.kinematics.initial_pose_with_covariance.pose.position.z = position_map.z();

    // Calculate object orientation
    float yaw = calculate_object_orientation(bbox, aw_z, base2map_transform);
    predicted_object.kinematics.initial_pose_with_covariance.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);

    // Set shape
    predicted_object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    predicted_object.shape.dimensions.x = bbox.bbox[3];  // length
    predicted_object.shape.dimensions.y = bbox.bbox[2];  // width
    predicted_object.shape.dimensions.z = bbox.bbox[5];  // height

    // Set velocity
    float v_x = bbox.bbox[8];
    float v_y = bbox.bbox[9];
    auto [aw_vx, aw_vy, aw_vz] = coordinate_transformer_.vad2aw_xyz(v_x, v_y, 0.0f);

    predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x = aw_vx;
    predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y = aw_vy;
    predicted_object.kinematics.initial_twist_with_covariance.twist.linear.z = 0.0f;

    // Process predicted trajectories
    auto predicted_paths = process_predicted_trajectories(bbox, aw_z, base2map_transform, final_yaw);
    predicted_object.kinematics.predicted_paths = predicted_paths;

    predicted_objects.objects.push_back(predicted_object);
  }

  return predicted_objects;
}

} // namespace autoware::tensorrt_vad::vad_interface
