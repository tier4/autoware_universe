#include "autoware/tensorrt_vad/vad_interface.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <cmath>

namespace autoware::tensorrt_vad
{

VadInterface::VadInterface(const VadInterfaceConfig& config, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
  : tf_buffer_(tf_buffer),
    target_image_width_(config.target_image_width),
    target_image_height_(config.target_image_height),
    input_image_width_(config.input_image_width),
    input_image_height_(config.input_image_height),
    detection_range_(config.detection_range),
    bev_h_(config.bev_h),
    bev_w_(config.bev_w),
    default_patch_angle_(config.default_patch_angle),
    default_command_(config.default_command),
    default_shift_(config.default_shift),
    image_normalization_param_mean_(config.image_normalization_param_mean),
    image_normalization_param_std_(config.image_normalization_param_std),
    vad2base_(config.vad2base),
    base2vad_(config.base2vad),
    map_colors_(config.map_colors),
    current_longitudinal_velocity_mps_(0.0f),
    prev_can_bus_(config.default_can_bus)
{
  // Mapping from Autoware camera index to VAD camera index
  autoware_to_vad_camera_mapping_ = config.autoware_to_vad_camera_mapping;
}

VadInputData VadInterface::convert_input(const VadInputTopicData & vad_input_topic_data)
{
  VadInputData vad_input_data;

  float scale_width = target_image_width_ / static_cast<float>(input_image_width_);
  float scale_height = target_image_height_ / static_cast<float>(input_image_height_);

  // Process lidar2img transformation
  vad_input_data.lidar2img_ = process_lidar2img(
    vad_input_topic_data.camera_infos,
    scale_width, scale_height
  );
  
  // Process can_bus and shift data
  vad_input_data.can_bus_ = process_can_bus(
    vad_input_topic_data.kinematic_state,
    vad_input_topic_data.acceleration,
    prev_can_bus_
  );
  vad_input_data.shift_ = process_shift(vad_input_data.can_bus_, prev_can_bus_);
  
  // Calculate current longitudinal velocity (node_timestep = 100ms = 0.1s)
  constexpr double node_timestep = 0.1;
  current_longitudinal_velocity_mps_ = calculate_current_longitudinal_velocity(
    vad_input_data.can_bus_, prev_can_bus_, node_timestep);
  
  // Process image data
  vad_input_data.camera_images_ = process_image(vad_input_topic_data.images);
  
  // Set default command
  vad_input_data.command_ = default_command_;
  
  // Update prev_can_bus_ for next iteration
  prev_can_bus_ = vad_input_data.can_bus_;
  
  return vad_input_data;
}

VadOutputTopicData VadInterface::convert_output(
  const VadOutputData & vad_output_data,
  const rclcpp::Time & stamp,
  double trajectory_timestep,
  const Eigen::Matrix4f & base2map_transform) const
{
  VadOutputTopicData output_topic_data;

  // Convert candidate trajectories
  output_topic_data.candidate_trajectories = process_candidate_trajectories(
    vad_output_data.predicted_trajectories_, stamp, trajectory_timestep, base2map_transform);
  
  // Convert trajectory
  output_topic_data.trajectory = process_trajectory(
    vad_output_data.predicted_trajectory_, stamp, trajectory_timestep, base2map_transform);

  // Convert map_points from VAD coordinate system to Autoware coordinate system
  output_topic_data.map_points = process_map_points(vad_output_data.map_polylines_, stamp, base2map_transform);

  return output_topic_data;
}

std::optional<Eigen::Matrix4f> VadInterface::lookup_base2cam(tf2_ros::Buffer & buffer, const std::string & source_frame) const
{
  std::string target_frame = "base_link";

  try {
    geometry_msgs::msg::TransformStamped lookup_result =
        buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    
    // Translation
    transform_matrix(0, 3) = lookup_result.transform.translation.x;
    transform_matrix(1, 3) = lookup_result.transform.translation.y;
    transform_matrix(2, 3) = lookup_result.transform.translation.z;
    
    // Rotation (quaternion to rotation matrix conversion)
    Eigen::Quaternionf q(
        lookup_result.transform.rotation.w,
        lookup_result.transform.rotation.x,
        lookup_result.transform.rotation.y,
        lookup_result.transform.rotation.z);
    transform_matrix.block<3, 3>(0, 0) = q.toRotationMatrix();

    // calculate the inverse transformation
    Eigen::Matrix4f transform_matrix_inverse = transform_matrix.inverse();

    return transform_matrix_inverse;

  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("VadInterface"), "Failed to get TF transformation: %s -> %s. Reason: %s",
                 source_frame.c_str(), target_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

Eigen::Matrix4f VadInterface::create_viewpad(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info) const
{
  Eigen::Matrix3f k_matrix;
  for (int32_t i = 0; i < 3; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      k_matrix(i, j) = camera_info->k[i * 3 + j];
    }
  }
  
  // Create viewpad
  Eigen::Matrix4f viewpad = Eigen::Matrix4f::Zero();
  viewpad.block<3, 3>(0, 0) = k_matrix;
  viewpad(3, 3) = 1.0f;
  
  return viewpad;
}

Eigen::Matrix4f VadInterface::apply_scaling(const Eigen::Matrix4f & lidar2img, float scale_width, float scale_height) const
{
  Eigen::Matrix4f scale_matrix = Eigen::Matrix4f::Identity();
  scale_matrix(0, 0) = scale_width;
  scale_matrix(1, 1) = scale_height;
  return scale_matrix * lidar2img;
}

std::vector<float> VadInterface::matrix_to_flat(const Eigen::Matrix4f & matrix) const
{
  std::vector<float> flat(16);
  int32_t k = 0;
  for (int32_t i = 0; i < 4; ++i) {
    for (int32_t j = 0; j < 4; ++j) {
      flat[k++] = matrix(i, j);
    }
  }
  return flat;
}

Lidar2ImgData VadInterface::process_lidar2img(
    const std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> & camera_infos,
    float scale_width, float scale_height) const
{
  std::vector<float> frame_lidar2img(16 * 6, 0.0f); // Reserve space for 6 cameras

  // Process each camera
  for (int32_t autoware_camera_id = 0; autoware_camera_id < 6; ++autoware_camera_id) {
    if (!camera_infos[autoware_camera_id]) {
      continue;
    }

    auto base2cam_opt = lookup_base2cam(*tf_buffer_, camera_infos[autoware_camera_id]->header.frame_id);
    if (!base2cam_opt) continue;
    Eigen::Matrix4f base2cam = *base2cam_opt;

    Eigen::Matrix4f viewpad = create_viewpad(camera_infos[autoware_camera_id]);
    Eigen::Matrix4f lidar2cam_rt = base2cam * vad2base_;
    Eigen::Matrix4f lidar2img = viewpad * lidar2cam_rt;

    // Apply scaling
    Eigen::Matrix4f  lidar2img_scaled = apply_scaling(lidar2img, scale_width, scale_height);

    std::vector<float> lidar2img_flat = matrix_to_flat(lidar2img_scaled);

    // Store result at VAD camera ID position after lidar2img calculation
    int32_t vad_camera_id = autoware_to_vad_camera_mapping_.at(autoware_camera_id);
    if (vad_camera_id >= 0 && vad_camera_id < 6) {
      std::copy(lidar2img_flat.begin(), lidar2img_flat.end(),
                frame_lidar2img.begin() + vad_camera_id * 16);
    }
  }

  return frame_lidar2img;
}

std::vector<float> VadInterface::normalize_image(unsigned char *image_data, int32_t width, int32_t height) const
{
  std::vector<float> normalized_image_data(width * height * 3);
  
  // Process in BGR order
  for (int32_t c = 0; c < 3; ++c) {
    for (int32_t h = 0; h < height; ++h) {
      for (int32_t w = 0; w < width; ++w) {
        int32_t src_idx = (h * width + w) * 3 + (2 - c); // BGR -> RGB
        int32_t dst_idx = c * height * width + h * width + w; // CHW format
        float pixel_value = static_cast<float>(image_data[src_idx]);
        normalized_image_data[dst_idx] = (pixel_value - image_normalization_param_mean_[c]) / image_normalization_param_std_[c];
      }
    }
  }
  
  return normalized_image_data;
}

CameraImagesData VadInterface::process_image(
  const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & images) const
{
  std::vector<std::vector<float>> frame_images;
  frame_images.resize(6); // Initialize in VAD camera order

  // Process each camera image
  for (int32_t autoware_idx = 0; autoware_idx < 6; ++autoware_idx) {
    const auto &image_msg = images[autoware_idx];

    // Create cv::Mat from sensor_msgs::msg::Image
    cv::Mat bgr_img;
    if (image_msg->encoding == "bgr8") {
      // For BGR8, use data directly
      bgr_img = cv::Mat(image_msg->height, image_msg->width, CV_8UC3, 
                        const_cast<uint8_t*>(image_msg->data.data()), image_msg->step);
    } else {
      throw std::runtime_error("Unsupported image encoding: " + image_msg->encoding);
    }

    if (bgr_img.empty()) {
      throw std::runtime_error("Failed to decode image data: " + std::to_string(autoware_idx));
    }

    // Convert to RGB
    cv::Mat rgb_img;
    cv::cvtColor(bgr_img, rgb_img, cv::COLOR_BGR2RGB);

    // Resize if size differs from target
    if (rgb_img.cols != target_image_width_ || rgb_img.rows != target_image_height_) {
      cv::resize(rgb_img, rgb_img, cv::Size(target_image_width_, target_image_height_));
    }

    // Normalize image
    std::vector<float> normalized_image_data = normalize_image(rgb_img.data, rgb_img.cols, rgb_img.rows);

    // Store in VAD camera order
    int32_t vad_idx = autoware_to_vad_camera_mapping_.at(autoware_idx);
    frame_images[vad_idx] = normalized_image_data;
  }

  // Concatenate image data
  std::vector<float> concatenated_data;
  size_t single_camera_size = 3 * target_image_height_ * target_image_width_;
  concatenated_data.reserve(single_camera_size * 6);

  // Camera order: {0, 1, 2, 3, 4, 5}
  for (int32_t camera_idx = 0; camera_idx < 6; ++camera_idx) {
    const auto &img_data = frame_images[camera_idx];
    if (img_data.size() != single_camera_size) {
      throw std::runtime_error("Invalid image size: " +
                               std::to_string(camera_idx));
    }
    concatenated_data.insert(concatenated_data.end(), img_data.begin(),
                             img_data.end());
  }

  return concatenated_data;
}

CanBusData VadInterface::process_can_bus(
  const nav_msgs::msg::Odometry::ConstSharedPtr & kinematic_state,
  const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr & acceleration,
  const std::vector<float> & prev_can_bus) const
{
  CanBusData can_bus(18, 0.0f);

  // Apply Autoware to VAD base_link coordinate transformation to position
  auto [vad_x, vad_y, vad_z] =
      aw2vad_xyz(kinematic_state->pose.pose.position.x,
                kinematic_state->pose.pose.position.y,
                kinematic_state->pose.pose.position.z);

  // translation (0:3)
  can_bus[0] = vad_x;
  can_bus[1] = vad_y;
  can_bus[2] = vad_z;

  // Apply Autoware to VAD base_link coordinate transformation to orientation
  Eigen::Quaternionf q_aw(
      kinematic_state->pose.pose.orientation.w,
      kinematic_state->pose.pose.orientation.x,
      kinematic_state->pose.pose.orientation.y,
      kinematic_state->pose.pose.orientation.z);

  Eigen::Quaternionf q_vad = aw2vad_quaternion(q_aw);

  // rotation (3:7)
  can_bus[3] = q_vad.x();
  can_bus[4] = q_vad.y();
  can_bus[5] = q_vad.z();
  can_bus[6] = q_vad.w();

  // Apply Autoware to VAD base_link coordinate transformation to acceleration
  auto [vad_ax, vad_ay, vad_az] =
      aw2vad_xyz(acceleration->accel.accel.linear.x,
                acceleration->accel.accel.linear.y,
                acceleration->accel.accel.linear.z);

  // acceleration (7:10)
  can_bus[7] = vad_ax;
  can_bus[8] = vad_ay;
  can_bus[9] = vad_az;

  // Apply Autoware to VAD base_link coordinate transformation to angular velocity
  auto [vad_wx, vad_wy, vad_wz] =
      aw2vad_xyz(kinematic_state->twist.twist.angular.x,
                kinematic_state->twist.twist.angular.y,
                kinematic_state->twist.twist.angular.z);

  // angular velocity (10:13)
  can_bus[10] = vad_wx;
  can_bus[11] = vad_wy;
  can_bus[12] = vad_wz;

  // Apply Autoware to VAD base_link coordinate transformation to velocity
  auto [vad_vx, vad_vy, vad_vz] =
      aw2vad_xyz(kinematic_state->twist.twist.linear.x,
                kinematic_state->twist.twist.linear.y,
                0.0f); // Set z-direction velocity to 0

  // velocity (13:16)
  can_bus[13] = vad_vx;
  can_bus[14] = vad_vy;
  can_bus[15] = vad_vz;

  // Calculate patch_angle[rad] (16)
  double yaw = std::atan2(
      2.0 * (can_bus[6] * can_bus[5] + can_bus[3] * can_bus[4]),
      1.0 - 2.0 * (can_bus[4] * can_bus[4] + can_bus[5] * can_bus[5]));
  if (yaw < 0)
    yaw += 2 * M_PI;
  can_bus[16] = static_cast<float>(yaw);

  // Calculate patch_angle[deg] (17)
  if (!prev_can_bus.empty()) {
    float prev_angle = prev_can_bus[16];
    can_bus[17] = (yaw - prev_angle) * 180.0f / M_PI;
  } else {
    can_bus[17] = default_patch_angle_; // Default value for first frame
  }

  return can_bus;
}

ShiftData VadInterface::process_shift(
  const CanBusData & can_bus,
  const CanBusData & prev_can_bus) const
{
  if (prev_can_bus.empty()) {
    return default_shift_;
  }

  float delta_x = can_bus[0] - prev_can_bus[0];  // translation difference
  float delta_y = can_bus[1] - prev_can_bus[1];  // translation difference
  float patch_angle_rad = can_bus[16];  // current patch_angle[rad]

  float real_w = detection_range_[3] - detection_range_[0];
  float real_h = detection_range_[4] - detection_range_[1];
  float grid_length[] = {real_h / bev_h_, real_w / bev_w_};

  float ego_angle = patch_angle_rad / M_PI * 180.0;
  float grid_length_y = grid_length[0];
  float grid_length_x = grid_length[1];

  float translation_length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
  float translation_angle = std::atan2(delta_y, delta_x) / M_PI * 180.0;
  float bev_angle = ego_angle - translation_angle;

  float shift_y = translation_length * std::cos(bev_angle / 180.0 * M_PI) /
                  grid_length_y / bev_h_;
  float shift_x = translation_length * std::sin(bev_angle / 180.0 * M_PI) /
                  grid_length_x / bev_w_;

  return {shift_x, shift_y};
}

std::tuple<float, float, float> VadInterface::aw2vad_xyz(float aw_x, float aw_y, float aw_z) const
{
  // Convert Autoware(base_link) coordinates [x, y, z] to VAD base_link coordinates
  Eigen::Vector4f aw_xyz(aw_x, aw_y, aw_z, 1.0f);
  Eigen::Vector4f vad_xyz = base2vad_ * aw_xyz;
  return {vad_xyz[0], vad_xyz[1], vad_xyz[2]};
}

std::tuple<float, float, float> VadInterface::vad2aw_xyz(float vad_x, float vad_y, float vad_z) const
{
  // Convert VAD base_link coordinates [x, y, z] to Autoware(base_link) coordinates
  Eigen::Vector4f vad_xyz(vad_x, vad_y, vad_z, 1.0f);
  Eigen::Vector4f aw_xyz = vad2base_ * vad_xyz;
  return {aw_xyz[0], aw_xyz[1], aw_xyz[2]};
}

Eigen::Quaternionf VadInterface::aw2vad_quaternion(const Eigen::Quaternionf & q_aw) const
{
  // base2vad_ rotation part to quaternion conversion
  Eigen::Matrix3f rot = base2vad_.block<3,3>(0,0);
  Eigen::Quaternionf q_v2a(rot); // base_link→vad rotation
  Eigen::Quaternionf q_v2a_inv = q_v2a.conjugate(); // For unit quaternion, inverse = conjugate
  // q_vad = q_v2a * q_aw * q_v2a_inv
  return q_v2a * q_aw * q_v2a_inv;
}

geometry_msgs::msg::Quaternion VadInterface::create_quaternion_from_yaw(double yaw) const
{
  geometry_msgs::msg::Quaternion q{};
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> VadInterface::create_trajectory_points(
  const std::vector<float> & predicted_trajectory,
  double trajectory_timestep,
  const Eigen::Matrix4f & base2map_transform) const
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  
  // function to transform direction vector from base coordinate system to map coordinate system
  auto transform_direction_to_map = [&base2map_transform](float base_dx, float base_dy) -> float {
    Eigen::Vector3f base_direction(base_dx, base_dy, 0.0f);
    Eigen::Vector3f map_direction = base2map_transform.block<3, 3>(0, 0) * base_direction;
    return std::atan2(map_direction.y(), map_direction.x());
  };
  
  // Add 0-second point (0,0)
  autoware_planning_msgs::msg::TrajectoryPoint initial_point;
  Eigen::Vector4f init_ego_position = base2map_transform * Eigen::Vector4f(0.0, 0.0, 0.0, 1.0);
  initial_point.pose.position.x = init_ego_position[0];
  initial_point.pose.position.y = init_ego_position[1];
  initial_point.pose.position.z = init_ego_position[2];
  // Convert initial direction to map coordinate system (when facing x-direction in base coordinate system)
  initial_point.pose.orientation = create_quaternion_from_yaw(transform_direction_to_map(1.0f, 0.0f));
  initial_point.longitudinal_velocity_mps = 2.5;
  initial_point.lateral_velocity_mps = 0.0;
  initial_point.acceleration_mps2 = 0.0;
  initial_point.heading_rate_rps = 0.0;
  initial_point.time_from_start.sec = 0;
  initial_point.time_from_start.nanosec = 0;
  points.push_back(initial_point);

  double prev_x = init_ego_position[0];
  double prev_y = init_ego_position[1];
  auto prev_orientation = initial_point.pose.orientation;

  for (size_t i = 0; i < predicted_trajectory.size(); i += 2) {
    autoware_planning_msgs::msg::TrajectoryPoint point;

    float vad_x = predicted_trajectory[i];
    float vad_y = predicted_trajectory[i + 1];

    auto [aw_x, aw_y, aw_z] = vad2aw_xyz(vad_x, vad_y, 0.0f);
    Eigen::Vector4f base_link_position(aw_x, aw_y, 0.0, 1.0);
    Eigen::Vector4f map_position = base2map_transform * base_link_position;

    point.pose.position.x = map_position[0];
    point.pose.position.y = map_position[1];
    point.pose.position.z = map_position[2];

    if (i + 2 < predicted_trajectory.size()) {
      float vad_dx = predicted_trajectory[i + 2] - predicted_trajectory[i];
      float vad_dy = predicted_trajectory[i + 3] - predicted_trajectory[i + 1];
      auto [aw_dx, aw_dy, aw_dz] = vad2aw_xyz(vad_dx, vad_dy, 0.0f);
      
      float yaw = transform_direction_to_map(aw_dx, aw_dy);
      point.pose.orientation = create_quaternion_from_yaw(yaw);
    } else {
      point.pose.orientation = prev_orientation;
    }

    // Calculate velocity (divide distance from previous point by time interval)
    auto distance = std::hypot(point.pose.position.x - prev_x, point.pose.position.y - prev_y);
    point.longitudinal_velocity_mps = static_cast<float>(distance / trajectory_timestep);
    
    point.lateral_velocity_mps = 0.0;
    point.acceleration_mps2 = 0.0;
    point.heading_rate_rps = 0.0;

    // Set time_from_start (1 second, 2 seconds, 3 seconds, 4 seconds, 5 seconds, 6 seconds)
    size_t point_index = i / 2;
    double time_sec = (point_index + 1) * trajectory_timestep;
    point.time_from_start.sec = static_cast<int32_t>(time_sec);
    point.time_from_start.nanosec = static_cast<uint32_t>((time_sec - point.time_from_start.sec) * 1e9);

    // Save current position for next calculation
    prev_x = point.pose.position.x;
    prev_y = point.pose.position.y;
    prev_orientation = point.pose.orientation;

    points.push_back(point);
  }

  return points;
}

autoware_internal_planning_msgs::msg::CandidateTrajectories VadInterface::process_candidate_trajectories(
  const std::map<int32_t, std::vector<float>> & predicted_trajectories,
  const rclcpp::Time & stamp,
  double trajectory_timestep,
  const Eigen::Matrix4f & base2map_transform) const
{
  autoware_internal_planning_msgs::msg::CandidateTrajectories candidate_trajectories_msg;

  // Add each command's trajectory as CandidateTrajectory
  for (const auto& [command_idx, trajectory] : predicted_trajectories) {
    autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_trajectory;
    
    // Set header
    candidate_trajectory.header.stamp = stamp;
    candidate_trajectory.header.frame_id = "map";
    
    // Set generator_id (unique UUID)
    candidate_trajectory.generator_id = autoware_utils_uuid::generate_uuid();

    candidate_trajectory.points = create_trajectory_points(trajectory, trajectory_timestep, base2map_transform);

    candidate_trajectories_msg.candidate_trajectories.push_back(candidate_trajectory);

    // Add GeneratorInfo for each command
    autoware_internal_planning_msgs::msg::GeneratorInfo generator_info;
    generator_info.generator_id = autoware_utils_uuid::generate_uuid();
    generator_info.generator_name.data = "autoware_tensorrt_vad_cmd_" + std::to_string(command_idx);
    candidate_trajectories_msg.generator_info.push_back(generator_info);
  }

  return candidate_trajectories_msg;
}

autoware_planning_msgs::msg::Trajectory VadInterface::process_trajectory(
  const std::vector<float> & predicted_trajectory,
  const rclcpp::Time & stamp,
  double trajectory_timestep,
  const Eigen::Matrix4f & base2map_transform) const
{
  autoware_planning_msgs::msg::Trajectory trajectory_msg;

  // Set header
  trajectory_msg.header.stamp = stamp;
  trajectory_msg.header.frame_id = "map";

  trajectory_msg.points = create_trajectory_points(predicted_trajectory, trajectory_timestep, base2map_transform);

  return trajectory_msg;
}


/**
 * @brief Process map points from VAD to Autoware coordinate system.
 * @param vad_map_polylines vector of MapPolyline
 * @param stamp timestamp in this frame
 * @param base2map_transform base_link to map transformation matrix
 * @return visualization_msgs::msg::MarkerArray 
 */
visualization_msgs::msg::MarkerArray VadInterface::process_map_points(
    const std::vector<MapPolyline>& vad_map_polylines,
    const rclcpp::Time& stamp,
    const Eigen::Matrix4f& base2map_transform) const
{
    visualization_msgs::msg::MarkerArray marker_array;

    int32_t marker_id = 0;
    for (const auto& map_polyline : vad_map_polylines) {
        const std::string& type = map_polyline.type;
        const auto& polyline = map_polyline.points;

        if (polyline.empty()) {
            ++marker_id;
            continue;  // if polyline is empty, skip
        }

        visualization_msgs::msg::Marker marker;
        marker.ns = type;  // namespace shows the type of the polyline
        marker.id = marker_id++;  // set unique ID for each marker
        marker.header.frame_id = "map";
        marker.header.stamp = stamp;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // orientation is fixed.
        marker.pose.orientation.w = 1.0;

        // Set color based on type
        auto color_it = map_colors_.find(type);
        if (color_it != map_colors_.end()) {
            const auto& color = color_it->second;
            marker.color.r = color[0];
            marker.color.g = color[1];
            marker.color.b = color[2];
        } else {
            // use white color as default
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
        }
        marker.color.a = 0.8;

        // Transform each point in the polyline and add to the marker
        for (const auto& point : polyline) {
            if (point.size() >= 2) {
                float vad_x = point[0];
                float vad_y = point[1];
                
                auto [aw_x, aw_y, aw_z] = vad2aw_xyz(vad_x, vad_y, 0.0f);
                
                Eigen::Vector4f base_point(aw_x, aw_y, 0.0f, 1.0f);
                Eigen::Vector4f map_point = base2map_transform * base_point;
                
                geometry_msgs::msg::Point geometry_point;
                geometry_point.x = map_point[0];
                geometry_point.y = map_point[1];
                geometry_point.z = map_point[2];
                
                marker.points.push_back(geometry_point);
            }
        }

        // Decide how to display the marker
        if (marker.points.size() >= 2) {
            // If there are 2 or more points, display as a line
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.scale.x = 0.1; // Line thickness
        } else {
            // If polyline does not have 2 or more points, skip
            continue;
        }

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}

float VadInterface::calculate_current_longitudinal_velocity(
  const std::vector<float> & can_bus,
  const std::vector<float> & prev_can_bus,
  double node_timestep) const
{
  if (prev_can_bus.empty() || can_bus.size() < 3 || prev_can_bus.size() < 3) {
    return 0.0f; // Return 0 if previous frame data is not available
  }

  // Calculate velocity from position data in can_bus (position: indices 0, 1)
  float delta_x = can_bus[0] - prev_can_bus[0];  // x-direction displacement
  float delta_y = can_bus[1] - prev_can_bus[1];  // y-direction displacement

  // Calculate 3D movement distance
  float distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);

  // Velocity = distance / time
  float velocity = distance / static_cast<float>(node_timestep);
  
  return velocity;
}

} // namespace autoware::tensorrt_vad

namespace autoware::tensorrt_vad
{

// VadInputTopicData class implementation
VadInputTopicData::VadInputTopicData(int32_t num_cameras) : num_cameras_(num_cameras) 
{
  images.resize(num_cameras_);
  camera_infos.resize(num_cameras_);
}

bool VadInputTopicData::is_complete() const 
{
  if (static_cast<int32_t>(images.size()) != num_cameras_ || 
      static_cast<int32_t>(camera_infos.size()) != num_cameras_) {
    return false;
  }
  
  for (int32_t i = 0; i < num_cameras_; ++i) {
    if (!images[i] || !camera_infos[i]) return false;
  }
  
  return kinematic_state != nullptr && 
         acceleration != nullptr;
}

void VadInputTopicData::reset() 
{
  images.clear();
  images.resize(num_cameras_);
  camera_infos.clear();
  camera_infos.resize(num_cameras_);
  kinematic_state.reset();
  acceleration.reset();
  frame_started_ = false;
}

void VadInputTopicData::set_image(std::size_t camera_id, const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
{
  ensure_frame_started(msg->header.stamp);
  if (camera_id < images.size()) {
    images[camera_id] = msg;
  }
}

void VadInputTopicData::set_camera_info(std::size_t camera_id, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) 
{
  ensure_frame_started(msg->header.stamp);
  if (camera_id < camera_infos.size()) {
    camera_infos[camera_id] = msg;
  }
}

void VadInputTopicData::set_kinematic_state(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) 
{
  ensure_frame_started(msg->header.stamp);
  kinematic_state = msg;
}

void VadInputTopicData::set_acceleration(const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr& msg) 
{
  ensure_frame_started(msg->header.stamp);
  acceleration = msg;
}

void VadInputTopicData::ensure_frame_started(const rclcpp::Time& msg_stamp) 
{
  if (!frame_started_) {
    stamp = msg_stamp;
    frame_started_ = true;
  }
}

// VadInterface class implementation
}
