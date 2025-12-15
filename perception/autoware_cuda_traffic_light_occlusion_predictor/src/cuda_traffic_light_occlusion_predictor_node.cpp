// Copyright 2023-2026 the Autoware Foundation
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

#include "autoware/cuda_traffic_light_occlusion_predictor/cuda_occlusion_predictor.hpp"
#include "autoware/cuda_traffic_light_occlusion_predictor/cuda_occlusion_kernels.hpp"  // For PointXYZ

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <perception_utils/prime_synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp>  // for ROS 2 Jazzy or newer
#else
#include <image_geometry/pinhole_camera_model.h>  // for ROS 2 Humble or older
#endif

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{

namespace
{
/**
 * @brief Calculate 3D ROI corners in camera frame (CPU-side, uses OpenCV)
 *
 * @param roi Traffic light ROI
 * @param pinhole_model Camera pinhole model
 * @param traffic_light_position_map Map of traffic light positions
 * @param tf_camera2map Transformation from camera to map frame
 * @param top_left Output: top-left corner in 3D
 * @param bottom_right Output: bottom-right corner in 3D
 */
void calcRoiVector3D(
  const tier4_perception_msgs::msg::TrafficLightRoi & roi,
  const image_geometry::PinholeCameraModel & pinhole_model,
  const std::map<lanelet::Id, tf2::Vector3> & traffic_light_position_map,
  const tf2::Transform & tf_camera2map,
  PointXYZ & top_left,
  PointXYZ & bottom_right)
{
  if (traffic_light_position_map.count(roi.traffic_light_id) == 0) {
    return;
  }
  double dist2cam = (tf_camera2map * traffic_light_position_map.at(roi.traffic_light_id)).length();

  // Project top-left corner
  {
    cv::Point2d pixel(roi.roi.x_offset, roi.roi.y_offset);
    cv::Point2d rectified_pixel = pinhole_model.rectifyPoint(pixel);
    cv::Point3d ray = pinhole_model.projectPixelTo3dRay(rectified_pixel);
    double ray_len = std::sqrt(ray.ddot(ray));
    top_left.x = dist2cam * ray.x / ray_len;
    top_left.y = dist2cam * ray.y / ray_len;
    top_left.z = dist2cam * ray.z / ray_len;
  }

  // Project bottom-right corner
  {
    cv::Point2d pixel(roi.roi.x_offset + roi.roi.width, roi.roi.y_offset + roi.roi.height);
    cv::Point2d rectified_pixel = pinhole_model.rectifyPoint(pixel);
    cv::Point3d ray = pinhole_model.projectPixelTo3dRay(rectified_pixel);
    double ray_len = std::sqrt(ray.ddot(ray));
    bottom_right.x = dist2cam * ray.x / ray_len;
    bottom_right.y = dist2cam * ray.y / ray_len;
    bottom_right.z = dist2cam * ray.z / ray_len;
  }
}
}  // anonymous namespace

// ============================================================================
// Node Implementation
// ============================================================================

class CudaTrafficLightOcclusionPredictorNode : public rclcpp::Node
{
public:
  explicit CudaTrafficLightOcclusionPredictorNode(const rclcpp::NodeOptions & node_options);

private:
  struct Config
  {
    double azimuth_occlusion_resolution_deg;
    double elevation_occlusion_resolution_deg;
    double max_valid_pt_dist;
    double max_image_cloud_delay;
    double max_wait_t;
    int max_occlusion_ratio;
    int64_t max_mem_pool_size_in_byte;
  };

  void mapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg);

  void pointcloudCallback(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr cloud_msg);

  void syncCallback(
    const tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr in_signal_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_roi_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr in_cam_info_msg,
    const uint8_t traffic_light_type);

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;

  // CUDA blackboard subscriber for zero-copy pointcloud
  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    pointcloud_sub_;

  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightArray>::SharedPtr signal_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<lanelet::Id, tf2::Vector3> traffic_light_position_map_;
  Config config_;

  std::shared_ptr<CudaOcclusionPredictor> cuda_occlusion_predictor_;

  typedef perception_utils::PrimeSynchronizer<
    tier4_perception_msgs::msg::TrafficLightArray, tier4_perception_msgs::msg::TrafficLightRoiArray,
    sensor_msgs::msg::CameraInfo>
    SynchronizerType;

  std::shared_ptr<SynchronizerType> synchronizer_;
  std::shared_ptr<SynchronizerType> synchronizer_ped_;

  // Latest CUDA pointcloud (stored from callback)
  cuda_blackboard::CudaPointCloud2::ConstSharedPtr latest_cloud_msg_;

  std::vector<bool> subscribed_;
  std::vector<int> occlusion_ratios_;
  tier4_perception_msgs::msg::TrafficLightArray out_msg_;
};

CudaTrafficLightOcclusionPredictorNode::CudaTrafficLightOcclusionPredictorNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_traffic_light_occlusion_predictor_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;

  // Subscribers
  map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&CudaTrafficLightOcclusionPredictorNode::mapCallback, this, _1));

  // Publishers
  signal_pub_ =
    create_publisher<tier4_perception_msgs::msg::TrafficLightArray>("~/output/traffic_signals", 1);

  // Configuration parameters
  config_.azimuth_occlusion_resolution_deg =
    declare_parameter<double>("azimuth_occlusion_resolution_deg");
  config_.elevation_occlusion_resolution_deg =
    declare_parameter<double>("elevation_occlusion_resolution_deg");
  config_.max_valid_pt_dist = declare_parameter<double>("max_valid_pt_dist");
  config_.max_image_cloud_delay = declare_parameter<double>("max_image_cloud_delay");
  config_.max_wait_t = declare_parameter<double>("max_wait_t");
  config_.max_occlusion_ratio = declare_parameter<int>("max_occlusion_ratio");
  config_.max_mem_pool_size_in_byte =
    declare_parameter<int64_t>("max_mem_pool_size_in_byte", 1e9);

  // Create CUDA occlusion predictor
  CudaOcclusionPredictorParameters cuda_params;
  cuda_params.max_valid_pt_distance = static_cast<float>(config_.max_valid_pt_dist);
  cuda_params.azimuth_occlusion_resolution_deg =
    static_cast<float>(config_.azimuth_occlusion_resolution_deg);
  cuda_params.elevation_occlusion_resolution_deg =
    static_cast<float>(config_.elevation_occlusion_resolution_deg);
  cuda_params.min_dist_from_occlusion_to_tl = 5.0f;  // Hardcoded as in original
  cuda_params.horizontal_sample_num = 20;            // Hardcoded as in original
  cuda_params.vertical_sample_num = 20;              // Hardcoded as in original

  try {
    cuda_occlusion_predictor_ = std::make_shared<CudaOcclusionPredictor>(
      cuda_params, config_.max_mem_pool_size_in_byte);

    // Log GPU information after successful initialization
    int current_device = 0;
    cudaGetDevice(&current_device);
    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, current_device);
    RCLCPP_INFO(
      this->get_logger(),
      "CUDA Traffic Light Occlusion Predictor initialized on GPU: %s (Compute Capability: %d.%d)",
      prop.name, prop.major, prop.minor);
    RCLCPP_INFO(
      this->get_logger(),
      "CUDA memory pool created with max size: %.2f GB",
      config_.max_mem_pool_size_in_byte / 1e9);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize CUDA occlusion predictor: %s", e.what());
    throw;
  }

  // CUDA blackboard subscriber for pointcloud (zero-copy!)
  pointcloud_sub_ = std::make_shared<
    cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
    *this, "~/input/cloud",
    std::bind(&CudaTrafficLightOcclusionPredictorNode::pointcloudCallback, this, _1));

  // Create synchronizers (without pointcloud - we handle it separately)
  const std::vector<std::string> topics{
    "~/input/car/traffic_signals", "~/input/rois", "~/input/camera_info"};
  const std::vector<rclcpp::QoS> qos(topics.size(), rclcpp::SensorDataQoS());
  synchronizer_ = std::make_shared<SynchronizerType>(
    this, topics, qos,
    std::bind(
      &CudaTrafficLightOcclusionPredictorNode::syncCallback, this, _1, _2, _3,
      tier4_perception_msgs::msg::TrafficLightRoi::CAR_TRAFFIC_LIGHT),
    config_.max_wait_t);

  const std::vector<std::string> topics_ped{
    "~/input/pedestrian/traffic_signals", "~/input/rois", "~/input/camera_info"};
  const std::vector<rclcpp::QoS> qos_ped(topics_ped.size(), rclcpp::SensorDataQoS());
  synchronizer_ped_ = std::make_shared<SynchronizerType>(
    this, topics_ped, qos_ped,
    std::bind(
      &CudaTrafficLightOcclusionPredictorNode::syncCallback, this, _1, _2, _3,
      tier4_perception_msgs::msg::TrafficLightRoi::PEDESTRIAN_TRAFFIC_LIGHT),
    config_.max_wait_t);

  subscribed_.resize(2, false);
  
  RCLCPP_INFO(this->get_logger(), "CUDA blackboard subscription created for zero-copy pointcloud transfer");
}

void CudaTrafficLightOcclusionPredictorNode::mapCallback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg)
{
  traffic_light_position_map_.clear();
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();

  lanelet::utils::conversion::fromBinMsg(*input_msg, lanelet_map_ptr);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
       ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (auto lsp : lights) {
      if (!lsp.isLineString()) {  // traffic lights must be linestrings
        continue;
      }
      lanelet::ConstLineString3d string3d = static_cast<lanelet::ConstLineString3d>(lsp);
      traffic_light_position_map_[lsp.id()] =
        autoware::traffic_light_utils::getTrafficLightCenter(string3d);
    }
  }
}

// Pointcloud callback - stores latest CUDA blackboard message
void CudaTrafficLightOcclusionPredictorNode::pointcloudCallback(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr cloud_msg)
{
  latest_cloud_msg_ = cloud_msg;
}

void CudaTrafficLightOcclusionPredictorNode::syncCallback(
  const tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr in_signal_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_roi_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr in_cam_info_msg,
  const uint8_t traffic_light_type)
{
  std::vector<int> occlusion_ratios;
  
  // Check if we have valid inputs including the latest CUDA pointcloud
  if (!latest_cloud_msg_ || in_cam_info_msg == nullptr || in_roi_msg == nullptr) {
    occlusion_ratios.resize(in_signal_msg->signals.size(), 0);
  } else {
    size_t not_detected_roi = 0;
    tier4_perception_msgs::msg::TrafficLightRoiArray selected_roi_msg;
    selected_roi_msg.rois.reserve(in_roi_msg->rois.size());
    for (size_t i = 0; i < in_roi_msg->rois.size(); ++i) {
      // not detected roi
      if (in_roi_msg->rois[i].roi.height == 0) {
        not_detected_roi++;
        continue;
      }
      if (in_roi_msg->rois.at(i).traffic_light_type == traffic_light_type) {
        selected_roi_msg.rois.push_back(in_roi_msg->rois.at(i));
      }
    }

    tier4_perception_msgs::msg::TrafficLightArray out_msg = *in_signal_msg;

    if (selected_roi_msg.rois.size() != in_signal_msg->signals.size() - not_detected_roi) {
      occlusion_ratios.resize(in_signal_msg->signals.size(), 0);
    } else {
      // Step 1: Get transformations (CPU-side, C++)
      Eigen::Matrix4d camera2cloud_eigen;
      try {
        camera2cloud_eigen =
          tf2::transformToEigen(tf_buffer_.lookupTransform(
                                  in_cam_info_msg->header.frame_id, latest_cloud_msg_->header.frame_id,
                                  rclcpp::Time(in_cam_info_msg->header.stamp),
                                  rclcpp::Duration::from_seconds(0.2)))
            .matrix();
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(), "Error: cannot get transform from "
                                << in_cam_info_msg->header.frame_id << " to "
                                << latest_cloud_msg_->header.frame_id);
        occlusion_ratios.resize(in_signal_msg->signals.size(), 0);
        return;
      }

      // Convert Eigen matrix to float array (row-major)
      float camera2cloud_transform[16];
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          camera2cloud_transform[i * 4 + j] = static_cast<float>(camera2cloud_eigen(i, j));
        }
      }

      // Step 2: Calculate ROI 3D points (CPU-side, uses OpenCV)
      std::vector<PointXYZ> roi_3d_points;
      roi_3d_points.reserve(selected_roi_msg.rois.size() * 2);  // [top_left, bottom_right] per ROI

      image_geometry::PinholeCameraModel pinhole_model;
      pinhole_model.fromCameraInfo(*in_cam_info_msg);

      // Get transformation from map to camera
      tf2::Transform tf_camera2map;
      try {
        geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_.lookupTransform(
          in_cam_info_msg->header.frame_id, "map",
          rclcpp::Time(in_cam_info_msg->header.stamp), rclcpp::Duration::from_seconds(0.2));
        tf2::fromMsg(tf_stamped.transform, tf_camera2map);
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Error: cannot get transform from " << in_cam_info_msg->header.frame_id << " to map");
        occlusion_ratios.resize(in_signal_msg->signals.size(), 0);
        return;
      }

      for (size_t i = 0; i < selected_roi_msg.rois.size(); i++) {
        PointXYZ top_left, bottom_right;
        calcRoiVector3D(
          selected_roi_msg.rois[i], pinhole_model, traffic_light_position_map_, tf_camera2map,
          top_left, bottom_right);
        roi_3d_points.push_back(top_left);
        roi_3d_points.push_back(bottom_right);
      }

      // Step 3: Call CUDA prediction (using CUDA blackboard - zero copy!)
      std::vector<int> cuda_occlusion_ratios;
      
      try {
        // Call CUDA prediction with blackboard message (zero-copy if from CUDA source!)
        cuda_occlusion_predictor_->predict(
          latest_cloud_msg_,
          camera2cloud_transform,
          roi_3d_points,
          cuda_occlusion_ratios);

        // Map CUDA occlusion ratios back to signal indices
        occlusion_ratios.resize(in_signal_msg->signals.size(), 0);
        size_t cuda_idx = 0;
        for (size_t i = 0; i < in_roi_msg->rois.size(); ++i) {
          if (in_roi_msg->rois[i].roi.height == 0) {
            continue;  // Skip undetected ROIs
          }
          if (in_roi_msg->rois.at(i).traffic_light_type == traffic_light_type) {
            if (cuda_idx < cuda_occlusion_ratios.size()) {
              occlusion_ratios[i] = cuda_occlusion_ratios[cuda_idx];
              cuda_idx++;
            }
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "CUDA occlusion prediction failed: " << e.what() << ". Skipping this frame.");
        occlusion_ratios.resize(in_signal_msg->signals.size(), 0);
      }
    }
  }

  size_t predicted_num = out_msg_.signals.size();

  for (size_t i = 0; i < occlusion_ratios.size(); i++) {
    out_msg_.signals.push_back(in_signal_msg->signals.at(i));

    if (occlusion_ratios[i] >= config_.max_occlusion_ratio) {
      autoware::traffic_light_utils::setSignalUnknown(out_msg_.signals.at(predicted_num + i), 0.0);
    }
  }

  // push back not detected rois
  for (size_t i = occlusion_ratios.size(); i < in_signal_msg->signals.size(); ++i) {
    out_msg_.signals.push_back(in_signal_msg->signals[i]);
  }

  subscribed_.at(traffic_light_type) = true;

  if (std::all_of(subscribed_.begin(), subscribed_.end(), [](bool v) { return v; })) {
    auto pub_msg = std::make_unique<tier4_perception_msgs::msg::TrafficLightArray>(out_msg_);
    pub_msg->header = in_signal_msg->header;
    signal_pub_->publish(std::move(pub_msg));
    out_msg_.signals.clear();
    std::fill(subscribed_.begin(), subscribed_.end(), false);
  }
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::CudaTrafficLightOcclusionPredictorNode)

