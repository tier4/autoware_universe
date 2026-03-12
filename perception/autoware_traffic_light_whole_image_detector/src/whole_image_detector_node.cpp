// Copyright 2024 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include "autoware_traffic_light_whole_image_detector/lightnet_decoder.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#ifdef ENABLE_GPU
#include "autoware_traffic_light_whole_image_detector/comlops_detector.hpp"
#endif

namespace autoware::traffic_light
{

class WholeImageDetectorNode : public rclcpp::Node
{
public:
  explicit WholeImageDetectorNode(const rclcpp::NodeOptions & options)
  : Node("traffic_light_whole_image_detector_node", options)
  {
    declare_parameter<std::string>("onnx_path", "models/CoMLOps-Large-Detection-Model-v1.0.1.onnx");
    declare_parameter<std::string>("names_file", "data/T4x.names");
    declare_parameter<std::string>("precision", "fp16");
    declare_parameter<std::string>("calibration_images", "");
    declare_parameter<std::string>("calib", "Entropy");
    declare_parameter<bool>("sparse", true);
    declare_parameter<std::vector<int64_t>>("anchors",
      std::vector<int64_t>{8, 9, 36, 11, 15, 28, 40, 36, 29, 72, 104, 24, 86, 67, 58, 140, 265, 70, 147, 134, 164, 303, 337, 206, 491, 400, 664, 705, 1269, 832});
    declare_parameter<int>("num_anchors", 5);
    declare_parameter<int>("num_classes", 10);
    declare_parameter<double>("score_thresh", 0.2);
    declare_parameter<double>("nms_thresh", 0.6);
    declare_parameter<bool>("use_cuda", true);
    declare_parameter<std::string>("traffic_light_class_name", "TRAFFIC_LIGHT");

    std::string package_share =
      ament_index_cpp::get_package_share_directory("autoware_traffic_light_whole_image_detector");
    std::string onnx_path = resolvePath(get_parameter("onnx_path").as_string(), package_share);
    std::string names_path = resolvePath(get_parameter("names_file").as_string(), package_share);

    names_ = whole_image_detector::loadNames(names_path);
    if (names_.empty()) {
      RCLCPP_WARN(
        get_logger(), "No class names loaded from %s, using default TRAFFIC_LIGHT",
        names_path.c_str());
      names_.push_back("TRAFFIC_LIGHT");
    }

    traffic_light_class_name_ = get_parameter("traffic_light_class_name").as_string();
    traffic_light_class_id_ = -1;
    for (size_t i = 0; i < names_.size(); ++i) {
      if (names_[i] == traffic_light_class_name_) {
        traffic_light_class_id_ = static_cast<int>(i);
        break;
      }
    }
    if (traffic_light_class_id_ < 0 && !traffic_light_class_name_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "traffic_light_class_name '%s' not in names; publishing all detections",
        traffic_light_class_name_.c_str());
    }

    std::vector<int64_t> anchors_param = get_parameter("anchors").as_integer_array();
    anchors_.assign(anchors_param.begin(), anchors_param.end());
    num_anchors_ = get_parameter("num_anchors").as_int();
    num_classes_ = get_parameter("num_classes").as_int();
    score_thresh_ = get_parameter("score_thresh").as_double();
    nms_thresh_ = get_parameter("nms_thresh").as_double();

#ifdef ENABLE_GPU
    try {
      detector_ = std::make_unique<whole_image_detector::ComlopsDetector>(
        this, onnx_path, get_parameter("precision").as_string(), anchors_, num_anchors_,
        num_classes_, score_thresh_, nms_thresh_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "ComlopsDetector init failed: %s", e.what());
      detector_.reset();
    }
#else
    (void)onnx_path;
#endif

#ifdef ENABLE_GPU
    stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
#endif

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "~/input/image", rclcpp::SensorDataQoS(),
      std::bind(&WholeImageDetectorNode::onImage, this, std::placeholders::_1));
    roi_pub_ =
      create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>("~/output/rois", 1);
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/output/debug/image", 1);
  }

private:
  static std::string resolvePath(const std::string & path, const std::string & package_share_dir)
  {
    if (path.empty()) return path;
    if (path[0] == '/') return path;
    return package_share_dir + "/" + path;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
#ifdef ENABLE_GPU
    if (stop_watch_ptr_) {
      stop_watch_ptr_->toc("processing_time", true);
    }
    if (!detector_) {
      tier4_perception_msgs::msg::TrafficLightRoiArray out_msg;
      out_msg.header = msg->header;
      roi_pub_->publish(out_msg);
      return;
    }
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    std::vector<whole_image_detector::BBoxInfo> all_boxes;
    if (!detector_->detect(img, all_boxes)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "ComlopsDetector::detect failed");
      tier4_perception_msgs::msg::TrafficLightRoiArray out_msg;
      out_msg.header = msg->header;
      roi_pub_->publish(out_msg);
      return;
    }
    // Filter to traffic_light class if specified
    std::vector<whole_image_detector::BBoxInfo> filtered;
    for (const auto & b : all_boxes) {
      if (traffic_light_class_id_ >= 0 && b.class_id != traffic_light_class_id_) continue;
      filtered.push_back(b);
    }
    tier4_perception_msgs::msg::TrafficLightRoiArray out_msg;
    out_msg.header = msg->header;
    for (size_t i = 0; i < filtered.size(); ++i) {
      const auto & b = filtered[i];
      tier4_perception_msgs::msg::TrafficLightRoi roi;
      roi.roi.x_offset = static_cast<uint32_t>(std::max(0.f, b.box.x1));
      roi.roi.y_offset = static_cast<uint32_t>(std::max(0.f, b.box.y1));
      roi.roi.width = static_cast<uint32_t>(std::max(0.f, b.box.x2 - b.box.x1));
      roi.roi.height = static_cast<uint32_t>(std::max(0.f, b.box.y2 - b.box.y1));
      roi.traffic_light_id = static_cast<int64_t>(i);
      roi.traffic_light_type = tier4_perception_msgs::msg::TrafficLightRoi::CAR_TRAFFIC_LIGHT;
      out_msg.rois.push_back(roi);
    }
    roi_pub_->publish(out_msg);

    if (debug_publisher_) {
      const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
      const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
      const double pipeline_latency_ms =
        std::chrono::duration<double, std::milli>(
          std::chrono::nanoseconds(
            (get_clock()->now() - out_msg.header.stamp).nanoseconds()))
          .count();
      debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "debug/cyclic_time_ms", cyclic_time_ms);
      debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "debug/processing_time_ms", processing_time_ms);
      debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "debug/pipeline_latency_ms", pipeline_latency_ms);
    }

    // Publish debug image with ROI overlay when someone is subscribed
    if (debug_image_pub_->get_subscription_count() > 0) {
      cv::Mat debug_img = img.clone();
      const cv::Scalar color_roi(0, 255, 0);  // BGR green
      const int thickness = 2;
      for (size_t i = 0; i < filtered.size(); ++i) {
        const auto & b = filtered[i];
        const int x1 = static_cast<int>(std::max(0.f, b.box.x1));
        const int y1 = static_cast<int>(std::max(0.f, b.box.y1));
        const int x2 = static_cast<int>(std::max(0.f, b.box.x2));
        const int y2 = static_cast<int>(std::max(0.f, b.box.y2));
        cv::rectangle(debug_img, cv::Point(x1, y1), cv::Point(x2, y2), color_roi, thickness);
        const std::string label = "TL#" + std::to_string(i) + " " + std::to_string(static_cast<int>(b.prob * 100)) + "%";
        int baseline = 0;
        cv::Size text_sz = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::putText(
          debug_img, label, cv::Point(x1, std::max(text_sz.height, y1 - 2)),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, color_roi, 1, cv::LINE_AA);
      }
      const auto debug_msg =
        cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, debug_img).toImageMsg();
      debug_image_pub_->publish(*debug_msg);
    }
#else
    (void)msg;
    tier4_perception_msgs::msg::TrafficLightRoiArray out_msg;
    out_msg.header = msg->header;
    roi_pub_->publish(out_msg);
#endif
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  std::vector<std::string> names_;
  std::string traffic_light_class_name_;
  int traffic_light_class_id_;
  std::vector<int> anchors_;
  int num_anchors_;
  int num_classes_;
  double score_thresh_;
  double nms_thresh_;

#ifdef ENABLE_GPU
  std::unique_ptr<whole_image_detector::ComlopsDetector> detector_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;
#endif
};

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::WholeImageDetectorNode)
