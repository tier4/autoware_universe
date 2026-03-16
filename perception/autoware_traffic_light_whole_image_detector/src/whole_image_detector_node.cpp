// Copyright 2024 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include "autoware_traffic_light_whole_image_detector/lightnet_decoder.hpp"
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
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
    // remove default set values in declare_parameter to avoid confusion
    std::string onnx_path = this->declare_parameter<std::string>("onnx_path");
    std::string names_path = this->declare_parameter<std::string>("names_file");
    std::string precision = this->declare_parameter<std::string>("precision");
    std::string calibration_images =  this->declare_parameter<std::string>("calibration_images");
    std::string calib = this->declare_parameter<std::string>("calib");
    // bool sparse = this->declare_parameter<bool>("sparse");
    bool build_only = this->declare_parameter<bool>("build_only");
    // bool use_cuda = this->declare_parameter<bool>("use_cuda");
    std::string traffic_light_class_name = this->declare_parameter<std::string>("traffic_light_class_name");
    float score_thresh = static_cast<float>(this->declare_parameter<double>("score_thresh"));
    float nms_thresh = static_cast<float>(this->declare_parameter<double>("nms_thresh"));
    int num_anchors = this->declare_parameter<int>("num_anchors");
    int num_classes = this->declare_parameter<int>("num_classes");
    std::vector<int64_t> anchors_param = this->declare_parameter<std::vector<int64_t>>("anchors");

    names_ = whole_image_detector::loadNames(names_path);
    if (names_.empty()) {
      RCLCPP_WARN(
        get_logger(), "No class names loaded from %s, using default TRAFFIC_LIGHT",
        names_path.c_str());
      names_.push_back("TRAFFIC_LIGHT");
    }

    traffic_light_class_name_ = traffic_light_class_name;
    traffic_light_class_id_ = -1;
    for (size_t i = 0; i < names_.size(); ++i) {
      if (names_[i] == traffic_light_class_name_) {
        traffic_light_class_id_ = static_cast<int>(i);
        break;
      }
    }
    RCLCPP_INFO(
      get_logger(), "Loaded %zu class names from %s", names_.size(), names_path.c_str());
    // log names_:
    for (size_t i = 0; i < names_.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  class_id=%zu: %s", i, names_[i].c_str());
    }
    if (traffic_light_class_id_ < 0 && !traffic_light_class_name_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "traffic_light_class_name '%s' not in names; publishing all detections",
        traffic_light_class_name_.c_str());
    }

    anchors_.assign(anchors_param.begin(), anchors_param.end());
    num_anchors_ = num_anchors;
    num_classes_ = num_classes;
    score_thresh_ = static_cast<double>(score_thresh);
    nms_thresh_ = static_cast<double>(nms_thresh);

#ifdef ENABLE_GPU
    try {
      detector_ = std::make_unique<whole_image_detector::ComlopsDetector>(
        this, onnx_path, precision, anchors_, num_anchors_, num_classes_, score_thresh_,
        nms_thresh_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "ComlopsDetector init failed: %s", e.what());
      detector_.reset();
    }
    if (build_only) {
      RCLCPP_INFO(get_logger(), "TensorRT engine built; exiting (build_only=true).");
      rclcpp::shutdown();
      return;
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
    objects_pub_ = create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      "~/output/rois", 1);
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/output/debug/image", 1);
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
#ifdef ENABLE_GPU
    if (stop_watch_ptr_) {
      stop_watch_ptr_->toc("processing_time", true);
    }
    if (!detector_) {
      tier4_perception_msgs::msg::DetectedObjectsWithFeature out_msg;
      out_msg.header = msg->header;
      objects_pub_->publish(out_msg);
      return;
    }
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    std::vector<whole_image_detector::BBoxInfo> all_boxes;
    if (!detector_->detect(img, all_boxes)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "ComlopsDetector::detect failed");
      tier4_perception_msgs::msg::DetectedObjectsWithFeature out_msg;
      out_msg.header = msg->header;
      objects_pub_->publish(out_msg);
      return;
    }
    // Filter to traffic_light class if specified
    std::vector<whole_image_detector::BBoxInfo> filtered;
    for (const auto & b : all_boxes) {
      if (traffic_light_class_id_ >= 0 && b.class_id != traffic_light_class_id_) continue;
      filtered.push_back(b);
    }
    tier4_perception_msgs::msg::DetectedObjectsWithFeature out_msg;
    out_msg.header = msg->header;
    for (size_t i = 0; i < filtered.size(); ++i) {
      const auto & b = filtered[i];
      tier4_perception_msgs::msg::DetectedObjectWithFeature obj;
      obj.object.existence_probability = static_cast<float>(b.prob);
      autoware_perception_msgs::msg::ObjectClassification cl;
      cl.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      cl.probability = static_cast<float>(b.prob);
      obj.object.classification.push_back(cl);
      obj.feature.roi.x_offset = static_cast<uint32_t>(std::max(0, static_cast<int>(b.box.x1)));
      obj.feature.roi.y_offset = static_cast<uint32_t>(std::max(0, static_cast<int>(b.box.y1)));
      obj.feature.roi.width =
        static_cast<uint32_t>(std::max(0.f, b.box.x2 - b.box.x1));
      obj.feature.roi.height =
        static_cast<uint32_t>(std::max(0.f, b.box.y2 - b.box.y1));
      out_msg.feature_objects.push_back(obj);
    }
    objects_pub_->publish(out_msg);

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
    tier4_perception_msgs::msg::DetectedObjectsWithFeature out_msg;
    out_msg.header = msg->header;
    objects_pub_->publish(out_msg);
#endif
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    objects_pub_;
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
