// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_NODE_HPP_
#define AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_NODE_HPP_

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"

#include <autoware/tensorrt_yolox/tensorrt_yolox.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/semantic.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "NvJpegDecoder.h"
#include <cudaEGL.h>
#include "nvbufsurface.h"

namespace autoware::tensorrt_yolox
{
// cspell: ignore Semseg
using LabelMap = std::map<int, std::string>;
using Label = tier4_perception_msgs::msg::Semantic;
class TrtYoloXNode : public rclcpp::Node
{
  struct RoiOverlaySemsegLabel
  {
    bool UNKNOWN;
    bool CAR;
    bool TRUCK;
    bool BUS;
    bool MOTORCYCLE;
    bool BICYCLE;
    bool PEDESTRIAN;
    bool ANIMAL;
    bool isOverlay(const uint8_t label) const
    {
      return (label == Label::UNKNOWN && UNKNOWN) || (label == Label::CAR && CAR) ||
             (label == Label::TRUCK && TRUCK) || (label == Label::BUS && BUS) ||
             (label == Label::ANIMAL && ANIMAL) || (label == Label::MOTORBIKE && MOTORCYCLE) ||
             (label == Label::BICYCLE && BICYCLE) || (label == Label::PEDESTRIAN && PEDESTRIAN);
    };
  };  // struct RoiOverlaySemsegLabel

public:
  explicit TrtYoloXNode(const rclcpp::NodeOptions & node_options);

  virtual ~TrtYoloXNode() {
    freeGpuBuffers();
  }

private:
  // void onConnect();
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  void onCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr input_compressed_image_msg);
  bool decodeToGpu(const std::vector<uint8_t>& compressed_data, uint8_t*& d_bgr_ptr, uint32_t& width, uint32_t& height);
  std::unique_ptr<NvJPEGDecoder> decoder_;
  NvBufSurface* surf_rgba_ = nullptr;
  uint8_t* d_bgr_ = nullptr;
  size_t d_bgr_size_ = 0;

  // Helper to clean up on node destruction
  void freeGpuBuffers() {
    if (surf_rgba_) {
      // Unmap if mapped, then destroy
      NvBufSurfaceUnMap(surf_rgba_, 0, 0); 
      NvBufSurfaceDestroy(surf_rgba_);
      surf_rgba_ = nullptr;
    }
    if (d_bgr_) {
      cudaFree(d_bgr_);
      d_bgr_ = nullptr;
    }
    d_bgr_size_ = 0;
  }

  bool initEGL() {
    // 1. Get the default display
    EGLDisplay egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (egl_display == EGL_NO_DISPLAY) {
        printf("Failed to get EGL Display\n");
        return false;
    }

    // 2. Initialize it
    if (!eglInitialize(egl_display, NULL, NULL)) {
        printf("Failed to initialize EGL\n");
        return false;
    }
    return true;
}

  bool readLabelFile(const std::string & label_path);
  void replaceLabelMap();
  void overlapSegmentByRoi(
    const tensorrt_yolox::Object & object, cv::Mat & mask, const int width, const int height);
  int mapRoiLabel2SegLabel(const int32_t roi_label_index);
  image_transport::Publisher image_pub_;
  image_transport::Publisher mask_pub_;

  image_transport::Publisher color_mask_pub_;

  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr objects_pub_;

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;

  // rclcpp::TimerBase::SharedPtr timer_;

  LabelMap label_map_;
  std::unique_ptr<tensorrt_yolox::TrtYoloX> trt_yolox_;
  bool is_roi_overlap_segment_;
  bool is_publish_color_mask_;
  float overlap_roi_score_threshold_;
  // TODO(badai-nguyen): change to function
  std::map<std::string, int> remap_roi_to_semantic_ = {
    {"UNKNOWN", 3},     // other
    {"ANIMAL", 0},      // other
    {"PEDESTRIAN", 6},  // person
    {"CAR", 7},         // car
    {"TRUCK", 7},       // truck
    {"BUS", 7},         // bus
    {"BICYCLE", 8},     // bicycle
    {"MOTORBIKE", 8},   // motorcycle
  };
  RoiOverlaySemsegLabel roi_overlay_segment_labels_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;
};

}  // namespace autoware::tensorrt_yolox

#endif  // AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_NODE_HPP_
