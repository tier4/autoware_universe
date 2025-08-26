#include "autoware/tensorrt_vad/input_converter/image_converter.hpp"
#include <opencv2/opencv.hpp>
#include <stdexcept>

namespace autoware::tensorrt_vad::vad_interface {

InputImageConverter::InputImageConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config)
  : Converter(coordinate_transformer, config)
{
}

std::vector<float> InputImageConverter::normalize_image(
  unsigned char* image_data, int32_t width, int32_t height) const
{
  std::vector<float> normalized_image_data(width * height * 3);
  
  // Process in BGR order
  for (int32_t c = 0; c < 3; ++c) {
    for (int32_t h = 0; h < height; ++h) {
      for (int32_t w = 0; w < width; ++w) {
        int32_t src_idx = (h * width + w) * 3 + (2 - c); // BGR -> RGB
        int32_t dst_idx = c * height * width + h * width + w; // CHW format
        float pixel_value = static_cast<float>(image_data[src_idx]);
        normalized_image_data[dst_idx] = (pixel_value - config_.image_normalization_param_mean[c]) / 
                                        config_.image_normalization_param_std[c];
      }
    }
  }
  
  return normalized_image_data;
}

CameraImagesData InputImageConverter::process_image(
  const std::vector<sensor_msgs::msg::Image::ConstSharedPtr>& images) const
{
  std::vector<std::vector<float>> frame_images;
  frame_images.resize(6); // Initialize in VAD camera order

  // Process each camera image
  for (int32_t autoware_idx = 0; autoware_idx < 6; ++autoware_idx) {
    const auto& image_msg = images[autoware_idx];

    // Create cv::Mat from sensor_msgs::msg::Image
    cv::Mat bgr_img;
    if (image_msg->encoding == "bgr8") {
      // For BGR8, use data directly
      bgr_img = cv::Mat(image_msg->height, image_msg->width, CV_8UC3, 
                        const_cast<uint8_t*>(image_msg->data.data()), image_msg->step);
    } else {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("autoware_tensorrt_vad"), *rclcpp::Clock::make_shared(), 5000, "Unsupported image encoding: %s", image_msg->encoding.c_str());
      continue;
    }

    if (bgr_img.empty()) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("autoware_tensorrt_vad"), *rclcpp::Clock::make_shared(), 5000, "Failed to decode image data: %d", autoware_idx);
      continue;
    }

    // Convert to RGB
    cv::Mat rgb_img;
    cv::cvtColor(bgr_img, rgb_img, cv::COLOR_BGR2RGB);

    // Resize if size differs from target
    if (rgb_img.cols != config_.target_image_width || rgb_img.rows != config_.target_image_height) {
      cv::resize(rgb_img, rgb_img, cv::Size(config_.target_image_width, config_.target_image_height));
    }

    // Normalize image
    std::vector<float> normalized_image_data = normalize_image(rgb_img.data, rgb_img.cols, rgb_img.rows);

    // Store in VAD camera order
    int32_t vad_idx = config_.autoware_to_vad_camera_mapping.at(autoware_idx);
    frame_images[vad_idx] = normalized_image_data;
  }

  // Concatenate image data
  std::vector<float> concatenated_data;
  size_t single_camera_size = 3 * config_.target_image_height * config_.target_image_width;
  concatenated_data.reserve(single_camera_size * 6);

  // Camera order: {0, 1, 2, 3, 4, 5}
  for (int32_t camera_idx = 0; camera_idx < 6; ++camera_idx) {
    const auto& img_data = frame_images[camera_idx];
    if (img_data.size() != single_camera_size) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("autoware_tensorrt_vad"), *rclcpp::Clock::make_shared(), 5000, "Invalid image size: %d", camera_idx);
      continue;
    }
    concatenated_data.insert(concatenated_data.end(), img_data.begin(), img_data.end());
  }

  return concatenated_data;
}

} // namespace autoware::tensorrt_vad::vad_interface
