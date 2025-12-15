// Copyright 2020 Tier IV, Inc.
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

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "autoware/image_transport_decompressor/image_transport_decompressor.hpp"

#include "NvJpegDecoder.h"
#include "nvbufsurface.h"
#include "nvbufsurftransform.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::image_preprocessor
{
ImageTransportDecompressor::ImageTransportDecompressor(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("image_transport_decompressor", node_options),
  encoding_(declare_parameter<std::string>("encoding"))
{
  jpeg_decoder_.reset(NvJPEGDecoder::createJPEGDecoder("jpegdec"));

  raw_image_pub_ =
    create_publisher<sensor_msgs::msg::Image>("~/output/raw_image", rclcpp::SensorDataQoS());

  compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    "~/input/compressed_image", rclcpp::SensorDataQoS(),
    std::bind(&ImageTransportDecompressor::onCompressedImage, this, std::placeholders::_1));
}

// void ImageTransportDecompressor::onCompressedImage(
//   const sensor_msgs::msg::CompressedImage::ConstSharedPtr input_compressed_image_msg)
// {
//   if (input_compressed_image_msg->data.empty()) {
//     return;
//   }

//   try {
//     // -------------------------------------------------------
//     // 1. Setup Formats (JetPack 6 NvBufSurface)
//     // -------------------------------------------------------
//     // JetPack 6 VIC hardware prefers 32-bit formats (RGBA/BGRA)
//     NvBufSurfaceColorFormat target_hw_format = NVBUF_COLOR_FORMAT_RGBA;
//     std::string target_ros_encoding = "bgr8";

//     if (encoding_ == "rgb8" || input_compressed_image_msg->format.find("rgb8") == 0) {
//       // If we want RGB, we often still use RGBA in hardware and strip alpha later
//       target_hw_format = NVBUF_COLOR_FORMAT_RGBA;
//       target_ros_encoding = "rgb8";
//     }

//     std::cout << target_ros_encoding << std::endl;

//     // -------------------------------------------------------
//     // 2. Decode to Hardware Buffer (fd_src)
//     // -------------------------------------------------------
//     uint32_t width, height, pixfmt;
//     int fd_src = -1;
//     unsigned char * bitstream = const_cast<unsigned char*>(input_compressed_image_msg->data.data());
//     size_t bitstream_len = input_compressed_image_msg->data.size();

//     if (jpeg_decoder_->decodeToFd(fd_src, bitstream, bitstream_len, pixfmt, width, height) < 0) {
//       RCLCPP_ERROR(get_logger(), "NvJPEGDecoder::decodeToFd failed");
//       return;
//     }

//     // -------------------------------------------------------
//     // 3. Allocate Destination Buffer (surf_dst)
//     // -------------------------------------------------------
//     NvBufSurface * surf_dst = NULL;
//     NvBufSurfaceAllocateParams alloc_params = {0};
    
//     alloc_params.params.width = width;
//     alloc_params.params.height = height;
//     alloc_params.params.layout = NVBUF_LAYOUT_PITCH;
//     alloc_params.params.colorFormat = target_hw_format;
//     alloc_params.params.memType = NVBUF_MEM_SURFACE_ARRAY;
//     alloc_params.memtag = NvBufSurfaceTag_VIDEO_CONVERT;

//     if (NvBufSurfaceAllocate(&surf_dst, 1, &alloc_params) != 0) {
//       RCLCPP_ERROR(get_logger(), "NvBufSurfaceAllocate failed");
//       return;
//     }

//     // -------------------------------------------------------
//     // 4. Wrap Source FD into Surface (surf_src)
//     // -------------------------------------------------------
//     NvBufSurface * surf_src = NULL;
//     if (NvBufSurfaceFromFd(fd_src, (void**)&surf_src) != 0) {
//       RCLCPP_ERROR(get_logger(), "NvBufSurfaceFromFd failed");
//       NvBufSurfaceDestroy(surf_dst);
//       return;
//     }

//     // -------------------------------------------------------
//     // 5. Color Conversion (Transform)
//     // -------------------------------------------------------
//     NvBufSurfTransformParams transform_params = {0};
//     transform_params.transform_flag = NVBUFSURF_TRANSFORM_FILTER;
//     transform_params.transform_filter = NvBufSurfTransformInter_Algo3; // Smart/High quality

//     // Perform conversion: YUV (src) -> RGBA (dst)
//     if (NvBufSurfTransform(surf_src, surf_dst, &transform_params) != 0) {
//       RCLCPP_ERROR(get_logger(), "NvBufSurfTransform failed");
//       NvBufSurfaceDestroy(surf_dst);
//       // surf_src is a wrapper around fd_src, usually doesn't need explicit destroy if imported
//       return;
//     }

//     // -------------------------------------------------------
//     // 6. Map to CPU and Sync
//     // -------------------------------------------------------
//     if (NvBufSurfaceMap(surf_dst, 0, -1, NVBUF_MAP_READ) != 0) {
//       RCLCPP_ERROR(get_logger(), "NvBufSurfaceMap failed");
//       NvBufSurfaceDestroy(surf_dst);
//       return;
//     }
    
//     // Sync cache for CPU reading
//     if (NvBufSurfaceSyncForCpu(surf_dst, 0, -1) != 0) {
//       RCLCPP_ERROR(get_logger(), "NvBufSurfaceSyncForCpu failed");
//       NvBufSurfaceUnMap(surf_dst, 0, -1);
//       NvBufSurfaceDestroy(surf_dst);
//       return;
//     }

//     // -------------------------------------------------------
//     // 7. Copy to ROS Message (4-channel -> 3-channel)
//     // -------------------------------------------------------
//     auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
//     image_msg->header = input_compressed_image_msg->header;
//     image_msg->height = height;
//     image_msg->width = width;
//     image_msg->encoding = target_ros_encoding;
//     image_msg->step = width * 3;
//     image_msg->data.resize(image_msg->step * height);

//     // Access the mapped data (Plane 0)
//     uint8_t * src_ptr = (uint8_t *)surf_dst->surfaceList[0].mappedAddr.addr[0];
//     uint32_t src_pitch = surf_dst->surfaceList[0].planeParams.pitch[0];
//     uint8_t * dst_ptr = image_msg->data.data();

//     // Loop to copy and strip Alpha channel (RGBA -> RGB / BGRA -> BGR)
//     // Assuming RGBA hardware format corresponds to [R, G, B, A] bytes
//     for (uint32_t y = 0; y < height; ++y) {
//       uint8_t * row_src = src_ptr + (y * src_pitch);
//       uint8_t * row_dst = dst_ptr + (y * image_msg->step);

//       for (uint32_t x = 0; x < width; ++x) {
//         // Copy R, G, B (or B, G, R depending on encoding)
//         row_dst[0] = row_src[0];
//         row_dst[1] = row_src[1];
//         row_dst[2] = row_src[2];

//         // If target was BGR8 but HW is RGBA, we might need to swap 0 and 2.
//         // However, NvBufSurfTransform usually handles color space correctly if configured.
//         // If colors look swapped (blue face), swap row_src[0] and row_src[2] here.
//         if (target_ros_encoding == "bgr8" && target_hw_format == NVBUF_COLOR_FORMAT_RGBA) {
//              std::swap(row_dst[0], row_dst[2]); 
//         }

//         row_src += 4; // Skip 4 bytes (RGBA)
//         row_dst += 3; // Advance 3 bytes (RGB)
//       }
//     }

//     // -------------------------------------------------------
//     // 8. Cleanup
//     // -------------------------------------------------------
//     NvBufSurfaceUnMap(surf_dst, 0, -1);
//     NvBufSurfaceDestroy(surf_dst);
    
//     // Note: fd_src is managed by NvJpegDecoder.
//     // In JP6 samples, the decoder reuse strategy suggests not closing it manually per frame 
//     // if using the same decoder instance, or the decoder handles it.
    
//     raw_image_pub_->publish(std::move(image_msg));

//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(get_logger(), "Exception: %s", e.what());
//   }
// }

void ImageTransportDecompressor::onCompressedImage(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr input_compressed_image_msg)
{
  auto start = get_clock()->now();

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  // Copy message header
  cv_ptr->header = input_compressed_image_msg->header;

  // Decode color/mono image
  try {
    cv_ptr->image = cv::imdecode(cv::Mat(input_compressed_image_msg->data), cv::IMREAD_COLOR);

    // Assign image encoding string
    const size_t split_pos = input_compressed_image_msg->format.find(';');
    if (split_pos == std::string::npos) {
      // Older version of compressed_image_transport does not signal image format
      switch (cv_ptr->image.channels()) {
        case 1:
          cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
          break;
        case 3:
          cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
          break;
        default:
          RCLCPP_ERROR(
            get_logger(), "Unsupported number of channels: %i", cv_ptr->image.channels());
          break;
      }
    } else {
      std::string image_encoding;
      if (encoding_ == std::string("rgb8")) {
        image_encoding = "rgb8";
      } else if (encoding_ == std::string("bgr8")) {
        image_encoding = "bgr8";
      } else {
        // default encoding
        image_encoding = input_compressed_image_msg->format.substr(0, split_pos);
      }

      cv_ptr->encoding = image_encoding;

      if (sensor_msgs::image_encodings::isColor(image_encoding)) {
        std::string compressed_encoding = input_compressed_image_msg->format.substr(split_pos);
        bool compressed_bgr_image =
          (compressed_encoding.find("compressed bgr") != std::string::npos);

        // Revert color transformation
        if (compressed_bgr_image) {
          // if necessary convert colors from bgr to rgb
          if (
            (image_encoding == sensor_msgs::image_encodings::RGB8) ||
            (image_encoding == sensor_msgs::image_encodings::RGB16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::RGBA8) ||
            (image_encoding == sensor_msgs::image_encodings::RGBA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::BGRA8) ||
            (image_encoding == sensor_msgs::image_encodings::BGRA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
          }
        } else {
          // if necessary convert colors from rgb to bgr
          if (
            (image_encoding == sensor_msgs::image_encodings::BGR8) ||
            (image_encoding == sensor_msgs::image_encodings::BGR16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::BGRA8) ||
            (image_encoding == sensor_msgs::image_encodings::BGRA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::RGBA8) ||
            (image_encoding == sensor_msgs::image_encodings::RGBA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
          }
        }
      }
    }
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0)) {
    // Publish message to user callback
    auto image_ptr = std::make_unique<sensor_msgs::msg::Image>(*cv_ptr->toImageMsg());
    raw_image_pub_->publish(std::move(image_ptr));
  }

  auto end = get_clock()->now();
  std::cout << "decompress processing_time_ms=" << (end - start).seconds() * 1000 << "[ms]" << std::endl;
}
}  // namespace autoware::image_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_preprocessor::ImageTransportDecompressor)
