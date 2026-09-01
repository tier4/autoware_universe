// Copyright 2026 TIER IV, Inc.
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

#include "autoware/tensorrt_e2e/providers/lidar_input_provider.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace
{
constexpr int64_t LOG_THROTTLE_INTERVAL_MS = 5000;

/// Find a field by name and return its offset, or -1 when absent.
int64_t find_field_offset(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name, uint8_t & datatype)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      datatype = field.datatype;
      return field.offset;
    }
  }
  return -1;
}

float read_field_as_float(const uint8_t * point, const int64_t offset, const uint8_t datatype)
{
  using sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::FLOAT32:
      return *reinterpret_cast<const float *>(point + offset);
    case PointField::FLOAT64:
      return static_cast<float>(*reinterpret_cast<const double *>(point + offset));
    case PointField::UINT8:
      return static_cast<float>(*(point + offset));
    case PointField::INT8:
      return static_cast<float>(*reinterpret_cast<const int8_t *>(point + offset));
    case PointField::UINT16:
      return static_cast<float>(*reinterpret_cast<const uint16_t *>(point + offset));
    case PointField::INT16:
      return static_cast<float>(*reinterpret_cast<const int16_t *>(point + offset));
    case PointField::UINT32:
      return static_cast<float>(*reinterpret_cast<const uint32_t *>(point + offset));
    case PointField::INT32:
      return static_cast<float>(*reinterpret_cast<const int32_t *>(point + offset));
    default:
      return 0.0f;
  }
}

}  // namespace

LidarInputProvider::LidarInputProvider(rclcpp::Node & node) : node_(node)
{
  points_tensor_name_ = node_.declare_parameter<std::string>("lidar.points_tensor", "points");
  num_points_tensor_name_ =
    node_.declare_parameter<std::string>("lidar.num_points_tensor", "num_points");
  max_delay_ms_ = node_.declare_parameter<double>("lidar.max_delay_ms", 200.0);
  intensity_scale_ = node_.declare_parameter<double>("lidar.intensity_scale", 1.0);
}

std::vector<std::string> LidarInputProvider::claim_inputs(
  const std::vector<TensorSpec> & engine_inputs)
{
  std::vector<std::string> claimed;

  const TensorSpec * points_spec = find_spec(engine_inputs, points_tensor_name_);
  if (!points_spec) {
    throw std::runtime_error(
      "The lidar input provider is enabled but the model has no input tensor named '" +
      points_tensor_name_ + "' (set lidar.points_tensor to match the model)");
  }

  // Accept [1, P, D] or [P, D].
  const auto & shape = points_spec->shape;
  const bool has_batch = shape.size() == 3;
  if (!(shape.size() == 2 || (has_batch && shape[0] == 1))) {
    throw std::runtime_error(
      "Model input '" + points_tensor_name_ + "' has shape " + shape_to_string(shape) +
      "; expected [1, P, D] or [P, D]");
  }
  const size_t base = has_batch ? 1 : 0;
  max_points_ = shape[base];
  point_dim_ = shape[base + 1];
  if (point_dim_ < 3 || point_dim_ > 5) {
    throw std::runtime_error(
      "Model input '" + points_tensor_name_ + "' has " + std::to_string(point_dim_) +
      " features per point; supported range is 3 (xyz) to 5 (xyz, intensity, time lag)");
  }
  points_shape_ = shape;
  claimed.push_back(points_tensor_name_);

  const TensorSpec * num_points_spec = find_spec(engine_inputs, num_points_tensor_name_);
  if (num_points_spec) {
    if (num_points_spec->num_elements() != 1) {
      throw std::runtime_error(
        "Model input '" + num_points_tensor_name_ + "' has shape " +
        shape_to_string(num_points_spec->shape) + "; expected a single element");
    }
    num_points_shape_ = num_points_spec->shape;
    num_points_claimed_ = true;
    claimed.push_back(num_points_tensor_name_);
  }

  pointcloud_sub_ = node_.create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{},
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_pointcloud_ = msg;
    });

  return claimed;
}

bool LidarInputProvider::collect(
  [[maybe_unused]] const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs,
  std::string & error)
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud = latest_pointcloud_;
  }
  if (!cloud) {
    error = "No point cloud received yet";
    return false;
  }

  const double delay_ms = (now - rclcpp::Time(cloud->header.stamp)).seconds() * 1e3;
  if (delay_ms > max_delay_ms_) {
    error = "Point cloud is stale (" + std::to_string(delay_ms) + " ms > " +
            std::to_string(max_delay_ms_) + " ms)";
    return false;
  }

  uint8_t x_type{}, y_type{}, z_type{}, intensity_type{};
  const int64_t x_offset = find_field_offset(*cloud, "x", x_type);
  const int64_t y_offset = find_field_offset(*cloud, "y", y_type);
  const int64_t z_offset = find_field_offset(*cloud, "z", z_type);
  const int64_t intensity_offset = find_field_offset(*cloud, "intensity", intensity_type);
  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    error = "Point cloud is missing x/y/z fields";
    return false;
  }
  if (point_dim_ >= 4 && intensity_offset < 0) {
    error = "The model expects intensity, but the point cloud has no 'intensity' field";
    return false;
  }

  const int64_t total_points =
    static_cast<int64_t>(cloud->width) * static_cast<int64_t>(cloud->height);
  const int64_t used_points = std::min(total_points, max_points_);
  if (total_points > max_points_) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Point cloud has %ld points; truncating to the model capacity of %ld", total_points,
      max_points_);
  }

  std::vector<float> data(static_cast<size_t>(max_points_) * point_dim_, 0.0f);
  const uint8_t * cloud_data = cloud->data.data();
  const size_t point_step = cloud->point_step;
  for (int64_t i = 0; i < used_points; ++i) {
    const uint8_t * point = cloud_data + static_cast<size_t>(i) * point_step;
    float * dst = data.data() + static_cast<size_t>(i) * point_dim_;
    dst[0] = read_field_as_float(point, x_offset, x_type);
    dst[1] = read_field_as_float(point, y_offset, y_type);
    dst[2] = read_field_as_float(point, z_offset, z_type);
    if (point_dim_ >= 4) {
      dst[3] = read_field_as_float(point, intensity_offset, intensity_type) *
               static_cast<float>(intensity_scale_);
    }
    // dim 5 (time lag) stays 0: the concatenated cloud is motion-compensated upstream.
  }

  inputs[points_tensor_name_] = Tensor::from_host(points_shape_, std::move(data));
  if (num_points_claimed_) {
    inputs[num_points_tensor_name_] =
      Tensor::from_host(num_points_shape_, {static_cast<float>(used_points)});
  }
  return true;
}

}  // namespace autoware::tensorrt_e2e
