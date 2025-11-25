// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_goal_planner_module/util.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <matplotlibcpp17/pyplot.h>
#include <pybind11/pytypes.h>

#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using autoware_internal_planning_msgs::msg::PathWithLaneId;
using visualization_msgs::msg::MarkerArray;
using autoware::behavior_path_planner::PathPointWithLaneId;
using pybind11::literals::operator""_a;

// Default parameters (can be overridden by command line arguments)
constexpr double DEFAULT_LATERAL_ACCELERATION_THRESHOLD = 0.5;  // m/s^2
constexpr double DEFAULT_LATERAL_ACCELERATION_FILTERING_DURATION = 1.0;  // s
constexpr double DEFAULT_VELOCITY = 1.389;  // m/s (5 km/h)
constexpr int64_t DEFAULT_TARGET_TIMESTAMP_SEC = 1763703128;
constexpr int64_t DEFAULT_TARGET_TIMESTAMP_NANOSEC = 444000000;

// Helper function to plot path with arrows
void plot_path(
  matplotlibcpp17::axes::Axes & axes, const std::vector<PathPointWithLaneId> & path_points,
  const std::string & color = "blue", const std::string & label = "")
{
  std::vector<double> xs, ys;
  std::vector<double> yaw_cos, yaw_sin;

  for (const auto & point : path_points) {
    xs.push_back(point.point.pose.position.x);
    ys.push_back(point.point.pose.position.y);
    const double yaw = autoware_utils::get_rpy(point.point.pose).z;
    yaw_cos.push_back(std::cos(yaw));
    yaw_sin.push_back(std::sin(yaw));
  }

  // Plot path line
  if (label.empty()) {
    axes.plot(Args(xs, ys), Kwargs("color"_a = color, "linewidth"_a = 2.0));
  } else {
    axes.plot(Args(xs, ys), Kwargs("color"_a = color, "linewidth"_a = 2.0, "label"_a = label));
  }

  // Plot orientation arrows
  axes.quiver(
    Args(xs, ys, yaw_cos, yaw_sin),
    Kwargs("angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 3.0, "color"_a = color, "alpha"_a = 0.6));
}

// Helper function to calculate end pose at evaluation distance from start pose
geometry_msgs::msg::Pose calculateEndPose(
  const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Pose & start_pose,
  const double evaluation_distance)
{
  // Find start_pose index in the path
  size_t start_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path_points.size(); ++i) {
    const double dist = std::hypot(
      path_points[i].point.pose.position.x - start_pose.position.x,
      path_points[i].point.pose.position.y - start_pose.position.y);
    if (dist < min_dist) {
      min_dist = dist;
      start_idx = i;
    }
  }

  // Calculate end pose at evaluation_distance from start_pose
  double accumulated_distance = 0.0;
  for (size_t i = start_idx + 1; i < path_points.size(); ++i) {
    const auto & prev_point = path_points[i - 1].point.pose.position;
    const auto & curr_point = path_points[i].point.pose.position;
    accumulated_distance += std::hypot(curr_point.x - prev_point.x, curr_point.y - prev_point.y);

    if (accumulated_distance >= evaluation_distance) {
      return path_points[i].point.pose;
    }
  }

  // If evaluation_distance exceeds path length, return last point
  return path_points.back().point.pose;
}

geometry_msgs::msg::Pose extractStartPoseFromMarkerArray(const MarkerArray & marker_array)
{
  geometry_msgs::msg::Pose start_pose;

  // Find start pose marker (typically the first pose marker in goal_planner markers)
  for (const auto & marker : marker_array.markers) {
    if (marker.ns == "pull_over_start_pose" ||
        (marker.type == visualization_msgs::msg::Marker::ARROW && marker.id == 0)) {
      start_pose = marker.pose;
      std::cout << "Found start pose from MarkerArray: "
                << "x=" << start_pose.position.x
                << ", y=" << start_pose.position.y
                << ", z=" << start_pose.position.z << std::endl;
      return start_pose;
    }
  }

  std::cerr << "Warning: Could not find start pose in MarkerArray, using default" << std::endl;
  return start_pose;
}

std::vector<PathPointWithLaneId> convertToPathPointWithLaneId(const PathWithLaneId & path_msg)
{
  std::vector<PathPointWithLaneId> path_points;
  path_points.reserve(path_msg.points.size());

  for (const auto & point : path_msg.points) {
    path_points.push_back(point);
  }

  std::cout << "Converted PathWithLaneId with " << path_points.size() << " points" << std::endl;
  return path_points;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: ros2 run autoware_behavior_path_goal_planner_module "
              << "analyze_rosbag_lateral_accel <rosbag_path> [options]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "  --threshold <value>       Lateral acceleration threshold [m/s^2] (default: "
              << DEFAULT_LATERAL_ACCELERATION_THRESHOLD << ")" << std::endl;
    std::cerr << "  --duration <value>        Filtering duration [s] (default: "
              << DEFAULT_LATERAL_ACCELERATION_FILTERING_DURATION << ")" << std::endl;
    std::cerr << "  --velocity <value>        Velocity [m/s] (default: "
              << DEFAULT_VELOCITY << ")" << std::endl;
    std::cerr << "  --timestamp-sec <value>   Target timestamp seconds (default: "
              << DEFAULT_TARGET_TIMESTAMP_SEC << ")" << std::endl;
    std::cerr << "  --timestamp-nsec <value>  Target timestamp nanoseconds (default: "
              << DEFAULT_TARGET_TIMESTAMP_NANOSEC << ")" << std::endl;
    return 1;
  }

  const std::string rosbag_path = argv[1];

  // Parse command line arguments
  double lateral_acceleration_threshold = DEFAULT_LATERAL_ACCELERATION_THRESHOLD;
  double lateral_acceleration_filtering_duration = DEFAULT_LATERAL_ACCELERATION_FILTERING_DURATION;
  double velocity = DEFAULT_VELOCITY;
  int64_t target_timestamp_sec = DEFAULT_TARGET_TIMESTAMP_SEC;
  int64_t target_timestamp_nsec = DEFAULT_TARGET_TIMESTAMP_NANOSEC;

  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--threshold" && i + 1 < argc) {
      lateral_acceleration_threshold = std::stod(argv[++i]);
    } else if (arg == "--duration" && i + 1 < argc) {
      lateral_acceleration_filtering_duration = std::stod(argv[++i]);
    } else if (arg == "--velocity" && i + 1 < argc) {
      velocity = std::stod(argv[++i]);
    } else if (arg == "--timestamp-sec" && i + 1 < argc) {
      target_timestamp_sec = std::stoll(argv[++i]);
    } else if (arg == "--timestamp-nsec" && i + 1 < argc) {
      target_timestamp_nsec = std::stoll(argv[++i]);
    }
  }

  std::cout << "Opening rosbag: " << rosbag_path << std::endl;

  // Setup rosbag reader
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = rosbag_path;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception & e) {
    std::cerr << "Failed to open rosbag: " << e.what() << std::endl;
    return 1;
  }

  // Target timestamp
  const rclcpp::Time target_time(target_timestamp_sec, target_timestamp_nsec);
  std::cout << "Target timestamp: " << target_time.seconds() << std::endl;

  // Use seek to jump near target timestamp
  const int64_t target_timestamp_ns = target_timestamp_sec * 1000000000LL + target_timestamp_nsec;
  constexpr double SEEK_OFFSET = 1.0;  // Seek 1 second before target to ensure we don't miss it
  const int64_t seek_timestamp_ns = target_timestamp_ns - static_cast<int64_t>(SEEK_OFFSET * 1e9);

  std::cout << "Seeking to timestamp: " << (seek_timestamp_ns / 1e9) << std::endl;
  try {
    reader.seek(seek_timestamp_ns);
    std::cout << "Seek successful!" << std::endl;
  } catch (const std::exception & e) {
    std::cout << "Seek failed (will read from start): " << e.what() << std::endl;
  }

  // Storage for messages
  geometry_msgs::msg::Pose start_pose;
  std::vector<PathPointWithLaneId> path_points;
  bool found_marker_array = false;
  bool found_path = false;

  rclcpp::Serialization<MarkerArray> marker_serialization;
  rclcpp::Serialization<PathWithLaneId> path_serialization;

  // Read rosbag with early termination
  size_t message_count = 0;
  bool started_target_window = false;
  constexpr double TIME_WINDOW = 0.1;  // seconds

  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    message_count++;

    const rclcpp::Time msg_time(
      bag_message->time_stamp / 1000000000,  // seconds
      bag_message->time_stamp % 1000000000   // nanoseconds
    );

    const double time_diff = (msg_time - target_time).seconds();

    // Skip messages before target window
    if (time_diff < -TIME_WINDOW) {
      continue;
    }

    // We've reached the target window
    if (!started_target_window) {
      std::cout << "Found target time window! (message #" << message_count
                << ", time: " << msg_time.seconds() << ")" << std::endl;
      started_target_window = true;
    }

    // Exit early if we've passed the target window
    if (time_diff > TIME_WINDOW) {
      std::cout << "Passed target time window (message #" << message_count << ")" << std::endl;
      break;
    }

    if (bag_message->topic_name == "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/goal_planner") {
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      MarkerArray marker_array;
      marker_serialization.deserialize_message(&serialized_msg, &marker_array);

      start_pose = extractStartPoseFromMarkerArray(marker_array);
      found_marker_array = true;
      std::cout << "Found MarkerArray at time: " << msg_time.seconds() << std::endl;
    }

    if (bag_message->topic_name == "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id") {
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      PathWithLaneId path_msg;
      path_serialization.deserialize_message(&serialized_msg, &path_msg);

      path_points = convertToPathPointWithLaneId(path_msg);
      found_path = true;
      std::cout << "Found PathWithLaneId at time: " << msg_time.seconds() << std::endl;
    }

    if (found_marker_array && found_path) {
      break;
    }
  }

  if (!found_marker_array) {
    std::cerr << "Error: Could not find MarkerArray message near target timestamp" << std::endl;
    return 1;
  }

  if (!found_path) {
    std::cerr << "Error: Could not find PathWithLaneId message near target timestamp" << std::endl;
    return 1;
  }

  // Apply lateral acceleration check
  std::cout << "\n=== Applying lateral acceleration check ===" << std::endl;
  std::cout << "Parameters:" << std::endl;
  std::cout << "  lateral_acceleration_threshold: " << lateral_acceleration_threshold << " m/s^2" << std::endl;
  std::cout << "  lateral_acceleration_filtering_duration: " << lateral_acceleration_filtering_duration << " s" << std::endl;
  std::cout << "  velocity: " << velocity << " m/s" << std::endl;
  std::cout << "\n";

  const bool result = autoware::behavior_path_planner::goal_planner_utils::is_lateral_acceleration_acceptable_near_start(
    path_points,
    start_pose,
    velocity,
    lateral_acceleration_filtering_duration,
    lateral_acceleration_threshold,
    true  // is_selected = true to always print debug info
  );

  std::cout << "\nResult: " << (result ? "ACCEPTED" : "REJECTED") << std::endl;

  // Visualization
  std::cout << "\n=== Visualization ===" << std::endl;

  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  // Create figure with good size
  auto [fig, axes] = plt.subplots(1, 1, Kwargs("figsize"_a = pybind11::make_tuple(16, 12)));
  auto & ax = axes[0];

  // Plot the path
  plot_path(ax, path_points, "blue", "Path");

  // Calculate and plot end pose at evaluation distance
  const double evaluation_distance = velocity * lateral_acceleration_filtering_duration;
  const auto end_pose = calculateEndPose(path_points, start_pose, evaluation_distance);

  std::cout << "Evaluation distance: " << evaluation_distance << " m" << std::endl;
  std::cout << "Start pose: x=" << start_pose.position.x
            << ", y=" << start_pose.position.y << std::endl;
  std::cout << "End pose (at " << evaluation_distance << "m): x=" << end_pose.position.x
            << ", y=" << end_pose.position.y << std::endl;

  // Plot start pose as red arrow
  std::vector<double> start_x = {start_pose.position.x};
  std::vector<double> start_y = {start_pose.position.y};
  const double start_yaw = autoware_utils::get_rpy(start_pose).z;
  std::vector<double> start_yaw_cos = {std::cos(start_yaw)};
  std::vector<double> start_yaw_sin = {std::sin(start_yaw)};

  ax.quiver(
    Args(start_x, start_y, start_yaw_cos, start_yaw_sin),
    Kwargs(
      "angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 1.5,
      "color"_a = "red", "width"_a = 0.015, "label"_a = "Start Pose"));

  // Plot end pose as blue arrow
  std::vector<double> end_x = {end_pose.position.x};
  std::vector<double> end_y = {end_pose.position.y};
  const double end_yaw = autoware_utils::get_rpy(end_pose).z;
  std::vector<double> end_yaw_cos = {std::cos(end_yaw)};
  std::vector<double> end_yaw_sin = {std::sin(end_yaw)};

  ax.quiver(
    Args(end_x, end_y, end_yaw_cos, end_yaw_sin),
    Kwargs(
      "angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 1.5,
      "color"_a = "green", "width"_a = 0.015, "label"_a = "End Pose (evaluation point)"));

  // Set plot properties
  ax.set_aspect(Args("equal"));
  ax.grid(Args(true));
  ax.legend();
  ax.set_title(Args("Lateral Acceleration Check Visualization"));
  ax.set_xlabel(Args("X [m]"));
  ax.set_ylabel(Args("Y [m]"));

  std::cout << "Showing visualization..." << std::endl;
  plt.show();

  rclcpp::shutdown();
  return 0;
}
