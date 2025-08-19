// Copyright 2025 TIER IV, Inc.
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

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/conversion/ego.hpp"
#include "autoware/diffusion_planner/conversion/lanelet.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/diffusion_planner/preprocessing/traffic_signals.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"
#include "rosbag_parser.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace autoware::diffusion_planner;
using namespace autoware_perception_msgs::msg;
using namespace autoware_planning_msgs::msg;
using namespace autoware_vehicle_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

// Using constants from dimensions.hpp
constexpr int64_t PAST_TIME_STEPS = EGO_HISTORY_SHAPE[1];  // 21
constexpr int64_t FUTURE_TIME_STEPS = OUTPUT_T;            // 80
constexpr int64_t NEIGHBOR_NUM = NEIGHBOR_SHAPE[1];        // 32
constexpr int64_t STATIC_NUM = STATIC_OBJECTS_SHAPE[1];    // 5
constexpr int64_t LANE_NUM = LANES_SHAPE[1];               // 70
constexpr int64_t LANE_LEN = POINTS_PER_SEGMENT;           // 20
constexpr int64_t ROUTE_NUM = ROUTE_LANES_SHAPE[1];        // 25
constexpr int64_t ROUTE_LEN = POINTS_PER_SEGMENT;          // 20

struct FrameData
{
  int64_t timestamp;
  LaneletRoute route;
  TrackedObjects tracked_objects;
  Odometry kinematic_state;
  AccelWithCovarianceStamped acceleration;
  TrafficLightGroupArray traffic_signals;
  TurnIndicatorsReport turn_indicator;
};

struct SequenceData
{
  std::vector<FrameData> data_list;
  LaneletRoute route;
};

// Training data structure for binary file (all fixed size)
struct TrainingDataBinary
{
  // Header information
  uint32_t version;  // Data format version

  // Fixed size data arrays
  float ego_agent_past[21 * 4];               // (21, 4)
  float ego_current_state[10];                // (10,)
  float ego_agent_future[80 * 4];             // (80, 4)
  float neighbor_agents_past[32 * 21 * 11];   // (32, 21, 11)
  float neighbor_agents_future[32 * 80 * 3];  // (32, 80, 3)
  float static_objects[5 * 10];               // (5, 10)
  float lanes[70 * 20 * 13];                  // (70, 20, 13) - SEGMENT_POINT_DIM=13
  float lanes_speed_limit[70 * 20];           // (70, 20)
  int32_t lanes_has_speed_limit[70];          // (70,)
  float route_lanes[25 * 20 * 13];            // (25, 20, 13) - SEGMENT_POINT_DIM=13
  float route_lanes_speed_limit[25 * 20];     // (25, 20)
  int32_t route_lanes_has_speed_limit[25];    // (25,)
  float goal_pose[3];                         // (3,)
  int32_t turn_indicator;                     // scalar

  // Constructor with zero initialization
  TrainingDataBinary() : version(1), turn_indicator(0)
  {
    std::fill(std::begin(ego_agent_past), std::end(ego_agent_past), 0.0f);
    std::fill(std::begin(ego_current_state), std::end(ego_current_state), 0.0f);
    std::fill(std::begin(ego_agent_future), std::end(ego_agent_future), 0.0f);
    std::fill(std::begin(neighbor_agents_past), std::end(neighbor_agents_past), 0.0f);
    std::fill(std::begin(neighbor_agents_future), std::end(neighbor_agents_future), 0.0f);
    std::fill(std::begin(static_objects), std::end(static_objects), 0.0f);
    std::fill(std::begin(lanes), std::end(lanes), 0.0f);
    std::fill(std::begin(lanes_speed_limit), std::end(lanes_speed_limit), 0.0f);
    std::fill(std::begin(lanes_has_speed_limit), std::end(lanes_has_speed_limit), 0);
    std::fill(std::begin(route_lanes), std::end(route_lanes), 0.0f);
    std::fill(std::begin(route_lanes_speed_limit), std::end(route_lanes_speed_limit), 0.0f);
    std::fill(std::begin(route_lanes_has_speed_limit), std::end(route_lanes_has_speed_limit), 0);
    std::fill(std::begin(goal_pose), std::end(goal_pose), 0.0f);
  }
};

int64_t parse_timestamp(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

template <typename T>
bool check_and_update_msg(
  std::deque<T> & msgs, const builtin_interfaces::msg::Time & target_stamp,
  const std::string & topic_name, T & result_msg)
{
  if (msgs.empty()) {
    std::cout << "Cannot find " << topic_name << " msg" << std::endl;
    return false;
  }

  const int64_t target_time = parse_timestamp(target_stamp);
  T best_msg = msgs.front();
  int64_t best_diff = std::numeric_limits<int64_t>::max();
  int64_t best_index = 0;

  for (int64_t i = 0; i < static_cast<int64_t>(msgs.size()); ++i) {
    const auto & msg = msgs[i];
    builtin_interfaces::msg::Time msg_stamp;
    if constexpr (
      std::is_same_v<T, Odometry> || std::is_same_v<T, TrackedObjects> ||
      std::is_same_v<T, AccelWithCovarianceStamped>) {
      msg_stamp = msg.header.stamp;
    } else if constexpr (
      std::is_same_v<T, TurnIndicatorsReport> || std::is_same_v<T, TrafficLightGroupArray>) {
      msg_stamp = msg.stamp;
    }

    const int64_t msg_time = parse_timestamp(msg_stamp);
    const int64_t diff = std::abs(target_time - msg_time);

    if (diff < best_diff) {
      best_diff = diff;
      best_msg = msg;
      best_index = i;
    }
  }

  if (best_diff > static_cast<int64_t>(0.2 * 1e9)) {  // Over 200 msec
    std::cout << "Over 200 msec: " << topic_name << ", msgs.size()=" << msgs.size()
              << ", diff=" << best_diff << std::endl;
    return false;
  }

  // Remove processed messages (like Python version)
  msgs.erase(msgs.begin(), msgs.begin() + best_index);
  result_msg = best_msg;
  return true;
}

std::vector<float> create_ego_sequence(
  const std::vector<FrameData> & data_list, const int64_t start_idx, const int64_t time_steps,
  const Eigen::Matrix4f & map2bl_matrix)
{
  // Extract odometry messages from FrameData
  std::deque<nav_msgs::msg::Odometry> odometry_deque;
  for (int64_t j = 0; j < time_steps; ++j) {
    const int64_t index = std::min(start_idx + j, static_cast<int64_t>(data_list.size()) - 1);
    odometry_deque.push_back(data_list[index].kinematic_state);
  }
  return preprocess::create_ego_agent_past(odometry_deque, time_steps, map2bl_matrix);
}

std::vector<float> process_neighbor_agents(
  const std::vector<FrameData> & data_list, const int64_t current_idx,
  const Eigen::Matrix4f & map2bl_matrix)
{
  std::vector<float> neighbor_data(NEIGHBOR_NUM * PAST_TIME_STEPS * 11, 0.0f);

  // Track agents across multiple frames for past data
  std::map<std::string, std::vector<AgentState>> agent_histories;

  // Collect past data
  for (int64_t t = 0; t < PAST_TIME_STEPS; ++t) {
    const int64_t frame_idx =
      std::max(static_cast<int64_t>(0), current_idx - PAST_TIME_STEPS + 1 + t);
    if (frame_idx >= static_cast<int64_t>(data_list.size())) {
      break;
    }

    const std::vector<TrackedObject> & objects = data_list[frame_idx].tracked_objects.objects;
    for (const TrackedObject & obj : objects) {
      const std::string obj_id = autoware_utils_uuid::to_hex_string(obj.object_id);
      AgentState agent_state(obj);
      agent_state.apply_transform(map2bl_matrix);

      agent_histories[obj_id].push_back(agent_state);
    }
  }

  // Sort agents by distance to ego (at current frame)
  std::vector<std::pair<std::string, float>> agent_distances;
  for (const auto & [obj_id, history] : agent_histories) {
    if (!history.empty()) {
      const auto & last_state = history.back();
      float dist = std::sqrt(last_state.x() * last_state.x() + last_state.y() * last_state.y());
      agent_distances.push_back({obj_id, dist});
    }
  }

  std::sort(agent_distances.begin(), agent_distances.end(), [](const auto & a, const auto & b) {
    return a.second < b.second;
  });

  // Fill neighbor data for closest agents
  int64_t agent_idx = 0;
  for (const std::pair<std::string, float> & agent_distance : agent_distances) {
    if (agent_idx >= NEIGHBOR_NUM) break;

    const std::vector<AgentState> & history = agent_histories[agent_distance.first];
    for (int64_t t = 0; t < PAST_TIME_STEPS; ++t) {
      const int64_t base_idx = agent_idx * PAST_TIME_STEPS * 11 + t * 11;

      if (t < static_cast<int64_t>(history.size())) {
        const AgentState & state = history[t];
        const std::array<float, AGENT_STATE_DIM> data_array = state.as_array();
        for (size_t i = 0; i < data_array.size(); ++i) {
          neighbor_data[base_idx + i] = data_array[i];
        }
      }
    }
    agent_idx++;
  }

  return neighbor_data;
}

std::vector<float> process_neighbor_future(
  const std::vector<FrameData> & data_list, const int64_t current_idx,
  const Eigen::Matrix4f & map2bl_matrix)
{
  std::vector<float> neighbor_future(NEIGHBOR_NUM * FUTURE_TIME_STEPS * 3, 0.0f);

  // Get current frame objects for tracking
  if (current_idx >= static_cast<int64_t>(data_list.size())) return neighbor_future;

  const std::vector<TrackedObject> & current_objects =
    data_list[current_idx].tracked_objects.objects;
  std::map<std::string, int64_t> object_id_to_idx;

  // Create mapping of object IDs to indices (sorted by distance)
  std::vector<std::pair<std::string, float>> agent_distances;
  for (const TrackedObject & obj : current_objects) {
    const std::string obj_id = autoware_utils_uuid::to_hex_string(obj.object_id);

    const Eigen::Vector4f pos_vec(
      obj.kinematics.pose_with_covariance.pose.position.x,
      obj.kinematics.pose_with_covariance.pose.position.y,
      obj.kinematics.pose_with_covariance.pose.position.z, 1.0);
    const Eigen::Vector4f transformed_pos = map2bl_matrix * pos_vec;
    const float dist = std::sqrt(
      transformed_pos.x() * transformed_pos.x() + transformed_pos.y() * transformed_pos.y());
    agent_distances.push_back({obj_id, dist});
  }

  std::sort(agent_distances.begin(), agent_distances.end(), [](const auto & a, const auto & b) {
    return a.second < b.second;
  });

  for (int64_t i = 0; i < static_cast<int64_t>(agent_distances.size()) && i < NEIGHBOR_NUM; ++i) {
    object_id_to_idx[agent_distances[i].first] = i;
  }

  // Fill future data
  for (int64_t t = 0; t < FUTURE_TIME_STEPS; ++t) {
    const int64_t future_frame_idx = current_idx + 1 + t;
    if (future_frame_idx >= static_cast<int64_t>(data_list.size())) break;

    const std::vector<TrackedObject> & future_objects =
      data_list[future_frame_idx].tracked_objects.objects;

    for (const TrackedObject & obj : future_objects) {
      const std::string obj_id = autoware_utils_uuid::to_hex_string(obj.object_id);

      if (object_id_to_idx.find(obj_id) != object_id_to_idx.end()) {
        const int64_t agent_idx = object_id_to_idx[obj_id];
        const int64_t base_idx = agent_idx * FUTURE_TIME_STEPS * 3 + t * 3;

        const Point & pos = obj.kinematics.pose_with_covariance.pose.position;
        const Quaternion & ori = obj.kinematics.pose_with_covariance.pose.orientation;

        const Eigen::Vector4f pos_vec(pos.x, pos.y, pos.z, 1.0);
        const Eigen::Vector4f transformed_pos = map2bl_matrix * pos_vec;

        const Eigen::Quaternionf quat(ori.w, ori.x, ori.y, ori.z);
        const Eigen::Matrix3f rot_matrix = quat.toRotationMatrix();
        const Eigen::Matrix3f transformed_rot = map2bl_matrix.block<3, 3>(0, 0) * rot_matrix;
        const float yaw = std::atan2(transformed_rot(1, 0), transformed_rot(0, 0));

        neighbor_future[base_idx + 0] = transformed_pos.x();
        neighbor_future[base_idx + 1] = transformed_pos.y();
        neighbor_future[base_idx + 2] = yaw;
      }
    }
  }

  return neighbor_future;
}

void save_binary_data(
  const std::string & output_path, const std::string & rosbag_dir_name, const std::string & token,
  const std::vector<float> & ego_past, const std::vector<float> & ego_current,
  const std::vector<float> & ego_future, const std::vector<float> & neighbor_past,
  const std::vector<float> & neighbor_future, const std::vector<float> & static_objects,
  const std::vector<float> & lanes, const std::vector<float> & lanes_speed_limit,
  const std::vector<bool> & lanes_has_speed_limit, const std::vector<float> & route_lanes,
  const std::vector<float> & route_lanes_speed_limit,
  const std::vector<bool> & route_lanes_has_speed_limit, const std::vector<float> & goal_pose,
  const int64_t turn_indicator)
{
  namespace fs = std::filesystem;

  fs::create_directories(output_path);

  // Copy data to struct
  TrainingDataBinary data;

  // Copy vector data to fixed arrays
  std::copy(ego_past.begin(), ego_past.end(), data.ego_agent_past);
  std::copy(ego_current.begin(), ego_current.end(), data.ego_current_state);
  std::copy(ego_future.begin(), ego_future.end(), data.ego_agent_future);
  std::copy(neighbor_past.begin(), neighbor_past.end(), data.neighbor_agents_past);
  std::copy(neighbor_future.begin(), neighbor_future.end(), data.neighbor_agents_future);
  std::copy(static_objects.begin(), static_objects.end(), data.static_objects);
  std::copy(lanes.begin(), lanes.end(), data.lanes);
  std::copy(lanes_speed_limit.begin(), lanes_speed_limit.end(), data.lanes_speed_limit);
  std::copy(route_lanes.begin(), route_lanes.end(), data.route_lanes);
  std::copy(
    route_lanes_speed_limit.begin(), route_lanes_speed_limit.end(), data.route_lanes_speed_limit);
  std::copy(goal_pose.begin(), goal_pose.end(), data.goal_pose);

  // Convert bool to int32_t and copy
  for (size_t i = 0; i < lanes_has_speed_limit.size(); ++i) {
    data.lanes_has_speed_limit[i] = static_cast<int32_t>(lanes_has_speed_limit[i]);
  }
  for (size_t i = 0; i < route_lanes_has_speed_limit.size(); ++i) {
    data.route_lanes_has_speed_limit[i] = static_cast<int32_t>(route_lanes_has_speed_limit[i]);
  }

  data.turn_indicator = static_cast<int32_t>(turn_indicator);

  // Save to binary file with rosbag directory name prefix (same format as Python version)
  const std::string binary_filename = output_path + "/" + rosbag_dir_name + "_" + token + ".bin";
  std::ofstream file(binary_filename, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Failed to open file for writing: " << binary_filename << std::endl;
    return;
  }

  file.write(reinterpret_cast<const char *>(&data), sizeof(TrainingDataBinary));

  if (file.fail()) {
    std::cerr << "Failed to write data to file: " << binary_filename << std::endl;
    return;
  }

  file.close();
}

int main(int argc, char ** argv)
{
  // Initialize ROS2 for route handler functionality
  rclcpp::init(argc, argv);

  if (argc < 4) {
    std::cerr << "Usage: data_converter <rosbag_path> <vector_map_path> <save_dir> [--step=1] "
                 "[--limit=-1] [--min_frames=1700]"
              << std::endl;
    return 1;
  }

  const std::string rosbag_path = argv[1];
  const std::string vector_map_path = argv[2];
  const std::string save_dir = argv[3];

  int64_t step = 1;
  int64_t limit = -1;
  int64_t min_frames = 1700;
  int64_t search_nearest_route = 1;

  // Parse optional arguments
  for (int64_t i = 4; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg.find("--step=") == 0) {
      step = std::stoll(arg.substr(7));
    } else if (arg.find("--limit=") == 0) {
      limit = std::stoll(arg.substr(8));
    } else if (arg.find("--min_frames=") == 0) {
      min_frames = std::stoll(arg.substr(13));
    } else if (arg.find("--search_nearest_route=") == 0) {
      search_nearest_route = std::stoll(arg.substr(24));
    }
  }

  std::cout << "Processing rosbag: " << rosbag_path << std::endl;
  std::cout << "Vector map: " << vector_map_path << std::endl;
  std::cout << "Save directory: " << save_dir << std::endl;
  std::cout << "Step: " << step << ", Limit: " << limit << ", Min frames: " << min_frames
            << ", Search nearest route: " << search_nearest_route << std::endl;

  // Load Lanelet2 map and create context like in diffusion_planner_node
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr;

  // Create HADMapBin message from the OSM file
  const autoware_map_msgs::msg::LaneletMapBin map_bin_msg =
    autoware::test_utils::make_map_bin_msg(vector_map_path, 1.0);

  // Convert HADMapBin to lanelet map
  lanelet::utils::conversion::fromBinMsg(
    map_bin_msg, lanelet_map_ptr, &traffic_rules_ptr, &routing_graph_ptr);

  std::cout << "Loaded lanelet2 map with " << lanelet_map_ptr->laneletLayer.size() << " lanelets"
            << std::endl;

  const preprocess::LaneSegmentContext lane_segment_context(lanelet_map_ptr);

  // Create route handler
  autoware::route_handler::RouteHandler route_handler(map_bin_msg);

  rosbag_parser::RosbagParser rosbag_parser(rosbag_path);
  rosbag_parser.create_reader(rosbag_path);

  // Parse messages from specific topics
  std::deque<Odometry> kinematic_states;
  std::deque<AccelWithCovarianceStamped> accelerations;
  std::deque<TrackedObjects> tracked_objects_msgs;
  std::deque<TurnIndicatorsReport> turn_indicators;
  std::vector<LaneletRoute> route_msgs;
  std::deque<TrafficLightGroupArray> traffic_signals;

  const std::vector<std::string> target_topics = {
    "/localization/kinematic_state",
    "/localization/acceleration",
    "/perception/object_recognition/tracking/objects",
    "/planning/mission_planning/route",
    "/vehicle/status/turn_indicators_status",
    "/perception/traffic_light_recognition/traffic_signals"};

  int64_t parse_count = 0;
  while (rosbag_parser.has_next() && (limit < 0 || parse_count < limit)) {
    const rosbag2_storage::SerializedBagMessageSharedPtr msg = rosbag_parser.read_next();

    if (msg->topic_name == "/localization/kinematic_state") {
      const Odometry odometry = rosbag_parser.deserialize_message<Odometry>(msg);
      kinematic_states.push_back(odometry);
    } else if (msg->topic_name == "/localization/acceleration") {
      const AccelWithCovarianceStamped accel =
        rosbag_parser.deserialize_message<AccelWithCovarianceStamped>(msg);
      accelerations.push_back(accel);
    } else if (msg->topic_name == "/perception/object_recognition/tracking/objects") {
      const TrackedObjects objects = rosbag_parser.deserialize_message<TrackedObjects>(msg);
      tracked_objects_msgs.push_back(objects);
    } else if (msg->topic_name == "/planning/mission_planning/route") {
      const LaneletRoute route = rosbag_parser.deserialize_message<LaneletRoute>(msg);
      route_msgs.push_back(route);
    } else if (msg->topic_name == "/vehicle/status/turn_indicators_status") {
      const TurnIndicatorsReport turn_ind =
        rosbag_parser.deserialize_message<TurnIndicatorsReport>(msg);
      turn_indicators.push_back(turn_ind);
    } else if (msg->topic_name == "/perception/traffic_light_recognition/traffic_signals") {
      const TrafficLightGroupArray traffic_signal =
        rosbag_parser.deserialize_message<TrafficLightGroupArray>(msg);
      traffic_signals.push_back(traffic_signal);
    }

    parse_count++;
  }

  std::cout << "Parsed " << kinematic_states.size() << " kinematic states" << std::endl;
  std::cout << "Parsed " << accelerations.size() << " acceleration messages" << std::endl;
  std::cout << "Parsed " << tracked_objects_msgs.size() << " tracked objects" << std::endl;
  std::cout << "Parsed " << route_msgs.size() << " route messages" << std::endl;
  std::cout << "Parsed " << turn_indicators.size() << " turn indicator messages" << std::endl;
  std::cout << "Parsed " << traffic_signals.size() << " traffic signal messages" << std::endl;

  if (route_msgs.empty()) {
    std::cerr << "No route messages found in rosbag" << std::endl;
    return 1;
  }

  if (tracked_objects_msgs.empty()) {
    std::cerr << "No tracked objects found in rosbag" << std::endl;
    return 1;
  }

  // Create sequences based on tracked objects (base topic at 10Hz)
  std::vector<SequenceData> sequences;
  for (const LaneletRoute & route : route_msgs) {
    sequences.push_back({{}, route});
  }

  // Process each tracked objects message with synchronization like Python version
  const int64_t n = static_cast<int64_t>(tracked_objects_msgs.size());
  std::cout << "n=" << n << std::endl;

  for (int64_t i = 0; i < n; ++i) {
    const TrackedObjects & tracking = tracked_objects_msgs[i];
    const int64_t timestamp = parse_timestamp(tracking.header.stamp);

    // Find matching messages with synchronization check like Python version
    Odometry kinematic;
    AccelWithCovarianceStamped accel;
    TrafficLightGroupArray traffic_signal;
    TurnIndicatorsReport turn_ind;

    bool ok = true;

    // Check all messages
    ok =
      ok && check_and_update_msg(
              kinematic_states, tracking.header.stamp, "/localization/kinematic_state", kinematic);
    ok = ok && check_and_update_msg(
                 accelerations, tracking.header.stamp, "/localization/acceleration", accel);
    ok = ok && check_and_update_msg(
                 traffic_signals, tracking.header.stamp,
                 "/perception/traffic_light_recognition/traffic_signals", traffic_signal);
    ok = ok && check_and_update_msg(
                 turn_indicators, tracking.header.stamp, "/vehicle/status/turn_indicators_status",
                 turn_ind);

    // Check route
    int64_t max_route_index = -1;
    if (search_nearest_route) {
      // Find the latest route msg
      int64_t max_route_timestamp = 0;
      for (int64_t j = 0; j < static_cast<int64_t>(route_msgs.size()); ++j) {
        const LaneletRoute & route_msg = route_msgs[j];
        const int64_t route_stamp = parse_timestamp(route_msg.header.stamp);
        if (max_route_timestamp <= route_stamp && route_stamp <= timestamp) {
          max_route_timestamp = route_stamp;
          max_route_index = j;
        }
      }
      if (max_route_index == -1) {
        std::cout << "Cannot find route msg at " << i << std::endl;
        continue;
      }
    } else {
      // Use the first route msg
      max_route_index = 0;
    }

    // Check kinematic_state covariance validation
    if (ok) {
      const std::array<double, 36> & covariance = kinematic.pose.covariance;
      const double covariance_xx = covariance[0];
      const double covariance_yy = covariance[7];

      if (covariance_xx > 1e-1 || covariance_yy > 1e-1) {
        std::cout << "Invalid kinematic_state covariance_xx=" << covariance_xx
                  << ", covariance_yy=" << covariance_yy << std::endl;
        ok = false;
      }
    }

    SequenceData & sequence = sequences[max_route_index];

    // Handle frame based on validation result
    if (!ok) {
      if (sequence.data_list.empty()) {
        // At the beginning of recording, some msgs may be missing - Skip this frame
        std::cout << "Skip this frame i=" << i << "/n=" << n << std::endl;
        continue;
      } else {
        // If the msg is missing in the middle of recording, we can use the msgs to this point
        std::cout << "Finish at this frame i=" << i << "/n=" << n << std::endl;
        break;
      }
    }

    const FrameData frame_data{timestamp, sequence.route, tracking, kinematic,
                               accel,     traffic_signal, turn_ind};

    sequence.data_list.push_back(frame_data);
  }

  // Because FreeSpacePlanner sometimes changes goal_pose at the end, combine such things.
  for (int64_t i = static_cast<int64_t>(sequences.size()) - 2; i >= 0; --i) {
    const LaneletRoute & route_msg_l = sequences[i].route;
    const LaneletRoute & route_msg_r = sequences[i + 1].route;

    if (route_msg_l.start_pose != route_msg_r.start_pose) {
      std::cout << "Route start pose mismatch: " << i << " != " << i + 1 << std::endl;
      continue;
    }

    std::cout << "Concatenate sequence " << i << " and " << i + 1 << std::endl;
    std::cout << "Before sequence[" << i << "].data_list.size()=" << sequences[i].data_list.size()
              << " frames" << std::endl;

    sequences[i].data_list.insert(
      sequences[i].data_list.end(), sequences[i + 1].data_list.begin(),
      sequences[i + 1].data_list.end());

    std::cout << "After sequence[" << i << "].data_list.size()=" << sequences[i].data_list.size()
              << " frames" << std::endl;

    sequences.erase(sequences.begin() + i + 1);
  }

  const std::string rosbag_dir_name = std::filesystem::path(rosbag_path).filename();
  const int64_t sequence_num = static_cast<int64_t>(sequences.size());
  std::cout << "Total " << sequence_num << " sequences" << std::endl;

  // Process sequences
  for (int64_t seq_id = 0; seq_id < static_cast<int64_t>(sequences.size()); ++seq_id) {
    SequenceData & seq = sequences[seq_id];
    const int64_t n = static_cast<int64_t>(seq.data_list.size());

    std::cout << "Processing sequence " << seq_id + 1 << "/" << sequences.size() << " with " << n
              << " frames" << std::endl;

    if (n < min_frames) {
      std::cout << "Skipping sequence with only " << n << " frames (min: " << min_frames << ")"
                << std::endl;
      continue;
    }

    route_handler.setRoute(seq.route);
    if (!route_handler.isHandlerReady()) {
      std::cout << "Route handler is not ready for sequence " << seq_id + 1 << std::endl;
      return 1;
    }

    // Process frames with stopping count tracking
    int64_t stopping_count = 0;
    for (int64_t i = PAST_TIME_STEPS; i < n - FUTURE_TIME_STEPS; i += step) {
      // Create token in same format as Python version: seq_id(8digits) + i(8digits)
      std::ostringstream token_stream;
      token_stream << std::setfill('0') << std::setw(8) << seq_id << std::setw(8) << i;
      const std::string token = token_stream.str();

      // Get transformation matrix
      const auto [bl2map, map2bl] = utils::get_transform_matrix(seq.data_list[i].kinematic_state);

      // Create ego sequences
      const std::vector<float> ego_past =
        create_ego_sequence(seq.data_list, i - PAST_TIME_STEPS + 1, PAST_TIME_STEPS, map2bl);
      const std::vector<float> ego_future =
        create_ego_sequence(seq.data_list, i + 1, FUTURE_TIME_STEPS, map2bl);

      // Create ego current state
      const EgoState ego_state(
        seq.data_list[i].kinematic_state, seq.data_list[i].acceleration, 2.79f);
      const std::vector<float> ego_current = ego_state.as_array();

      // Process neighbor agents
      const std::vector<float> neighbor_past = process_neighbor_agents(seq.data_list, i, map2bl);
      const std::vector<float> neighbor_future = process_neighbor_future(seq.data_list, i, map2bl);

      // Process lanes and routes
      const Point & ego_pos = seq.data_list[i].kinematic_state.pose.pose.position;
      const float center_x = ego_pos.x;
      const float center_y = ego_pos.y;

      // Process traffic signals for this frame using the traffic signals from FrameData
      std::map<lanelet::Id, preprocess::TrafficSignalStamped> traffic_light_id_map;
      const auto current_stamp = seq.data_list[i].tracked_objects.header.stamp;
      const rclcpp::Time current_time(current_stamp);

      auto msg_ptr = std::make_shared<TrafficLightGroupArray>(seq.data_list[i].traffic_signals);
      preprocess::process_traffic_signals(msg_ptr, traffic_light_id_map, current_time, 5.0, true);

      // Get lanes data with speed limits
      const auto [lanes, lanes_speed_limit] = lane_segment_context.get_lane_segments(
        map2bl, traffic_light_id_map, center_x, center_y, LANE_LEN);

      // Create has_speed_limit flags based on speed_limit values
      std::vector<bool> lanes_has_speed_limit(lanes_speed_limit.size());
      for (size_t idx = 0; idx < lanes_speed_limit.size(); ++idx) {
        lanes_has_speed_limit[idx] =
          (lanes_speed_limit[idx] > std::numeric_limits<float>::epsilon());
      }

      // Get route lanes data with speed limits
      std::vector<float> route_lanes(ROUTE_NUM * ROUTE_LEN * SEGMENT_POINT_DIM, 0.0f);
      std::vector<float> route_lanes_speed_limit(ROUTE_NUM * ROUTE_LEN, 0.0f);

      geometry_msgs::msg::Pose current_pose;
      current_pose.position = ego_pos;
      current_pose.orientation.w = 1.0;  // Identity quaternion

      lanelet::ConstLanelet current_preferred_lane;
      if (route_handler.getClosestPreferredLaneletWithinRoute(
            current_pose, &current_preferred_lane)) {
        constexpr double backward_path_length = constants::BACKWARD_PATH_LENGTH_M;
        constexpr double forward_path_length = constants::FORWARD_PATH_LENGTH_M;
        const auto current_lanes = route_handler.getLaneletSequence(
          current_preferred_lane, backward_path_length, forward_path_length);

        const auto [route_data, route_speed] =
          lane_segment_context.get_route_segments(map2bl, traffic_light_id_map, current_lanes);
        route_lanes = route_data;
        route_lanes_speed_limit = route_speed;
      }

      // Create route_lanes_has_speed_limit based on speed_limit values
      std::vector<bool> route_lanes_has_speed_limit(route_lanes_speed_limit.size());
      for (size_t idx = 0; idx < route_lanes_speed_limit.size(); ++idx) {
        route_lanes_has_speed_limit[idx] =
          (route_lanes_speed_limit[idx] > std::numeric_limits<float>::epsilon());
      }

      // Get goal pose
      const geometry_msgs::msg::Pose & goal_pose = seq.data_list[i].route.goal_pose;
      const Eigen::Matrix4f goal_pose_matrix = utils::pose_to_matrix4f(goal_pose);
      const Eigen::Vector4f goal_pos_bl =
        map2bl * Eigen::Vector4f(
                   goal_pose_matrix(0, 3), goal_pose_matrix(1, 3), goal_pose_matrix(2, 3), 1.0);

      // Get goal orientation in base_link frame
      const Eigen::Quaternionf goal_quat(
        goal_pose.orientation.w, goal_pose.orientation.x, goal_pose.orientation.y,
        goal_pose.orientation.z);
      const Eigen::Matrix3f goal_rot_map = goal_quat.toRotationMatrix();
      const Eigen::Matrix3f goal_rot_bl = map2bl.block<3, 3>(0, 0) * goal_rot_map;
      const float goal_yaw_bl = std::atan2(goal_rot_bl(1, 0), goal_rot_bl(0, 0));

      // Convert goal pose to vector for saving
      const std::vector<float> goal_pose_vec = {goal_pos_bl.x(), goal_pos_bl.y(), goal_yaw_bl};

      // Such data should be skipped.
      // (1)Ego vehicle is stopped
      // (2)The lanelet segment in front is a red light
      // (3)The GT trajectory is moving forward.

      // (1)Ego vehicle is stopped
      const bool is_stop = seq.data_list[i].kinematic_state.twist.twist.linear.x < 0.1;
      if (is_stop) {
        stopping_count++;
      } else {
        stopping_count = 0;
      }

      // if ego vehicle is stopped and close to goal, finish
      const float ego_future_last_x = ego_future[(FUTURE_TIME_STEPS - 1) * 4 + 0];
      const float ego_future_last_y = ego_future[(FUTURE_TIME_STEPS - 1) * 4 + 1];
      const float distance_to_goal_pose = std::sqrt(
        (ego_future_last_x - goal_pos_bl.x()) * (ego_future_last_x - goal_pos_bl.x()) +
        (ego_future_last_y - goal_pos_bl.y()) * (ego_future_last_y - goal_pos_bl.y()));

      if (stopping_count >= 10 && distance_to_goal_pose < 5.0) {
        std::cout << "finish at " << i << " because stopping_count=" << stopping_count
                  << " and distance_to_goal_pose=" << distance_to_goal_pose << std::endl;
        break;
      }

      // Check for red light (next segment)
      // route_tensor[:, 1, 0, -3] corresponds to second segment, first point, red light flag
      const int64_t segment_idx = 1;  // next segment
      const int64_t point_idx = 0;    // first point
      const int64_t red_light_index = segment_idx * ROUTE_LEN * SEGMENT_POINT_DIM +
                                      point_idx * SEGMENT_POINT_DIM + TRAFFIC_LIGHT_RED;
      const bool is_red_light = route_lanes[red_light_index] > 0.5;

      float sum_mileage = 0.0;
      for (int64_t j = 0; j < FUTURE_TIME_STEPS - 1; ++j) {
        const float dx = ego_future[(j + 1) * 4 + 0] - ego_future[j * 4 + 0];
        const float dy = ego_future[(j + 1) * 4 + 1] - ego_future[j * 4 + 1];
        sum_mileage += std::sqrt(dx * dx + dy * dy);
      }
      const bool is_future_forward = sum_mileage > 0.1;

      if (is_stop && is_red_light && is_future_forward) {
        std::cout << "Skip this frame " << i
                  << " because it is stop at red light and future trajectory is forward"
                  << std::endl;
        continue;
      }

      // Create placeholder data for static objects
      const std::vector<float> static_objects(STATIC_NUM * 10, 0.0f);

      const int64_t turn_indicator = seq.data_list[i].turn_indicator.report;

      // Save data
      save_binary_data(
        save_dir, rosbag_dir_name, token, ego_past, ego_current, ego_future, neighbor_past,
        neighbor_future, static_objects, lanes, lanes_speed_limit, lanes_has_speed_limit,
        route_lanes, route_lanes_speed_limit, route_lanes_has_speed_limit, goal_pose_vec,
        turn_indicator);

      if (i % 100 == 0) {
        std::cout << "Processed frame " << i << "/" << n << std::endl;
      }
    }
  }

  std::cout << "Data conversion completed!" << std::endl;

  rclcpp::shutdown();
}
