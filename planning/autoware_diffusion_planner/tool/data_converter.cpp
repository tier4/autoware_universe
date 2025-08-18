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
#include "numpy.hpp"
#include "rosbag_parser.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <map>
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
  TurnIndicatorsReport turn_indicator;
};

struct SequenceData
{
  std::vector<FrameData> data_list;
  LaneletRoute route;
};

int64_t parse_timestamp(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

template <typename T>
T get_nearest_msg(const std::deque<T> & msgs, const builtin_interfaces::msg::Time & target_stamp)
{
  if (msgs.empty()) {
    throw std::runtime_error("No messages available");
  }

  const int64_t target_time = parse_timestamp(target_stamp);
  T best_msg = msgs.front();
  int64_t best_diff = std::numeric_limits<int64_t>::max();

  for (const auto & msg : msgs) {
    builtin_interfaces::msg::Time msg_stamp;
    if constexpr (
      std::is_same_v<T, Odometry> || std::is_same_v<T, TrackedObjects> ||
      std::is_same_v<T, AccelWithCovarianceStamped>) {
      msg_stamp = msg.header.stamp;
    } else if constexpr (std::is_same_v<T, TurnIndicatorsReport>) {
      msg_stamp = msg.stamp;
    }

    const int64_t msg_time = parse_timestamp(msg_stamp);
    const int64_t diff = std::abs(target_time - msg_time);

    if (diff < best_diff) {
      best_diff = diff;
      best_msg = msg;
    }
  }

  return best_msg;
}

Eigen::Matrix4f get_transform_matrix(const Odometry & odometry)
{
  const geometry_msgs::msg::Point & pos = odometry.pose.pose.position;
  const geometry_msgs::msg::Quaternion & ori = odometry.pose.pose.orientation;

  const Eigen::Quaternionf quat(ori.w, ori.x, ori.y, ori.z);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = quat.toRotationMatrix();
  transform(0, 3) = pos.x;
  transform(1, 3) = pos.y;
  transform(2, 3) = pos.z;
  return transform;
}

std::vector<float> create_ego_sequence(
  const std::vector<FrameData> & data_list, const int64_t start_idx, const int64_t time_steps,
  const Eigen::Matrix4f & map2bl_matrix)
{
  std::vector<float> ego_sequence;
  ego_sequence.reserve(time_steps * 4);  // x, y, cos_yaw, sin_yaw

  for (int64_t j = 0; j < time_steps; ++j) {
    const int64_t index = std::min(start_idx + j, static_cast<int64_t>(data_list.size()) - 1);
    const Point & pos = data_list[index].kinematic_state.pose.pose.position;
    const Quaternion & ori = data_list[index].kinematic_state.pose.pose.orientation;

    const Eigen::Vector4f pos_vec(pos.x, pos.y, pos.z, 1.0);
    const Eigen::Vector4f transformed_pos = map2bl_matrix * pos_vec;

    const Eigen::Quaternionf quat(ori.w, ori.x, ori.y, ori.z);
    const Eigen::Matrix3f rot_matrix = quat.toRotationMatrix();
    const Eigen::Matrix3f transformed_rot = map2bl_matrix.block<3, 3>(0, 0) * rot_matrix;
    const float yaw = std::atan2(transformed_rot(1, 0), transformed_rot(0, 0));

    ego_sequence.push_back(transformed_pos.x());
    ego_sequence.push_back(transformed_pos.y());
    ego_sequence.push_back(std::cos(yaw));
    ego_sequence.push_back(std::sin(yaw));
  }

  return ego_sequence;
}

std::vector<float> create_ego_future_sequence(
  const std::vector<FrameData> & data_list, const int64_t start_idx, const int64_t time_steps,
  const Eigen::Matrix4f & map2bl_matrix)
{
  std::vector<float> ego_future;
  ego_future.reserve(time_steps * 3);  // x, y, yaw

  for (int64_t j = 0; j < time_steps; ++j) {
    const int64_t index = std::min(start_idx + j, static_cast<int64_t>(data_list.size()) - 1);
    const Point & pos = data_list[index].kinematic_state.pose.pose.position;
    const Quaternion & ori = data_list[index].kinematic_state.pose.pose.orientation;

    const Eigen::Vector4f pos_vec(pos.x, pos.y, pos.z, 1.0);
    const Eigen::Vector4f transformed_pos = map2bl_matrix * pos_vec;

    const Eigen::Quaternionf quat(ori.w, ori.x, ori.y, ori.z);
    const Eigen::Matrix3f rot_matrix = quat.toRotationMatrix();
    const Eigen::Matrix3f transformed_rot = map2bl_matrix.block<3, 3>(0, 0) * rot_matrix;
    const float yaw = std::atan2(transformed_rot(1, 0), transformed_rot(0, 0));

    ego_future.push_back(transformed_pos.x());
    ego_future.push_back(transformed_pos.y());
    ego_future.push_back(yaw);
  }

  return ego_future;
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
      AgentState agent_state(const_cast<TrackedObject &>(obj));
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

std::vector<float> process_lanes(
  const preprocess::LaneSegmentContext & lane_segment_context, const Point & ego_position,
  const Eigen::Matrix4f & map2bl_matrix,
  const std::map<lanelet::Id, preprocess::TrafficSignalStamped> & traffic_light_id_map)
{
  const float center_x = ego_position.x;
  const float center_y = ego_position.y;
  const auto [lanes_data, lanes_speed_limit] = lane_segment_context.get_lane_segments(
    map2bl_matrix, traffic_light_id_map, center_x, center_y, LANE_LEN);

  return lanes_data;
}

std::vector<float> process_route_lanes(
  const preprocess::LaneSegmentContext & lane_segment_context,
  const autoware::route_handler::RouteHandler & route_handler, const Point & ego_position,
  const Eigen::Matrix4f & map2bl_matrix,
  const std::map<lanelet::Id, preprocess::TrafficSignalStamped> & traffic_light_id_map)
{
  if (!route_handler.isHandlerReady()) {
    std::vector<float> route_lanes_data(ROUTE_NUM * ROUTE_LEN * SEGMENT_POINT_DIM, 0.0f);
    return route_lanes_data;
  }

  // Get current pose from ego position
  geometry_msgs::msg::Pose current_pose;
  current_pose.position = ego_position;
  current_pose.orientation.w = 1.0;  // Identity quaternion

  lanelet::ConstLanelet current_preferred_lane;
  if (!route_handler.getClosestPreferredLaneletWithinRoute(current_pose, &current_preferred_lane)) {
    std::vector<float> route_lanes_data(ROUTE_NUM * ROUTE_LEN * SEGMENT_POINT_DIM, 0.0f);
    return route_lanes_data;
  }

  constexpr double backward_path_length = constants::BACKWARD_PATH_LENGTH_M;
  constexpr double forward_path_length = constants::FORWARD_PATH_LENGTH_M;
  const auto current_lanes = route_handler.getLaneletSequence(
    current_preferred_lane, backward_path_length, forward_path_length);

  const auto [route_lanes_data, route_lanes_speed_limit] =
    lane_segment_context.get_route_segments(map2bl_matrix, traffic_light_id_map, current_lanes);

  return route_lanes_data;
}

void save_npz_data(
  const std::string & output_path, const std::string & token, const std::vector<float> & ego_past,
  const std::vector<float> & ego_current, const std::vector<float> & ego_future,
  const std::vector<float> & neighbor_past, const std::vector<float> & neighbor_future,
  const std::vector<float> & static_objects, const std::vector<float> & lanes,
  const std::vector<float> & route_lanes, const int64_t turn_indicator)
{
  namespace fs = std::filesystem;

  fs::create_directories(output_path);

  // Save individual arrays as .npy files (simulating npz format)
  const std::string base_filename = output_path + "/" + token;

  // ego_agent_past (21, 4) - x, y, cos_yaw, sin_yaw
  const int ego_past_shape[] = {static_cast<int>(PAST_TIME_STEPS), 4};
  aoba::SaveArrayAsNumpy(base_filename + "_ego_agent_past.npy", 2, ego_past_shape, ego_past.data());

  // ego_current_state (10,)
  const int ego_current_shape[] = {static_cast<int>(ego_current.size())};
  aoba::SaveArrayAsNumpy(
    base_filename + "_ego_current_state.npy", 1, ego_current_shape, ego_current.data());

  // ego_agent_future (80, 3)
  const int ego_future_shape[] = {static_cast<int>(FUTURE_TIME_STEPS), 3};
  aoba::SaveArrayAsNumpy(
    base_filename + "_ego_agent_future.npy", 2, ego_future_shape, ego_future.data());

  // neighbor_agents_past (32, 21, 11)
  const int neighbor_past_shape[] = {
    static_cast<int>(NEIGHBOR_NUM), static_cast<int>(PAST_TIME_STEPS), 11};
  aoba::SaveArrayAsNumpy(
    base_filename + "_neighbor_agents_past.npy", 3, neighbor_past_shape, neighbor_past.data());

  // neighbor_agents_future (32, 80, 3)
  const int neighbor_future_shape[] = {
    static_cast<int>(NEIGHBOR_NUM), static_cast<int>(FUTURE_TIME_STEPS), 3};
  aoba::SaveArrayAsNumpy(
    base_filename + "_neighbor_agents_future.npy", 3, neighbor_future_shape,
    neighbor_future.data());

  // static_objects (5, 10)
  const int static_shape[] = {static_cast<int>(STATIC_NUM), 10};
  aoba::SaveArrayAsNumpy(
    base_filename + "_static_objects.npy", 2, static_shape, static_objects.data());

  // lanes (70, 20, SEGMENT_POINT_DIM)
  const int lanes_shape[] = {
    static_cast<int>(LANE_NUM), static_cast<int>(LANE_LEN), static_cast<int>(SEGMENT_POINT_DIM)};
  aoba::SaveArrayAsNumpy(base_filename + "_lanes.npy", 3, lanes_shape, lanes.data());

  // route_lanes (25, 20, SEGMENT_POINT_DIM)
  const int route_shape[] = {
    static_cast<int>(ROUTE_NUM), static_cast<int>(ROUTE_LEN), static_cast<int>(SEGMENT_POINT_DIM)};
  aoba::SaveArrayAsNumpy(base_filename + "_route_lanes.npy", 3, route_shape, route_lanes.data());

  // turn_indicator (1,)
  const int turn_shape[] = {1};
  const int turn_indicator_int = static_cast<int>(turn_indicator);
  aoba::SaveArrayAsNumpy(base_filename + "_turn_indicator.npy", 1, turn_shape, &turn_indicator_int);
}

int main(int argc, char ** argv)
{
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

  // Parse optional arguments
  for (int64_t i = 4; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg.find("--step=") == 0) {
      step = std::stoll(arg.substr(7));
    } else if (arg.find("--limit=") == 0) {
      limit = std::stoll(arg.substr(8));
    } else if (arg.find("--min_frames=") == 0) {
      min_frames = std::stoll(arg.substr(13));
    }
  }

  std::cout << "Processing rosbag: " << rosbag_path << std::endl;
  std::cout << "Vector map: " << vector_map_path << std::endl;
  std::cout << "Save directory: " << save_dir << std::endl;
  std::cout << "Step: " << step << ", Limit: " << limit << ", Min frames: " << min_frames
            << std::endl;

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

  // Traffic light map for lane processing
  std::map<lanelet::Id, preprocess::TrafficSignalStamped> traffic_light_id_map;

  rosbag_parser::RosbagParser rosbag_parser(rosbag_path);
  rosbag_parser.create_reader(rosbag_path);

  // Parse messages from specific topics
  std::deque<Odometry> kinematic_states;
  std::deque<AccelWithCovarianceStamped> accelerations;
  std::deque<TrackedObjects> tracked_objects_msgs;
  std::deque<TurnIndicatorsReport> turn_indicators;
  std::vector<LaneletRoute> route_msgs;

  const std::vector<std::string> target_topics = {
    "/localization/kinematic_state", "/localization/acceleration",
    "/perception/object_recognition/tracking/objects", "/planning/mission_planning/route",
    "/vehicle/status/turn_indicators_status"};

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
    }

    parse_count++;
  }

  std::cout << "Parsed " << kinematic_states.size() << " kinematic states" << std::endl;
  std::cout << "Parsed " << accelerations.size() << " acceleration messages" << std::endl;
  std::cout << "Parsed " << tracked_objects_msgs.size() << " tracked objects" << std::endl;
  std::cout << "Parsed " << route_msgs.size() << " route messages" << std::endl;
  std::cout << "Parsed " << turn_indicators.size() << " turn indicator messages" << std::endl;

  if (tracked_objects_msgs.empty()) {
    std::cerr << "No tracked objects found in rosbag" << std::endl;
    return 1;
  }

  // Create sequences based on tracked objects (base topic at 10Hz)
  std::vector<SequenceData> sequences;
  if (!route_msgs.empty()) {
    for (const LaneletRoute & route : route_msgs) {
      sequences.push_back({{}, route});
    }
    // Set route to route handler
    route_handler.setRoute(route_msgs[0]);
  } else {
    sequences.push_back({{}, LaneletRoute()});
  }

  // Process each tracked objects message
  for (int64_t i = 0; i < static_cast<int64_t>(tracked_objects_msgs.size()); ++i) {
    const TrackedObjects & tracking = tracked_objects_msgs[i];

    // Find matching messages
    const Odometry kinematic = get_nearest_msg(kinematic_states, tracking.header.stamp);
    const AccelWithCovarianceStamped accel = get_nearest_msg(accelerations, tracking.header.stamp);
    const TurnIndicatorsReport turn_ind = get_nearest_msg(turn_indicators, tracking.header.stamp);

    const FrameData frame_data{
      parse_timestamp(tracking.header.stamp),
      sequences[0].route,  // Use first route for now
      tracking,
      kinematic,
      accel,
      turn_ind};

    sequences[0].data_list.push_back(frame_data);
  }

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

    // Process frames
    for (int64_t i = PAST_TIME_STEPS; i < n - FUTURE_TIME_STEPS; i += step) {
      const std::string token = std::to_string(seq_id) + "_" + std::to_string(i);

      // Get transformation matrix
      const Eigen::Matrix4f bl2map = get_transform_matrix(seq.data_list[i].kinematic_state);
      const Eigen::Matrix4f map2bl = bl2map.inverse();

      // Create ego sequences
      const std::vector<float> ego_past =
        create_ego_sequence(seq.data_list, i - PAST_TIME_STEPS + 1, PAST_TIME_STEPS, map2bl);
      const std::vector<float> ego_future =
        create_ego_future_sequence(seq.data_list, i + 1, FUTURE_TIME_STEPS, map2bl);

      // Create ego current state
      const EgoState ego_state(
        seq.data_list[i].kinematic_state, seq.data_list[i].acceleration, 2.79f);
      const std::vector<float> ego_current = ego_state.as_array();

      // Process neighbor agents
      const std::vector<float> neighbor_past = process_neighbor_agents(seq.data_list, i, map2bl);
      const std::vector<float> neighbor_future = process_neighbor_future(seq.data_list, i, map2bl);

      // Process lanes and routes
      const Point & ego_pos = seq.data_list[i].kinematic_state.pose.pose.position;
      const std::vector<float> lanes =
        process_lanes(lane_segment_context, ego_pos, map2bl, traffic_light_id_map);
      const std::vector<float> route_lanes = process_route_lanes(
        lane_segment_context, route_handler, ego_pos, map2bl, traffic_light_id_map);

      // Create placeholder data for static objects
      const std::vector<float> static_objects(STATIC_NUM * 10, 0.0f);

      const int64_t turn_indicator = seq.data_list[i].turn_indicator.report;

      // Save data
      save_npz_data(
        save_dir, token, ego_past, ego_current, ego_future, neighbor_past, neighbor_future,
        static_objects, lanes, route_lanes, turn_indicator);

      if (i % 100 == 0) {
        std::cout << "Processed frame " << i << "/" << n << std::endl;
      }
    }
  }

  std::cout << "Data conversion completed!" << std::endl;
}
