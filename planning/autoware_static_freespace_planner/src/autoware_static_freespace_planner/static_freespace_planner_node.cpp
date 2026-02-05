// Copyright 2026 TIER IV, Inc.

#include "autoware/static_freespace_planner/static_freespace_planner_node.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>

#include <algorithm>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::static_freespace_planner
{
StaticFreespacePlannerNode::StaticFreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("static_freespace_planner", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.map_path = declare_parameter<std::string>("static_map_path", "");
    // Add a static path after the obtained map path
    auto static_path = p.map_path + "/static_path";
    // Check if the static_path directory exists
    if (!std::filesystem::exists(static_path)) {
      // output fatal error and throw exception
      RCLCPP_FATAL(
        get_logger(), "static_path directory not found. Expected static paths in directory: %s",
        static_path.c_str());
      throw std::runtime_error("static_path not found");
    }

    p.waypoint_search_radius_m = declare_parameter<double>("waypoint_search_radius_m", 2.0);
    p.waypoint_yaw_threshold_rad = declare_parameter<double>("waypoint_yaw_threshold_rad", 0.26);
    p.update_rate = declare_parameter<double>("update_rate", 10.0);
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m", 1.0);
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec", 1.0);
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps", 0.01);
  }

  // Subscribers
  route_sub_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&StaticFreespacePlannerNode::onRoute, this, _1));

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
    debug_path_marker_pub_ = create_publisher<Marker>("~/debug/path_marker", qos);
    debug_route_name_pub_ = create_publisher<String>("~/debug/route_name", qos);
    parking_state_pub_ = create_publisher<Bool>("~/output/is_completed", qos);
    diagnostics_pub_ = create_publisher<DiagnosticArray>("~/output/diagnostics", qos);
  }

  // Timer
  {
    const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&StaticFreespacePlannerNode::onTimer, this));
  }

  // Components
  route_loader_ = std::make_unique<RouteIndexLoader>(node_param_.map_path);
  waypoint_loader_ = std::make_unique<WaypointLoader>();
  trajectory_generator_ = std::make_unique<TrajectoryGenerator>();
  debug_publisher_ =
    std::make_unique<DebugPublisher>(this, debug_path_marker_pub_, debug_route_name_pub_);

  // Load route definitions
  loadRoutes();

  // Create RouteMatcher
  route_matcher_ = std::make_unique<RouteMatcher>(
    route_definitions_, node_param_.waypoint_search_radius_m,
    node_param_.waypoint_yaw_threshold_rad);
}

void StaticFreespacePlannerNode::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  const auto lanelet_route = msg;

  // Update odometry data
  updateData();

  // Check if the parking scenario is active
  if (!isActive(scenario_)) {
    reset();
    return;
  }

  // Find matching route definition
  auto route_definition =
    route_matcher_->findMatchingRoute(lanelet_route->start_pose, lanelet_route->goal_pose);
  if (!route_definition) {
    RCLCPP_ERROR(get_logger(), "Failed to find matching route definition.");
    handleMatchingFailure(lanelet_route->start_pose, lanelet_route->goal_pose);
    return;
  }

  // Load waypoints for the matched route
  loadWaypoints(route_definition->csv_path);

  // Plan initial trajectory
  planTrajectory();

  // Reset completion flag
  publishCompleted(false);

  // Publish debug information
  debug_publisher_->publishPathMarker(current_waypoints_, "map");
  debug_publisher_->publishRouteName(route_definition->name);
}

void StaticFreespacePlannerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;

  odom_buffer_.push_back(msg);

  // Delete old data in buffer
  size_t max_size = odom_buffer_.size();
  while (max_size-- > 0) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);

    if (time_diff.seconds() < node_param_.th_stopped_time_sec) {
      break;
    }

    odom_buffer_.pop_front();
  }
}

void StaticFreespacePlannerNode::updateData()
{
  auto msgs = odom_sub_.take_data();
  for (const auto & msg : msgs) {
    onOdometry(msg);
  }
}

void StaticFreespacePlannerNode::onTimer()
{
  updateData();

  scenario_ = scenario_sub_.take_data();

  // Check if the parking scenario is active
  if (!isActive(scenario_)) {
    reset();
    return;
  }

  // If planning is completed, publish stop trajectory
  if (is_completed_) {
    publishStopTrajectory();
    return;
  }

  // If current trajectory completed, update to next seq
  if (isCurrentSeqCompleted()) {
    updateTargetSeq();
  }

  return;
}

bool StaticFreespacePlannerNode::isActive(Scenario::ConstSharedPtr scenario)
{
  if (!scenario) return false;

  const auto & s = scenario->activating_scenarios;
  return std::find(std::begin(s), std::end(s), Scenario::PARKING) != std::end(s);
}

void StaticFreespacePlannerNode::loadRoutes()
{
  route_definitions_ = route_loader_->loadRouteDefinitions();

  if (route_definitions_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "No route definition files found in the specified map path: %s/static_path/. "
      "Please ensure that CSV files are present.",
      node_param_.map_path.c_str());
  }

  return;
}

void StaticFreespacePlannerNode::loadWaypoints(const std::string & csv_path)
{
  current_waypoints_ = waypoint_loader_->loadWaypoints(csv_path);
  available_seqs_ = waypoint_loader_->getAvailableSeqs(csv_path);
  current_seq_index_ = 0;
}

void StaticFreespacePlannerNode::handleMatchingFailure(
  const Pose & start_pose, const Pose & goal_pose)
{
  // Output error log
  RCLCPP_ERROR(
    get_logger(),
    "No matching route found for requested start/goal poses. "
    "start: (%.2f, %.2f), goal: (%.2f, %.2f), search_radius: %.2f m",
    start_pose.position.x, start_pose.position.y, goal_pose.position.x, goal_pose.position.y,
    node_param_.waypoint_search_radius_m);

  // Publish stop trajectory
  publishStopTrajectory();

  // Create MRM diagnostic message
  DiagnosticStatus diag;
  diag.level = DiagnosticStatus::ERROR;
  diag.name = "static_freespace_planner";
  diag.message = "Route matching failed - No suitable route found";
  diag.hardware_id = "planning";

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "error_type";
  kv.value = "NO_MATCHING_ROUTE";
  diag.values.push_back(kv);

  // Publish diagnostic message
  publishDiagnostics(diag);
}

void StaticFreespacePlannerNode::publishDiagnostics(const DiagnosticStatus & status)
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->get_clock()->now();
  diag_array.status.push_back(status);
  diagnostics_pub_->publish(diag_array);
}

void StaticFreespacePlannerNode::publishCompleted(const bool is_completed)
{
  is_completed_ = is_completed;
  std_msgs::msg::Bool is_completed_msg;
  is_completed_msg.data = is_completed_;
  parking_state_pub_->publish(is_completed_msg);
}

void StaticFreespacePlannerNode::publishStopTrajectory()
{
  PoseStamped current_pose;
  current_pose.pose = odom_->pose.pose;
  current_pose.header = odom_->header;

  auto stop_traj = trajectory_generator_->createStopTrajectory(current_pose, get_clock());
  trajectory_pub_->publish(stop_traj);
}

void StaticFreespacePlannerNode::planTrajectory()
{
  PoseStamped current_pose;
  current_pose.pose = odom_->pose.pose;
  current_pose.header = odom_->header;

  trajectory_ = trajectory_generator_->createTrajectoryForSeq(
    current_waypoints_, current_seq_index_, current_pose);

  trajectory_pub_->publish(trajectory_);
}

void StaticFreespacePlannerNode::reset()
{
  trajectory_ = Trajectory();

  publishCompleted(false);

  return;
}

bool StaticFreespacePlannerNode::isCurrentSeqCompleted()
{
  if (current_waypoints_.empty()) {
    return false;
  }

  // Return false if vehicle is not stopped
  if (!isStopped(odom_buffer_, node_param_.th_stopped_velocity_mps)) {
    return false;
  }

  // Get the last trajectory point
  auto last_trajectory = trajectory_.points.back();

  // Calculate distance to the last waypoint of the current sequence
  double distance = autoware_utils::calc_distance2d(odom_->pose.pose, last_trajectory.pose);

  // Return true if within arrival threshold
  return distance < node_param_.th_arrived_distance_m;
}

bool StaticFreespacePlannerNode::isStopped(
  const std::deque<Odometry::ConstSharedPtr> & odom_buffer, const double th_stopped_velocity_mps)
{
  const double th_stopped_velocity_sq = th_stopped_velocity_mps * th_stopped_velocity_mps;
  for (const auto & odom : odom_buffer) {
    const auto & lin = odom->twist.twist.linear;
    const double velocity_sq = lin.x * lin.x + lin.y * lin.y + lin.z * lin.z;
    if (velocity_sq > th_stopped_velocity_sq) {
      return false;
    }
  }
  return true;
}

void StaticFreespacePlannerNode::updateTargetSeq()
{
  current_seq_index_++;
  // Check if the new seq index is available
  const auto it = std::find(available_seqs_.begin(), available_seqs_.end(), current_seq_index_);

  if (it != available_seqs_.end()) {
    planTrajectory();
  } else {
    publishCompleted(true);
  }
}

}  // namespace autoware::static_freespace_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::static_freespace_planner::StaticFreespacePlannerNode)
