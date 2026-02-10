// Copyright 2026 TIER IV, Inc.

#ifndef AUTOWARE__STATIC_FREESPACE_PLANNER__STATIC_FREESPACE_PLANNER_NODE_HPP_
#define AUTOWARE__STATIC_FREESPACE_PLANNER__STATIC_FREESPACE_PLANNER_NODE_HPP_

#include "autoware/static_freespace_planner/debug_publisher.hpp"
#include "autoware/static_freespace_planner/route_index_loader.hpp"
#include "autoware/static_freespace_planner/route_matcher.hpp"
#include "autoware/static_freespace_planner/trajectory_generator.hpp"
#include "autoware/static_freespace_planner/waypoint_loader.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/scenario.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <deque>
#include <memory>
#include <string>
#include <vector>

class TestStaticFreespacePlanner;

namespace autoware::static_freespace_planner
{

using autoware_internal_planning_msgs::msg::Scenario;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Bool;
using std_msgs::msg::String;
using visualization_msgs::msg::Marker;

using TrajectoryPublisher = rclcpp::Publisher<Trajectory>;
using MarkerPublisher = rclcpp::Publisher<Marker>;
using StringPublisher = rclcpp::Publisher<String>;
using BoolPublisher = rclcpp::Publisher<Bool>;
using DiagnosticArrayPublisher = rclcpp::Publisher<DiagnosticArray>;

using LaneletRouteSubscription = rclcpp::Subscription<LaneletRoute>;
using PollingScenarioSubscriber = autoware_utils::InterProcessPollingSubscriber<Scenario>;
using PollingOdometrySubscriber =
  autoware_utils::InterProcessPollingSubscriber<Odometry, autoware_utils::polling_policy::All>;

struct NodeParam
{
  std::string map_path;               // map data root directory path
  double waypoint_search_radius_m;    // waypoint search radius for route matching
  double waypoint_yaw_threshold_rad;  // yaw threshold for route matching
  double update_rate;                 // timer update rate
  double th_arrived_distance_m;       // arrival threshold distance
  double th_stopped_time_sec;         // stopped time threshold
  double th_stopped_velocity_mps;     // stopped velocity threshold
};

class StaticFreespacePlannerNode : public rclcpp::Node
{
public:
  explicit StaticFreespacePlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // components
  std::unique_ptr<RouteIndexLoader> route_loader_;
  std::unique_ptr<WaypointLoader> waypoint_loader_;
  std::unique_ptr<RouteMatcher> route_matcher_;
  std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<DebugPublisher> debug_publisher_;

  // Route definition list (loaded at startup)
  std::vector<RouteIndexLoader::RouteDefinition> route_definitions_;

  // current route and waypoints
  std::vector<WaypointLoader::Waypoint> current_waypoints_;  // selected waypoints for current route
  std::vector<int> available_seqs_;  // available seq numbers in current waypoints
  int current_seq_index_;            // current sequence index
  bool is_completed_ = false;        // completion flag for all sequences

  // ros2 interfaces
  TrajectoryPublisher::SharedPtr trajectory_pub_;
  MarkerPublisher::SharedPtr debug_path_marker_pub_;
  StringPublisher::SharedPtr debug_route_name_pub_;
  BoolPublisher::SharedPtr parking_state_pub_;
  DiagnosticArrayPublisher::SharedPtr diagnostics_pub_;
  LaneletRouteSubscription::SharedPtr route_sub_;
  PollingScenarioSubscriber scenario_sub_{this, "~/input/scenario"};
  PollingOdometrySubscriber odom_sub_{this, "~/input/odometry", rclcpp::QoS{100}};
  rclcpp::TimerBase::SharedPtr timer_;

  // callback functions
  void onRoute(const LaneletRoute::ConstSharedPtr msg);
  void onTimer();

  // main functions
  void loadRoutes();
  void loadWaypoints(const std::string & csv_path);
  void planTrajectory();
  void handleMatchingFailure(const Pose & start_pose, const Pose & goal_pose);
  void publishDiagnostics(const DiagnosticStatus & status);
  void publishCompleted(const bool is_completed);
  void publishStopTrajectory();

  // sequence management
  void updateTargetSeq();        // After arrival determination, proceed to next seq
  bool isCurrentSeqCompleted();  // Determine whether the end of the current seq has been reached
  int getCurrentSeq();           // Get the currently running seq number

  // utility functions
  bool isStopped(
    const std::deque<Odometry::ConstSharedPtr> & odom_buffer, const double th_stopped_velocity_mps);
  bool isActive(Scenario::ConstSharedPtr scenario);
  void updateData();
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void reset();

  // params
  NodeParam node_param_;

  // msgs
  Trajectory trajectory_;
  Scenario::ConstSharedPtr scenario_;
  Odometry::ConstSharedPtr odom_;
  std::deque<Odometry::ConstSharedPtr> odom_buffer_;

  friend class ::TestStaticFreespacePlanner;
};
}  // namespace autoware::static_freespace_planner

#endif  // AUTOWARE__STATIC_FREESPACE_PLANNER__STATIC_FREESPACE_PLANNER_NODE_HPP_
