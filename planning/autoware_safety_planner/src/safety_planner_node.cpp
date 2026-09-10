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

#include "safety_planner_node.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_geometry/ear_clipping.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::safety_planner
{

SafetyPlannerNode::SafetyPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("safety_planner_node", options),
  normal_generator_uuid_(autoware_utils_uuid::generate_uuid()),
  cautious_generator_uuid_(autoware_utils_uuid::generate_uuid()),
  vehicle_info_(vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  param_listener_ =
    std::make_shared<::safety_planner::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  pub_debug_normal_trajectory_ = this->create_publisher<Trajectory>("~/debug/normal/trajectory", 1);
  pub_debug_cautious_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/cautious/trajectory", 1);
  pub_candidate_trajectories_ =
    this->create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);
  pub_debug_marker_ = this->create_publisher<MarkerArray>("~/debug/debug_marker", 1);
  pub_debug_normal_constraints_marker_ =
    this->create_publisher<MarkerArray>("~/debug/normal/constraints", 1);
  pub_debug_cautious_constraints_marker_ =
    this->create_publisher<MarkerArray>("~/debug/cautious/constraints", 1);

  debug_processing_time_detail_pub_ =
    this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  planner_ = std::make_unique<SafetyPlanner>(params_, time_keeper_);
  for (const auto & name : planner_->get_constraint_generator_plugin_names()) {
    constraint_debug_marker_publishers_[name] =
      this->create_publisher<MarkerArray>("~/debug/constraints/" + name, 1);
  }

  const auto planning_freq = rclcpp::Rate(10.0);
  timer_ = rclcpp::create_timer(
    this, get_clock(), planning_freq.period(), std::bind(&SafetyPlannerNode::on_timer, this));
}

bool SafetyPlannerNode::is_data_ready(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  // TODO(odashima): check topic timeout

  if (!input_data.lanelet_map_bin_ptr) {
    notify_waiting("lanelet map");
    return false;
  }
  if (!input_data.route_ptr) {
    notify_waiting("route");
    return false;
  }
  if (!input_data.lanelet_map_bin_ptr) {
    notify_waiting("lanelet map");
    return false;
  }
  if (!input_data.odometry_ptr) {
    notify_waiting("odometry");
    return false;
  }
  if (!input_data.acceleration_ptr) {
    notify_waiting("acceleration");
    return false;
  }
  if (!input_data.steering_ptr) {
    notify_waiting("steering");
    return false;
  }

  return true;
}

SafetyPlannerNode::InputData SafetyPlannerNode::take_data()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  InputData input_data;

  if (const auto msg = route_subscriber_.take_data()) {
    if (!msg->segments.empty()) {
      route_ptr_ = msg;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "input route is empty, ignoring...");
    }
  }
  input_data.route_ptr = route_ptr_;

  if (const auto msg = vector_map_subscriber_.take_data()) {
    lanelet_map_bin_ptr_ = msg;
  }
  input_data.lanelet_map_bin_ptr = lanelet_map_bin_ptr_;

  if (const auto msg = odometry_subscriber_.take_data()) {
    odometry_ptr_ = msg;
  }
  input_data.odometry_ptr = odometry_ptr_;

  if (const auto msg = acceleration_subscriber_.take_data()) {
    acceleration_ptr_ = msg;
  }
  input_data.acceleration_ptr = acceleration_ptr_;

  if (const auto msg = objects_subscriber_.take_data()) {
    predicted_objects_ptr_ = msg;
  }
  input_data.predicted_objects_ptr = predicted_objects_ptr_;

  if (const auto msg = steering_subscriber_.take_data()) {
    steering_ptr_ = msg;
  }
  input_data.steering_ptr = steering_ptr_;

  if (const auto msg = pointcloud_subscriber_.take_data()) {
    obstacle_pointcloud_ptr_ = msg;
  }
  input_data.obstacle_pointcloud_ptr = obstacle_pointcloud_ptr_;

  return input_data;
}

void SafetyPlannerNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto input_data = take_data();
  if (!is_data_ready(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for necessary data to plan trajectories.");
    return;
  }

  if (!update_input(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Failed to update input. Skipping planning cycle.");
    return;
  }

  const auto planned = planner_->plan(input_);
  if (!planned) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "%s. Skipping this cycle.",
      planned.error().c_str());

    publish_trajectories(SafetyPlannerResult{});
    return;
  }
  const auto & result = planned.value();

  {
    autoware_utils_debug::ScopedTimeTrack publish_st("publish_result", *time_keeper_);

    publish_trajectories(result);

    publish_constraint_generator_debug_markers(result.debug.constraint_generator_outputs);
    publish_constraints_markers(result.debug.constraint_generator_outputs);

    publish_planner_debug_trajectories(result.debug);
    publish_debug_markers(result.debug);

    // TODO(odashima): publish planning factors?
    // publish_planning_factors();
  }
}

bool SafetyPlannerNode::update_route_manager(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & current_pose = input_data.odometry_ptr->pose.pose;

  // Rebuild it when the route or the map is replaced, since the old one no longer applies
  const bool needs_create = !input_.route_manager ||
                            route_uuid_of_route_manager_ != input_data.route_ptr->uuid ||
                            map_ptr_of_route_manager_ != input_data.lanelet_map_bin_ptr;

  if (!needs_create) {
    try {
      input_.route_manager = std::move(*input_.route_manager)
                               .update_current_pose(
                                 current_pose, params_.ego_nearest_lanelet.dist_threshold_m,
                                 params_.ego_nearest_lanelet.yaw_threshold_rad);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "update_current_pose threw: %s", e.what());
      input_.route_manager = std::nullopt;
    }
    if (input_.route_manager) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Failed to track current pose on the route. Re-creating RouteManager.");
  }

  input_.route_manager =
    RouteManager::create(*input_data.lanelet_map_bin_ptr, *input_data.route_ptr, current_pose);

  if (!input_.route_manager) {
    route_uuid_of_route_manager_.reset();
    map_ptr_of_route_manager_.reset();
    return false;
  }

  route_uuid_of_route_manager_ = input_data.route_ptr->uuid;
  map_ptr_of_route_manager_ = input_data.lanelet_map_bin_ptr;
  return true;
}

bool SafetyPlannerNode::update_input(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  input_.vehicle_info = vehicle_info_;

  if (!update_route_manager(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Failed to update RouteManager.");
    return false;
  }

  input_.odometry = *input_data.odometry_ptr;
  input_.acceleration = *input_data.acceleration_ptr;
  input_.steering = *input_data.steering_ptr;
  input_.goal_pose = input_data.route_ptr->goal_pose;
  input_.predicted_objects = input_data.predicted_objects_ptr;

  return true;
}

void SafetyPlannerNode::publish_trajectories(const SafetyPlannerResult & result) const
{
  Trajectory ego_only;
  ego_only.header.frame_id = "map";
  ego_only.header.stamp = input_.odometry.header.stamp;
  ego_only.points.emplace_back().pose = input_.odometry.pose.pose;

  CandidateTrajectories candidate_trajectories;
  const auto add = [&](
                     const std::optional<PlannedTrajectory> & planned, const UUID & generator_id,
                     const std::string & generator_name, const auto & debug_pub) {
    const auto & trajectory = planned ? planned->trajectory : ego_only;
    auto & candidate = candidate_trajectories.candidate_trajectories.emplace_back();
    candidate.header = trajectory.header;
    candidate.generator_id = generator_id;
    candidate.points = trajectory.points;
    if (planned) {
      candidate.turn_indicators_command = planned->turn_indicators;
    }
    auto & generator_info = candidate_trajectories.generator_info.emplace_back();
    generator_info.generator_id = generator_id;
    generator_info.generator_name.data = generator_name;

    debug_pub->publish(trajectory);
  };
  add(
    result.normal_trajectory, normal_generator_uuid_, "SafetyPlanner_Normal",
    pub_debug_normal_trajectory_);
  add(
    result.cautious_trajectory, cautious_generator_uuid_, "SafetyPlanner_Cautious",
    pub_debug_cautious_trajectory_);
  pub_candidate_trajectories_->publish(candidate_trajectories);
}

void SafetyPlannerNode::publish_planner_debug_trajectories(const SafetyPlannerResult::Debug & debug)
{
  publish_planner_debug_trajectories("normal", debug.normal);
  publish_planner_debug_trajectories("cautious", debug.cautious);
}

void SafetyPlannerNode::publish_planner_debug_trajectories(
  const std::string & side, const TrajectoryPlannerDebug & debug)
{
  for (const auto & [name, trajectory] : debug.trajectories) {
    auto & pub = planner_debug_trajectory_pubs_[side + "/" + name];
    if (!pub) {
      pub = this->create_publisher<Trajectory>("~/debug/" + side + "/" + name, 1);
    }
    pub->publish(trajectory);
  }
}

void SafetyPlannerNode::publish_debug_markers(const SafetyPlannerResult::Debug & debug) const
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const auto now = this->now();
  MarkerArray marker_array;
  // The planner markers change namespaces between cycles, which would leave the ones that
  // disappeared behind, so clear them first
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);

  // -------------------- the context --------------------
  // current_pose
  {
    auto marker = create_default_marker(
      "map", now, "current_pose", 0, Marker::ARROW, create_marker_scale(2.0, 0.5, 0.5),
      create_marker_color(0.0, 1.0, 0.0, 0.999));
    marker.pose = input_.odometry.pose.pose;
    marker_array.markers.push_back(marker);
  }

  // goal_pose
  {
    auto marker = create_default_marker(
      "map", now, "goal_pose", 0, Marker::ARROW, create_marker_scale(2.0, 0.5, 0.5),
      create_marker_color(1.0, 0.0, 0.0, 0.999));
    marker.pose = input_.goal_pose;
    marker_array.markers.push_back(marker);
  }

  // reference_path
  {
    auto marker = create_default_marker(
      "map", now, "reference_path", 0, Marker::LINE_STRIP, create_marker_scale(0.2, 0.0, 0.0),
      create_marker_color(0.0, 0.5, 1.0, 0.999));
    constexpr double MARKER_INTERVAL_M = 1.0;
    const auto & reference_path = debug.reference_path;
    for (double s = 0.0; s < reference_path.length(); s += MARKER_INTERVAL_M) {
      marker.points.push_back(reference_path.compute(s).point.pose.position);
    }
    if (reference_path.length() > 0.0) {
      marker.points.push_back(reference_path.compute(reference_path.length()).point.pose.position);
    }
    if (marker.points.size() >= 2) {
      marker_array.markers.push_back(marker);
    }
  }

  // -------------------- the planner --------------------
  const auto append_planner_markers =
    [&](const std::string & side, const TrajectoryPlannerDebug & planner_debug) {
      for (const auto & [name, markers] : planner_debug.markers) {
        for (auto marker : markers.markers) {
          marker.ns = side + "/" + marker.ns;
          marker_array.markers.push_back(std::move(marker));
        }
      }
    };
  append_planner_markers("normal", debug.normal);
  append_planner_markers("cautious", debug.cautious);

  pub_debug_marker_->publish(marker_array);
}

void SafetyPlannerNode::publish_constraint_generator_debug_markers(
  const std::map<std::string, ConstraintGeneratorOutput> & constraints) const
{
  for (const auto & [plugin_name, output] : constraints) {
    if (!output.debug_markers) {
      continue;
    }
    const auto it = constraint_debug_marker_publishers_.find(plugin_name);
    if (it == constraint_debug_marker_publishers_.end()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "no debug marker publisher for plugin '%s'", plugin_name.c_str());
      continue;
    }
    it->second->publish(*output.debug_markers);
  }
}

void SafetyPlannerNode::publish_constraints_markers(
  const std::map<std::string, ConstraintGeneratorOutput> & constraints) const
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const auto now = this->now();
  const double z = input_.odometry.pose.pose.position.z;

  MarkerArray normal_markers;
  MarkerArray cautious_markers;
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  normal_markers.markers.push_back(delete_all);
  cautious_markers.markers.push_back(delete_all);

  const auto to_point = [](const Point2d & p, const double z) {
    geometry_msgs::msg::Point q;
    q.x = p.x();
    q.y = p.y();
    q.z = z;
    return q;
  };

  const auto hard_constraint_color = create_marker_color(1.0, 0.0, 0.0, 0.999);
  const auto soft_constraint_color = create_marker_color(1.0, 0.8, 0.0, 0.999);

  const auto boundary_marker = [&](
                                 const Boundary & boundary, const Constraint & constraint,
                                 const std::string & ns, const int id) {
    const bool is_hard = constraint.hardness == Hardness::HARD;
    auto marker = create_default_marker(
      "map", now, ns, id, Marker::LINE_STRIP, create_marker_scale(is_hard ? 0.3 : 0.15, 0.0, 0.0),
      is_hard ? hard_constraint_color : soft_constraint_color);
    for (const auto & p : boundary.polyline) {
      marker.points.push_back(to_point(p, z));
    }
    return marker;
  };

  // Time-dependent payloads use z as the time axis: z = ego z + t * TIME_Z_PER_SEC
  constexpr double TIME_Z_PER_SEC = 1.0;  // [m/s]

  // The occupancy is drawn as a tube: the ring of every timed sample and the edges joining the
  // matching vertices of consecutive samples
  const auto keep_out_marker = [&](
                                 const KeepOut & keep_out, const Constraint & constraint,
                                 const std::string & ns, const int id) {
    const bool is_hard = constraint.hardness == Hardness::HARD;
    auto marker = create_default_marker(
      "map", now, ns, id, Marker::LINE_LIST, create_marker_scale(0.1, 0.0, 0.0),
      is_hard ? hard_constraint_color : soft_constraint_color);

    // (time, ring in the map frame) per sample
    std::vector<std::pair<double, std::vector<Point2d>>> samples;
    if (const auto * body = std::get_if<RigidBody>(&keep_out.occupancy)) {
      for (const auto & wp : body->waypoints) {
        const double c = std::cos(wp.pose.yaw);
        const double sn = std::sin(wp.pose.yaw);
        std::vector<Point2d> ring;
        for (const auto & v : body->shape.outer()) {
          ring.emplace_back(
            wp.pose.position.x() + c * v.x() - sn * v.y(),
            wp.pose.position.y() + sn * v.x() + c * v.y());
        }
        samples.emplace_back(wp.t, std::move(ring));
      }
    } else if (const auto * seq = std::get_if<TimedPolygonSequence>(&keep_out.occupancy)) {
      for (const auto & tp : seq->polygons) {
        samples.emplace_back(
          tp.t, std::vector<Point2d>(tp.polygon.outer().begin(), tp.polygon.outer().end()));
      }
    }

    for (std::size_t k = 0; k < samples.size(); ++k) {
      const auto & [t, ring] = samples[k];
      const double zk = z + t * TIME_Z_PER_SEC;
      for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
        marker.points.push_back(to_point(ring[i], zk));
        marker.points.push_back(to_point(ring[i + 1], zk));
      }
      if (k == 0) {
        continue;
      }
      const auto & [t_prev, ring_prev] = samples[k - 1];
      if (ring_prev.size() != ring.size()) {  // TimedPolygonSequence may change the vertex count
        continue;
      }
      const double z_prev = z + t_prev * TIME_Z_PER_SEC;
      for (std::size_t i = 0; i < ring.size(); ++i) {
        marker.points.push_back(to_point(ring_prev[i], z_prev));
        marker.points.push_back(to_point(ring[i], zk));
      }
    }
    return marker;
  };

  const double horizon_s = params_.trajectory_horizon_s;
  const auto gate_marker =
    [&](const Gate & gate, const Constraint & constraint, const std::string & ns, const int id) {
      const bool is_hard = constraint.hardness == Hardness::HARD;
      const auto & a = gate.line.first;
      const auto & b = gate.line.second;
      const double length = std::hypot(b.x() - a.x(), b.y() - a.y());
      const double z0 = z + gate.time.t0 * TIME_Z_PER_SEC;
      const double z1 = z + std::min(gate.time.t1, horizon_s) * TIME_Z_PER_SEC;
      auto color = is_hard ? hard_constraint_color : soft_constraint_color;
      color.a = 0.5;
      auto marker = create_default_marker(
        "map", now, ns, id, Marker::CUBE, create_marker_scale(0.1, length, z1 - z0), color);
      marker.pose.position.x = 0.5 * (a.x() + b.x());
      marker.pose.position.y = 0.5 * (a.y() + b.y());
      marker.pose.position.z = 0.5 * (z0 + z1);
      // The cube's y axis lies along the segment, so the yaw is that of the normal
      marker.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
        std::atan2(b.y() - a.y(), b.x() - a.x()) - M_PI_2);
      return marker;
    };

  // Same split as the planner: normal = DEFINITE only, cautious = DEFINITE + POSSIBLE
  const auto add = [&](Marker marker, const Constraint & constraint) {
    if (constraint.certainty == Certainty::DEFINITE) {
      normal_markers.markers.push_back(marker);
    }
    cautious_markers.markers.push_back(std::move(marker));
  };

  // A ScalarBound has no geometry of its own beyond the region, so the numbers go into text:
  // "v <= 5.00 [m/s]" at the centroid of the region, or, for a bound that holds everywhere, one
  // line of a list floating above the ego
  const auto bound_text = [](const ScalarBound & bound) {
    const char * symbol = "";
    const char * unit = "";
    switch (bound.quantity) {
      case BoundedQuantity::VELOCITY:
        symbol = "v";
        unit = "[m/s]";
        break;
      case BoundedQuantity::LON_ACCEL:
        symbol = "a";
        unit = "[m/s^2]";
        break;
      case BoundedQuantity::LON_JERK:
        symbol = "|j|";
        unit = "[m/s^3]";
        break;
      case BoundedQuantity::LAT_ACCEL:
        symbol = "|a_lat|";
        unit = "[m/s^2]";
        break;
      case BoundedQuantity::CURVATURE:
        symbol = "|k|";
        unit = "[1/m]";
        break;
      case BoundedQuantity::STEER_ANGLE:
        symbol = "|d|";
        unit = "[rad]";
        break;
      case BoundedQuantity::STEER_RATE:
        symbol = "|d'|";
        unit = "[rad/s]";
        break;
    }
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    if (std::isfinite(bound.min)) {
      ss << bound.min << " <= ";
    }
    ss << symbol;
    if (std::isfinite(bound.max)) {
      ss << " <= " << bound.max;
    }
    ss << " " << unit;
    return ss.str();
  };
  const auto text_marker = [&](const std::string & ns, const int id, const std::string & text) {
    auto marker = create_default_marker(
      "map", now, ns, id, Marker::TEXT_VIEW_FACING, create_marker_scale(0.0, 0.0, 0.5),
      create_marker_color(1.0, 1.0, 1.0, 0.999));
    marker.text = text;
    return marker;
  };

  // [side][hardness], hardness = 0 for HARD and 1 for SOFT
  std::string normal_global_bounds[2];
  std::string cautious_global_bounds[2];
  for (const auto & [plugin_name, output] : constraints) {
    int id = 0;
    for (const auto & constraint : output.constraints) {
      const bool is_hard = constraint.hardness == Hardness::HARD;
      if (const auto * boundary = std::get_if<Boundary>(&constraint.payload)) {
        add(boundary_marker(*boundary, constraint, plugin_name + "/boundary", id++), constraint);
      } else if (const auto * keep_out = std::get_if<KeepOut>(&constraint.payload)) {
        auto marker = keep_out_marker(*keep_out, constraint, plugin_name + "/keep_out", id++);
        // rviz warns on a line marker without points (an occupancy that has no sample)
        if (!marker.points.empty()) {
          add(std::move(marker), constraint);
        }
      } else if (const auto * gate = std::get_if<Gate>(&constraint.payload)) {
        add(gate_marker(*gate, constraint, plugin_name + "/gate", id++), constraint);
      } else if (const auto * bound = std::get_if<ScalarBound>(&constraint.payload)) {
        const std::string line =
          plugin_name + (is_hard ? "[hard]: " : "[soft]: ") + bound_text(*bound);
        if (!bound->region) {
          if (constraint.certainty == Certainty::DEFINITE) {
            normal_global_bounds[is_hard ? 0 : 1] += line + "\n";
          }
          cautious_global_bounds[is_hard ? 0 : 1] += line + "\n";
          continue;
        }
        const auto & ring = bound->region->outer();
        if (ring.empty()) {
          continue;
        }
        // A filled face rather than an outline, which would be lost among the Boundary lines.
        // Lanelet regions are concave, so a fan from the centroid would not do
        auto fill_color = is_hard ? hard_constraint_color : soft_constraint_color;
        fill_color.a = 0.3;
        auto fill = create_default_marker(
          "map", now, plugin_name + "/scalar_bound", id, Marker::TRIANGLE_LIST,
          create_marker_scale(1.0, 1.0, 1.0), fill_color);
        for (const auto & triangle : autoware_utils_geometry::triangulate(*bound->region)) {
          for (std::size_t k = 0; k < 3; ++k) {
            fill.points.push_back(to_point(triangle.outer()[2 - k], z));
          }
        }
        Point2d centroid{0.0, 0.0};
        for (const auto & p : ring) {
          centroid += p;
        }
        centroid /= static_cast<double>(ring.size());
        auto text = text_marker(plugin_name + "/scalar_bound_text", id, line);
        text.pose.position = to_point(centroid, z + 0.5);
        add(std::move(fill), constraint);
        add(std::move(text), constraint);
        ++id;
      }
    }
  }
  // The lists follow the ego so the values can be watched while driving; the hard one sits above
  // the soft one
  const auto add_global_bounds = [&](MarkerArray & markers, const std::string(&texts)[2]) {
    for (int hardness = 0; hardness < 2; ++hardness) {
      if (texts[hardness].empty()) {
        continue;
      }
      auto marker = text_marker("scalar_bound_global", hardness, texts[hardness]);
      // Attached to base_link rather than placed at the odometry of this cycle: rviz then moves
      // it with the TF at its own rate, instead of in 10 Hz steps that jitter against a view
      // following the ego. Stamp 0 = the latest transform
      marker.header.frame_id = "base_link";
      marker.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
      marker.pose.position.z = hardness == 0 ? 4.0 : 3.0;
      markers.markers.push_back(marker);
    }
  };
  add_global_bounds(normal_markers, normal_global_bounds);
  add_global_bounds(cautious_markers, cautious_global_bounds);

  pub_debug_normal_constraints_marker_->publish(normal_markers);
  pub_debug_cautious_constraints_marker_->publish(cautious_markers);
}

}  // namespace autoware::safety_planner

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::safety_planner::SafetyPlannerNode)
