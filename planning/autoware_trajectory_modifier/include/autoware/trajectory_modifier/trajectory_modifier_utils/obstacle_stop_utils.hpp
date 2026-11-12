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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__OBSTACLE_STOP_UTILS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__OBSTACLE_STOP_UTILS_HPP_

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/point_types/types.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/geometry/index/rtree.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_hash.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::utils::obstacle_stop
{
using autoware::point_types::PointCloudClassification;
using autoware::point_types::PointXYZCPE;
using sensor_msgs::msg::PointCloud2;
using PointCloud = pcl::PointCloud<PointXYZCPE>;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

enum class ObjectType : uint8_t {
  UNKNOWN = 0,
  CAR,
  TRUCK,
  BUS,
  TRAILER,
  MOTORCYCLE,
  BICYCLE,
  PEDESTRIAN,
  ANIMAL,
  HAZARD,
  FLAT_SURFACE,
  STRUCTURE,
  VEGETATION,
  NOISE,
};

inline static const std::unordered_map<std::string, ObjectType> string_to_object_type = {
  {"unknown", ObjectType::UNKNOWN},
  {"car", ObjectType::CAR},
  {"truck", ObjectType::TRUCK},
  {"bus", ObjectType::BUS},
  {"trailer", ObjectType::TRAILER},
  {"motorcycle", ObjectType::MOTORCYCLE},
  {"bicycle", ObjectType::BICYCLE},
  {"pedestrian", ObjectType::PEDESTRIAN},
  {"animal", ObjectType::ANIMAL},
  {"hazard", ObjectType::HAZARD},
  {"flat_surface", ObjectType::FLAT_SURFACE},
  {"structure", ObjectType::STRUCTURE},
  {"vegetation", ObjectType::VEGETATION},
  {"noise", ObjectType::NOISE}};

inline static const std::unordered_map<uint8_t, ObjectType> classification_to_object_type = {
  {ObjectClassification::UNKNOWN, ObjectType::UNKNOWN},
  {ObjectClassification::CAR, ObjectType::CAR},
  {ObjectClassification::TRUCK, ObjectType::TRUCK},
  {ObjectClassification::BUS, ObjectType::BUS},
  {ObjectClassification::TRAILER, ObjectType::TRAILER},
  {ObjectClassification::MOTORCYCLE, ObjectType::MOTORCYCLE},
  {ObjectClassification::BICYCLE, ObjectType::BICYCLE},
  {ObjectClassification::PEDESTRIAN, ObjectType::PEDESTRIAN},
  {ObjectClassification::ANIMAL, ObjectType::ANIMAL},
  {ObjectClassification::HAZARD, ObjectType::HAZARD}};

inline static const std::unordered_map<PointCloudClassification, ObjectType>
  pcd_class_to_object_type = {
    {PointCloudClassification::CAR, ObjectType::CAR},
    {PointCloudClassification::TRUCK, ObjectType::TRUCK},
    {PointCloudClassification::BUS, ObjectType::BUS},
    {PointCloudClassification::MOTORCYCLE, ObjectType::MOTORCYCLE},
    {PointCloudClassification::BICYCLE, ObjectType::BICYCLE},
    {PointCloudClassification::PEDESTRIAN, ObjectType::PEDESTRIAN},
    {PointCloudClassification::ANIMAL, ObjectType::ANIMAL},
    {PointCloudClassification::HAZARD, ObjectType::HAZARD},
    {PointCloudClassification::FLAT_SURFACE, ObjectType::FLAT_SURFACE},
    {PointCloudClassification::STRUCTURE, ObjectType::STRUCTURE},
    {PointCloudClassification::VEGETATION, ObjectType::VEGETATION},
    {PointCloudClassification::NOISE, ObjectType::NOISE},
    {PointCloudClassification::INVALID, ObjectType::UNKNOWN}};

using ObjectDecelMap = std::unordered_map<ObjectType, double>;

struct LateralMargin
{
  double normal{0.0};
  double low_velocity{0.0};
};
using LateralMarginMap = std::unordered_map<ObjectType, LateralMargin>;

inline ObjectType to_object_type(const PredictedObject & object)
{
  const auto label =
    object.classification.empty()
      ? ObjectClassification::UNKNOWN
      : autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  const auto it = classification_to_object_type.find(label);
  return it != classification_to_object_type.end() ? it->second : ObjectType::UNKNOWN;
}

inline ObjectType to_object_type(const PointCloudClassification classification)
{
  const auto it = pcd_class_to_object_type.find(classification);
  return it != pcd_class_to_object_type.end() ? it->second : ObjectType::UNKNOWN;
}

inline LateralMargin get_lateral_margin(const LateralMarginMap & map, const ObjectType type)
{
  if (const auto it = map.find(type); it != map.end()) {
    return it->second;
  }
  if (const auto it = map.find(ObjectType::UNKNOWN); it != map.end()) {
    return it->second;
  }
  return LateralMargin{0.0, 0.0};
}

struct CollisionPoint
{
  size_t target_source_index;  // index into target_objects for object collisions, or into the
                               // pointcloud for pcd collisions
  geometry_msgs::msg::Point point;
  double arc_length;
  double obstacle_lon_vel;
  bool is_active{false};
  bool is_dynamic{false};
  rclcpp::Time first_seen;
  rclcpp::Time last_seen;

  /**
   * @brief Construct a collision sample from a point and its arc length along the reference path.
   * @param point Collision position in map frame.
   * @param arc_length Signed arc length from the start of the trajectory to the collision point.
   * @param is_dynamic Whether this collision is dynamic.
   */
  CollisionPoint(
    const size_t target_source_index, const geometry_msgs::msg::Point & point,
    const double arc_length, const double obstacle_lon_vel, const bool is_dynamic = false)
  : target_source_index(target_source_index),
    point(point),
    arc_length(arc_length),
    obstacle_lon_vel(obstacle_lon_vel),
    is_dynamic(is_dynamic)
  {
  }

  /**
   * @brief Copy a collision point and attach timing / activation state (e.g. for hysteresis).
   * @param collision_point Source geometry and arc length.
   * @param first_seen Time associated with first detection of this collision state.
   * @param last_seen Time associated with last detection of this collision state.
   * @param active Whether this collision is currently considered active.
   */
  CollisionPoint(
    const CollisionPoint & collision_point, const rclcpp::Time & first_seen,
    const rclcpp::Time & last_seen, const bool active)
  : target_source_index(collision_point.target_source_index),
    point(collision_point.point),
    arc_length(collision_point.arc_length),
    obstacle_lon_vel(collision_point.obstacle_lon_vel),
    is_active(active),
    is_dynamic(collision_point.is_dynamic),
    first_seen(first_seen),
    last_seen(last_seen)
  {
  }
};

/// Ego vehicle footprint sampled along the detection trajectory.
/// Queried as an OBB using TrajectoryShape extents plus a class-specific lateral margin.
struct EgoFootprint
{
  geometry_msgs::msg::Pose pose;
  size_t traj_index{0};    ///< nearest original trajectory index
  double arc_length{0.0};  ///< arc length from trajectory start to this pose (baselink)
  double lon_vel{0.0};     ///< target longitudinal velocity along trajectory at this pose
};

using FootprintNode = std::pair<autoware_utils_geometry::Box2d, size_t>;
using FootprintRtree =
  boost::geometry::index::rtree<FootprintNode, boost::geometry::index::rstar<16>>;

struct TrajectoryShape
{
  autoware_utils_geometry::Box2d bounding_box;
  double trajectory_length{0.0};
  double forward_traj_length{0.0};
  std::vector<EgoFootprint> footprints;
  FootprintRtree rtree;
  double ego_half_width{0.0};
  double ego_front_offset{0.0};  ///< vehicle_info.max_longitudinal_offset_m
  double ego_back_offset{0.0};   ///< -vehicle_info.min_longitudinal_offset_m

  [[nodiscard]] double ego_arc_length() const { return trajectory_length - forward_traj_length; }
};

/// Longitudinal state of a target object relative to the ego path.
struct ObjectRssState
{
  ObjectType type{ObjectType::UNKNOWN};
  geometry_msgs::msg::Point nearest_point;  ///< object polygon vertex nearest along the path
  double arc_length{0.0};         ///< arc length from trajectory start to nearest_point [m]
  double gap{0.0};                ///< distance from ego front to nearest_point, >= 0 [m]
  double lon_vel{0.0};            ///< object velocity projected on the path direction [m/s]
  double stopping_distance{0.0};  ///< object braking distance, 0 if not moving forward [m]
  bool is_dynamic{false};         ///< |lon_vel| > stopped velocity threshold
};

struct TargetObject
{
  PredictedObject object;
  Polygon2d polygon;
  Polygon2d intersection_polygon;
  Polygon2d ego_footprint;
  std::vector<geometry_msgs::msg::Pose> predicted_path;
  bool is_safe{false};
  double distance_from_ego{0.0};
  double safe_distance{0.0};
  ObjectRssState rss_state;

  explicit TargetObject(const PredictedObject & object) : object(object) {}
};
using TargetObjects = std::vector<TargetObject>;

struct QueryResult
{
  size_t index;
  Polygon2d ego_footprint;
  Polygon2d intersection_polygon;
};

struct DebugData
{
  PointCloud2::SharedPtr filtered_points;
  PredictedObjects filtered_objects;
  TargetObjects target_objects;
  TrajectoryShape trajectory_shape;
  std::vector<geometry_msgs::msg::Point> target_pcd_points;
  std::optional<PredictedObject> colliding_object;
  double ego_z = 0.0;  // cached for marker placement during publish_debug_data
};

/**
 * @brief Trim the trajectory after the first zero-velocity point and remove overlapping duplicate
 * points.
 * @param[in,out] trajectory_points Trajectory to shorten and deduplicate in place.
 */
void trim_trajectory_and_remove_duplicates(TrajectoryPoints & trajectory_points);

/**
 * @brief Build an R-tree of ego footprints along the obstacle-detection portion of the trajectory.
 * @details Detection length combines stopping-distance logic with the path ahead of the ego.
 * Footprints are sampled with a minimum spacing of 0.1 m and interpolated to a maximum spacing of
 * 1.0 m. Each entry stores the pose, nearest original trajectory index, and arc length from the
 * trajectory start. Vehicle extents are stored without extra lateral margin; class-specific
 * expansion is applied at query time.
 * @return TrajectoryShape with footprints, rtree, bounding box, lengths, and ego extents populated.
 */
TrajectoryShape build_trajectory_footprint_index(
  const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double ego_vel,
  const double ego_accel, const double decel, const double jerk, const double stop_margin,
  const double time_delay = 0.0);

/**
 * @brief Find the nearest ego footprint whose expanded OBB contains the given point.
 * @details Coarse R-tree AABB query (inflated by `lat_margin`), then a precise vehicle-frame
 * OBB test: x in [-ego_back_offset, ego_front_offset], |y| <= ego_half_width + lat_margin.
 * @return Overlapping footprint index and Ego OBB (including `lat_margin`) transformed into the
 * nearest hit pose frame, or nullopt if none contain the point.
 */
std::optional<QueryResult> query_overlapping_footprint(
  const TrajectoryShape & shape, const Point2d & point, const LateralMargin lat_margin,
  const double traj_low_velocity_th);

/**
 * @brief Find the nearest ego footprint that intersects the given object polygon.
 * @details Coarse R-tree AABB query (inflated by `lat_margin`), then
 * `autoware_utils_geometry::sat::intersects` in the footprint frame. The ego OBB (including
 * `lat_margin`) is built once per query; each candidate only inverse-transforms the object.
 * @return Overlapping footprint index and Ego OBB (including `lat_margin`) transformed into the
 * nearest hit pose frame, or nullopt if none intersect.
 */
std::optional<QueryResult> query_overlapping_footprint(
  const TrajectoryShape & shape, const Polygon2d & polygon, const LateralMargin lat_margin,
  const double traj_low_velocity_th);

/**
 * @brief Find the closest point-cloud obstacle that overlaps an ego footprint.
 * @details Each point is queried against `trajectory_shape.rtree` with the lateral margin for its
 * class. The collision is the inlier with minimum arc length (footprint arc length plus
 * longitudinal offset in the vehicle frame). All inlier points are appended to `target_pcd_points`.
 * @param trajectory_shape Ego footprint index from build_trajectory_footprint_index().
 * @param pointcloud Input points in the same frame as the trajectory.
 * @param lateral_margin_map Per-class lateral expansion of the ego footprint [m].
 * @param traj_low_velocity_th Low velocity threshold for the trajectory [m/s].
 * @param[out] target_pcd_points Points that overlapped at least one footprint.
 * @return Nearest collision by arc length, or nullopt if the cloud is empty or none intersect.
 */
std::optional<CollisionPoint> get_nearest_pcd_collision(
  const TrajectoryShape & trajectory_shape, const PointCloud::Ptr & pointcloud,
  const LateralMarginMap & lateral_margin_map, const double traj_low_velocity_th,
  std::vector<geometry_msgs::msg::Point> & target_pcd_points);

/**
 * @brief Braking distance of an object moving along the ego path.
 * @param type Object type used to look up the braking deceleration.
 * @param lon_vel Object longitudinal velocity along the ego path [m/s].
 * @param object_decel_map Braking magnitude [m/s^2] per object type.
 * @param stopped_vel_th Objects with `lon_vel` below this [m/s] have zero braking distance.
 * @return Braking distance [m], 0 if the object is not moving forward or its type has no
 * deceleration.
 */
double get_object_stopping_distance(
  const ObjectType type, const double lon_vel, const ObjectDecelMap & object_decel_map,
  const double stopped_vel_th);

/**
 * @brief Evaluate the longitudinal state of the input target object.
 * @details Finds the nearest intersection point of the target object with the trajectory and
 * calculates its arc length, gap from ego front, and longitudinal velocity of the object along the
 * trajectory at the intersection point.
 * @param target_object Target object to evaluate.
 * @param trajectory_points Reference path used to find the nearest intersection point.
 * @param trajectory_shape For ego footprint used to calculate the ego front arc length.
 * @param object_decel_map Braking magnitude [m/s^2] per object type.
 * @param stopped_vel_th Objects with |lon_vel| below this [m/s] are considered static.
 * @return Object RSS state with the calculated arc length, gap, and longitudinal velocity.
 */
ObjectRssState evaluate_object_state(
  const TargetObject & target_object, const TrajectoryPoints & trajectory_points,
  const TrajectoryShape & trajectory_shape, const ObjectDecelMap & object_decel_map,
  const double stopped_vel_th);

/**
 * @brief RSS longitudinal safe distance between the ego and an object ahead.
 * @param ego_vel Ego velocity [m/s].
 * @param ego_decel Magnitude of ego deceleration [m/s^2].
 * @param object_stopping_distance Object braking distance [m].
 * @param reaction_time Ego reaction time [s].
 * @param safety_margin Distance added to the required gap [m].
 * @return Required gap from ego front to object [m]; may be negative.
 */
double get_safe_distance(
  const double ego_vel, const double ego_decel, const double object_stopping_distance,
  const double reaction_time, const double safety_margin);

/**
 * @brief Largest ego velocity whose RSS safe distance does not exceed the current gap.
 * @details Inverse of get_safe_distance(): solves
 * `gap = v * reaction_time + v^2 / (2 * ego_decel) - object_stopping_distance + safety_margin`
 * for the non-negative root `v`.
 * @param gap Distance from ego front to object [m].
 * @param ego_decel Magnitude of ego deceleration [m/s^2].
 * @param object_stopping_distance Object braking distance [m].
 * @param reaction_time Ego reaction time [s].
 * @param safety_margin Distance added to the required gap [m].
 * @return Velocity [m/s], 0 if the gap does not exceed the required standstill distance.
 */
double compute_cruise_velocity(
  const double gap, const double ego_decel, const double object_stopping_distance,
  const double reaction_time, const double safety_margin);

/**
 * @brief Select the object to follow: the nearest along the path that moves forward.
 * @param target_objects Target objects.
 * @return The nearest target object with `is_dynamic` and positive `lon_vel`, or nullopt.
 */
std::optional<TargetObject> select_cruise_target(const TargetObjects & target_objects);

/**
 * @brief Find the nearest predicted object that is not longitudinally safe at the current state.
 * @details Each object is evaluated once from its stored `rss_state`, against the ego speed at the
 * start of `trajectory_points`. An object whose braking distance is zero, or whose gap is inside
 * the RSS safe distance, is a collision candidate. The candidate with the smallest collision arc
 * length is returned. When `use_rss_check` is false, every object is a candidate at its contact
 * arc length.
 * @param trajectory_points Reference path; the initial ego speed is read from the first point.
 * @param target_objects Predicted objects to evaluate. Each `rss_state` is already filled.
 * @param ego_decel Magnitude of ego deceleration [m/s^2] for the ego stopping term.
 * @param reaction_time system reaction time [s] to respond to detected collision.
 * @param min_safe_distance Lower limit on the required gap while the object is moving [m].
 * @param rss_safety_buffer Buffer added inside the RSS safe-distance formula [m].
 * @return Collision geometry and arc length, or nullopt if inputs are invalid or objects are safe
 */
std::optional<CollisionPoint> get_nearest_object_collision(
  TargetObjects & target_objects, const TrajectoryPoints & trajectory_points,
  const double ego_decel, const double reaction_time, const double min_safe_distance,
  const double rss_safety_buffer, const bool use_rss_check = true);

/// Filters predicted objects by semantic type, speed, and spatial relationship to the trajectory.
struct ObjectFilter
{
  /**
   * @brief Construct a filter from allowed type names and speed thresholds.
   * @param object_type_strings Allowed object classes (see string_to_object_type).
   * @param stopped_velocity_th Used when filtering by target area for "moving" vs stopped.
   * @param safety_buffer Safety buffer to expand object shape [m].
   * @param traj_low_velocity_th Low velocity threshold for the trajectory [m/s].
   */
  ObjectFilter(
    const std::vector<std::string> & bbox_object_type_strings,
    const std::vector<std::string> & polygon_object_type_strings, const double stopped_velocity_th,
    const double safety_buffer, const double traj_low_velocity_th)
  : stopped_velocity_th_(stopped_velocity_th),
    safety_buffer_(safety_buffer),
    traj_low_velocity_th_(traj_low_velocity_th)
  {
    for (const auto & object_type_string : bbox_object_type_strings) {
      if (string_to_object_type.count(object_type_string) == 0) continue;
      bbox_object_types_.emplace(string_to_object_type.at(object_type_string));
    }
    for (const auto & object_type_string : polygon_object_type_strings) {
      if (string_to_object_type.count(object_type_string) == 0) continue;
      polygon_object_types_.emplace(string_to_object_type.at(object_type_string));
    }
  }

  /**
   * @brief Remove objects that are too fast or whose class is not in the configured allow-list.
   * @param[in,out] objects Predicted objects message updated in place.
   */
  void filter_objects(PredictedObjects & objects)
  {
    objects.objects.erase(
      std::remove_if(
        objects.objects.begin(), objects.objects.end(),
        [&](const auto & object) {
          const auto label =
            object.classification.empty()
              ? ObjectClassification::UNKNOWN
              : autoware::object_recognition_utils::getHighestProbLabel(object.classification);
          if (classification_to_object_type.count(label) == 0) return true;
          if (object.shape.type == Shape::BOUNDING_BOX)
            return bbox_object_types_.count(classification_to_object_type.at(label)) == 0;
          if (object.shape.type == Shape::POLYGON)
            return polygon_object_types_.count(classification_to_object_type.at(label)) == 0;
          return true;
        }),
      objects.objects.end());
  }

  /**
   * @brief Keep only objects that intersect ego footprints, dropping lateral departures.
   * @details Objects with no overlapping footprint (expanded by `lat_margin`) are removed. A moving
   * object is also removed when its highest-confidence predicted path leaves through a side of the
   * ego corridor before ego's time_from_start reaches that overlap. A path that leaves through the
   * front or rear of the corridor is kept.
   * @param[in,out] objects Predicted objects to filter in place.
   * @param trajectory_points Reference path for time and geometry queries.
   * @param trajectory_shape Ego footprint index used for overlap queries.
   * @param lateral_margin_map Per-class lateral expansion of the ego footprint [m].
   * @param[out] target_polygons Footprints of objects that remain after filtering.
   */
  void filter_by_target_area(
    TargetObjects & target_objects, const TrajectoryPoints & trajectory_points,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    const TrajectoryShape & trajectory_shape, const LateralMarginMap & lateral_margin_map,
    const ObjectDecelMap & object_decel_map);

  /**
   * @brief Update allow-listed types and velocity thresholds without reconstructing the filter.
   */
  void set_params(
    const std::vector<std::string> & bbox_object_type_strings,
    const std::vector<std::string> & polygon_object_type_strings, const double stopped_velocity_th,
    const double safety_buffer, const double traj_low_velocity_th)
  {
    bbox_object_types_.clear();
    polygon_object_types_.clear();
    for (const auto & object_type_string : bbox_object_type_strings) {
      if (string_to_object_type.count(object_type_string) == 0) continue;
      bbox_object_types_.emplace(string_to_object_type.at(object_type_string));
    }
    for (const auto & object_type_string : polygon_object_type_strings) {
      if (string_to_object_type.count(object_type_string) == 0) continue;
      polygon_object_types_.emplace(string_to_object_type.at(object_type_string));
    }
    stopped_velocity_th_ = stopped_velocity_th;
    safety_buffer_ = safety_buffer;
    traj_low_velocity_th_ = traj_low_velocity_th;
  }

private:
  std::unordered_set<ObjectType> bbox_object_types_;
  std::unordered_set<ObjectType> polygon_object_types_;
  double stopped_velocity_th_;
  double safety_buffer_;
  double traj_low_velocity_th_;
};

/// Range and semantic-label filtering plus object masking for obstacle point clouds.
struct PointCloudFilter
{
  /**
   * @brief Configure the set of point-cloud class labels kept by subsequent filters.
   */
  explicit PointCloudFilter(const std::vector<std::string> & target_types)
  {
    for (const auto & target_type : target_types) {
      if (string_to_object_type.count(target_type) == 0) continue;
      pcd_types_.emplace(string_to_object_type.at(target_type));
    }
  };

  /**
   * @brief Update the kept point-cloud class labels at runtime.
   */
  void set_params(const std::vector<std::string> & target_types)
  {
    pcd_types_.clear();
    for (const auto & target_type : target_types) {
      if (string_to_object_type.count(target_type) == 0) continue;
      pcd_types_.emplace(string_to_object_type.at(target_type));
    }
  }

  /**
   * @brief Crop the cloud to an axis-aligned box, then keep only configured class types.
   * @param[in,out] pointcloud Cloud updated in place; empty if nothing remains.
   * @param min_x,max_x,min_y,max_y,min_z,max_z Crop box bounds in the cloud frame.
   */
  void filter_pointcloud(
    PointCloud::Ptr & pointcloud, const double min_x, const double max_x, const double min_y,
    const double max_y, const double min_z, const double max_z);

  /**
   * @brief Remove points that lie inside predicted object footprints (expanded by a small margin).
   * @param[in,out] pointcloud Cloud to strip in place.
   * @param objects Predicted obstacles whose shapes define removal regions.
   */
  void filter_pointcloud_by_object(PointCloud::Ptr & pointcloud, const PredictedObjects & objects);

private:
  std::unordered_set<ObjectType> pcd_types_;
};

/// Temporal association of obstacle detections (objects and points) with hysteresis.
struct ObstacleTracker
{
  /**
   * @brief Construct with time buffers and association thresholds.
   * @param on_time_buffer Seconds a track must be seen before it is considered active.
   * @param off_time_buffer Seconds without observation before an active track may be removed.
   * @param object_distance_th Max 2D distance to match a new detection to an existing object track.
   * @param object_yaw_th Max yaw difference to match object detections [rad].
   * @param pcd_distance_th Max 2D distance to match a point to an existing point track.
   * @param grace_period Seconds to keep inactive tracks before deletion.
   */
  ObstacleTracker(
    const double on_time_buffer, const double off_time_buffer, const double object_distance_th,
    const double object_yaw_th, const double pcd_distance_th, const double grace_period)
  : on_time_buffer_(on_time_buffer),
    off_time_buffer_(off_time_buffer),
    object_distance_th_(object_distance_th),
    object_yaw_th_(object_yaw_th),
    pcd_distance_th_(pcd_distance_th),
    grace_period_(grace_period)
  {
  }

  /**
   * @brief Update association and timing parameters.
   */
  void set_params(
    const double on_time_buffer, const double off_time_buffer, const double object_distance_th,
    const double object_yaw_th, const double pcd_distance_th, const double grace_period)
  {
    on_time_buffer_ = on_time_buffer;
    off_time_buffer_ = off_time_buffer;
    object_distance_th_ = object_distance_th;
    object_yaw_th_ = object_yaw_th;
    pcd_distance_th_ = pcd_distance_th;
    grace_period_ = grace_period;
  }

  /**
   * @brief Update tracked objects
   * @details Use input objects to update the tracked objects history and remove obsolete objects,
   * based on the on_time_buffer and off_time_buffer.
   * @param objects Input predicted objects after filtering
   * @param persistent_objects Output persistent objects
   * @param now Current stamp used for track aging and activation timing
   */
  void update_objects(
    const PredictedObjects & objects, TargetObjects & persistent_objects, const rclcpp::Time & now);

  /**
   * @brief Update tracked points
   * @details Use input pointcloud to update the tracked points history and remove obsolete points,
   * based on the on_time_buffer and off_time_buffer.
   * @param points Input pointcloud
   * @param persistent_points Output persistent points
   * @param now Current stamp used for track aging and activation timing
   */
  void update_points(
    const PointCloud::Ptr & points, PointCloud::Ptr & persistent_points, const rclcpp::Time & now);

private:
  double on_time_buffer_;
  double off_time_buffer_;
  double object_distance_th_;
  double object_yaw_th_;
  double pcd_distance_th_;
  double grace_period_;

  struct PersistentObstacle
  {
    rclcpp::Time first_seen_time;
    rclcpp::Time last_seen_time;
    bool is_active{false};

    explicit PersistentObstacle(const rclcpp::Time & now)
    : first_seen_time(now), last_seen_time(now)
    {
    }
  };

  struct PersistentObject : public PersistentObstacle
  {
    PredictedObject object;

    explicit PersistentObject(const PredictedObject & object, const rclcpp::Time & now)
    : PersistentObstacle(now), object(object)
    {
    }
  };
  std::unordered_map<boost::uuids::uuid, PersistentObject, boost::hash<boost::uuids::uuid>>
    persistent_objects_map_;

  struct PersistentPoint : public PersistentObstacle
  {
    PointXYZCPE point;
    explicit PersistentPoint(const PointXYZCPE & point, const rclcpp::Time & now)
    : PersistentObstacle(now), point(point)
    {
    }
  };
  std::unordered_map<boost::uuids::uuid, PersistentPoint, boost::hash<boost::uuids::uuid>>
    persistent_point_map_;

  boost::uuids::random_generator id_generator_;
};

}  // namespace autoware::trajectory_modifier::utils::obstacle_stop

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__OBSTACLE_STOP_UTILS_HPP_
