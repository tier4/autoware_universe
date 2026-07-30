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

#include "assessment.hpp"

#include <boost/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
bool is_pedestrian_or_bicycle(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object)
{
  using autoware_perception_msgs::msg::ObjectClassification;

  const auto label =
    autoware::object_recognition_utils::getHighestProbLabel(predicted_object.classification);
  return label == ObjectClassification::PEDESTRIAN || label == ObjectClassification::BICYCLE;
}

std::vector<Polygon2d> collect_nearby_intersection_area_polygons(
  const lanelet::LaneletMap & lanelet_map, const Polygon2d & object_hull)
{
  std::vector<Polygon2d> polygons;
  std::set<lanelet::Id> intersection_area_ids;
  lanelet::BoundingBox2d search_bbox;
  for (const auto & point : object_hull.outer()) {
    search_bbox.extend(lanelet::BasicPoint2d(point.x(), point.y()));
  }

  for (const auto & lanelet : lanelet_map.laneletLayer.search(search_bbox)) {
    const lanelet::Id area_id =
      std::atoi(std::string(lanelet.attributeOr("intersection_area", std::string{"0"})).c_str());
    if (area_id == 0 || !intersection_area_ids.insert(area_id).second) {
      continue;
    }

    const auto polygon_it = lanelet_map.polygonLayer.find(area_id);
    if (polygon_it == lanelet_map.polygonLayer.end()) {
      continue;
    }

    Polygon2d polygon;
    for (const auto & point : polygon_it->basicPolygon()) {
      polygon.outer().emplace_back(point.x(), point.y());
    }
    polygons.push_back(std::move(polygon));
  }

  return polygons;
}

std::vector<Polygon2d> collect_nearby_crosswalk_or_walkway_polygons(
  const lanelet::LaneletMap & lanelet_map, const Polygon2d & object_hull)
{
  std::vector<Polygon2d> polygons;
  lanelet::BoundingBox2d search_bbox;
  for (const auto & point : object_hull.outer()) {
    search_bbox.extend(lanelet::BasicPoint2d(point.x(), point.y()));
  }

  for (const auto & lanelet : lanelet_map.laneletLayer.search(search_bbox)) {
    const auto subtype = lanelet.attributeOr(lanelet::AttributeName::Subtype, std::string{});
    if (
      subtype != lanelet::AttributeValueString::Crosswalk &&
      subtype != lanelet::AttributeValueString::Walkway) {
      continue;
    }

    Polygon2d polygon;
    for (const auto & point : lanelet.polygon2d().basicPolygon()) {
      polygon.outer().emplace_back(point.x(), point.y());
    }
    polygons.push_back(std::move(polygon));
  }

  return polygons;
}

bool intersects_any(const Polygon2d & object_polygon, const std::vector<Polygon2d> & polygons)
{
  return std::any_of(polygons.begin(), polygons.end(), [&](const auto & polygon) {
    return boost::geometry::intersects(object_polygon, polygon);
  });
}

bool is_vru_prioritized_at_collision(
  const CollisionDetail & nominal_collision_result, const lanelet::LaneletMap & lanelet_map)
{
  const auto intersection_area_polygons =
    collect_nearby_intersection_area_polygons(lanelet_map, nominal_collision_result.object_hull);
  if (!intersects_any(nominal_collision_result.object_hull, intersection_area_polygons)) {
    return true;
  }

  const auto crosswalk_or_walkway_polygons =
    collect_nearby_crosswalk_or_walkway_polygons(lanelet_map, nominal_collision_result.object_hull);
  return intersects_any(nominal_collision_result.object_hull, crosswalk_or_walkway_polygons);
}
}  // namespace
}  // namespace autoware::trajectory_validator::plugin::safety

namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment
{
std::optional<CollisionDetail> find_collision_timing(
  const TrajectoryData & ref_trajectory, const TrajectoryData & test_trajectory,
  const DracParams::PetMargin & pet_find_range, double time_resolution)
{
  const double max_pet_threshold =
    std::max(pet_find_range.ego_earlier, pet_find_range.object_earlier);

  const auto overall_test_index_range = test_trajectory.resolve_covering_index_range(
    {ref_trajectory.getTimes().front() - pet_find_range.object_earlier,
     ref_trajectory.getTimes().back() + pet_find_range.ego_earlier});
  if (!overall_test_index_range) {
    return std::nullopt;
  }

  if (!boost::geometry::intersects(
        ref_trajectory.get_or_compute_overall_envelope(),
        test_trajectory.get_or_compute_envelope(*overall_test_index_range))) {
    return std::nullopt;
  }

  struct CandidateFinding
  {
    double ttc;
    double pet;
    IndexRange ref_index_range;
    IndexRange test_index_range;
  };

  const auto make_collision_detail =
    [&](
      const CandidateFinding & worst_pet,
      const CandidateFinding & first_collision) -> CollisionDetail {
    return CollisionDetail{
      test_trajectory.getObjectIdentification(),
      CollisionTiming{first_collision.ttc, first_collision.pet},
      CollisionTiming{worst_pet.ttc, worst_pet.pet},
      ref_trajectory.getPoses(),
      test_trajectory.getPoses(),
      ref_trajectory.get_or_compute_convex(worst_pet.ref_index_range),
      test_trajectory.get_or_compute_convex(worst_pet.test_index_range)};
  };

  std::optional<CandidateFinding> first_collision_timing{};
  std::optional<CandidateFinding> worst_pet_timing{};
  for (size_t i = 0; i < ref_trajectory.size(); ++i) {
    size_t prev_i = (i == 0) ? 0 : i - 1;
    const double ref_start_time = ref_trajectory.getTimes().at(prev_i);
    const double ref_end_time = ref_trajectory.getTimes().at(i);

    const IndexRange ref_index_range{prev_i, i};
    const Box2d & ref_envelope = ref_trajectory.get_or_compute_envelope(ref_index_range);
    const Polygon2d & ref_convex = ref_trajectory.get_or_compute_convex(ref_index_range);

    const double current_pet_limit =
      worst_pet_timing.has_value() ? std::abs(worst_pet_timing->pet) : max_pet_threshold;

    const auto rough_test_index_range = test_trajectory.resolve_covering_index_range(
      {ref_start_time - current_pet_limit, ref_end_time + current_pet_limit});
    if (!rough_test_index_range) {
      continue;
    }

    if (!boost::geometry::intersects(
          ref_envelope, test_trajectory.get_or_compute_envelope(rough_test_index_range.value()))) {
      continue;
    }

    const auto has_intersects = [&](const IndexRange & index_range) -> bool {
      if (!boost::geometry::intersects(
            ref_envelope, test_trajectory.get_or_compute_envelope(index_range))) {
        return false;
      }

      return geometry::intersects_sat(
        ref_convex, test_trajectory.get_or_compute_convex(index_range));
    };

    const auto find_candidate = [&](const double pet_range) -> std::optional<CandidateFinding> {
      for (const double pet : {-pet_range, pet_range}) {
        const auto test_index_range =
          test_trajectory.resolve_covering_index_range({ref_start_time + pet, ref_end_time + pet});
        if (!test_index_range || !has_intersects(test_index_range.value())) {
          continue;
        }

        return CandidateFinding{ref_start_time, pet, ref_index_range, test_index_range.value()};
      }
      return std::nullopt;
    };

    for (double pet_range = 0.0; pet_range < current_pet_limit; pet_range += time_resolution) {
      const auto candidate = find_candidate(pet_range);
      if (!candidate) {
        continue;
      }

      worst_pet_timing = *candidate;
      if (!first_collision_timing.has_value()) {
        first_collision_timing = worst_pet_timing;
      }
      break;
    }
    if (worst_pet_timing.has_value() && worst_pet_timing->pet == 0.0) {
      return make_collision_detail(worst_pet_timing.value(), first_collision_timing.value());
    }
  }

  if (!worst_pet_timing.has_value()) {
    return std::nullopt;
  }

  return make_collision_detail(worst_pet_timing.value(), first_collision_timing.value());
}

RiskLevel::_level_type identify_risk_level(
  const std::optional<double> & required_acceleration,
  const DracParams::EgoDracAcceleration & acceleration_params)
{
  if (!required_acceleration.has_value()) {
    return acceleration_params.enable_abandon ? RiskLevel::LOW_CAUTION : RiskLevel::FATAL;
  }

  if (required_acceleration >= acceleration_params.safe_limit) {
    return RiskLevel::SAFE;
  } else if (required_acceleration >= acceleration_params.danger_limit) {
    return RiskLevel::DANGER;
  } else {
    return RiskLevel::FATAL;
  }
}

// Object counterpart of identify_risk_level: classifies how hard the object must brake to avoid the
// collision into a risk level, mirroring the ego DRAC acceleration tiers.
RiskLevel::_level_type identify_object_risk_level(
  const double required_object_acceleration,
  const DracParams::ObjectDracAcceleration & object_acceleration_params)
{
  return (required_object_acceleration >= object_acceleration_params.safe_limit)
           ? RiskLevel::SAFE
           : RiskLevel::LOW_CAUTION;
}

struct EgoDracAssessmentParams
{
  DracParams::PetMargin pet_margin{};
  EgoFootprintMargin ego_footprint_margin{};
  DracParams::EgoDracAcceleration acceleration{};
  double braking_delay{};
};

// Object counterpart of EgoDracAssessmentParams. Nothing about the ego motion is needed here: the
// assessment runs against the nominal ego trajectory, which the caller supplies ready-made.
struct ObjectDracAssessmentParams
{
  DracParams::PetMargin pet_margin{};
  DracParams::ObjectDracAcceleration acceleration{};
};

std::pair<std::optional<double>, std::optional<CollisionDetail>> assess_ego_drac(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const TrajectoryData & object_trajectory, const EgoDracAssessmentParams & params,
  const GlobalParams & global_params)
{
  std::vector<double> ego_acceleration_list{
    params.acceleration.safe_limit, params.acceleration.danger_limit,
    params.acceleration.fatal_limit};

  std::optional<CollisionDetail> last_detected_collision{};
  for (auto ego_acceleration : ego_acceleration_list) {
    trajectory::EgoTrajectoryGenerationParams ego_traj_params{
      params.braking_delay, ego_acceleration, params.ego_footprint_margin};
    const auto & ego_trajectory =
      ego_trajectory_cache.get_or_compute_trajectory_data(ego_traj_params);

    auto detected_collision = find_collision_timing(
      ego_trajectory, object_trajectory, params.pet_margin, global_params.time_resolution);
    if (!detected_collision.has_value()) {
      return {ego_acceleration, last_detected_collision};
    }
    last_detected_collision = std::move(detected_collision);
  }

  trajectory::EgoTrajectoryGenerationParams limit_ego_traj_params{
    0.0, ego_acceleration_list.back(), params.ego_footprint_margin};
  const auto & limit_ego_trajectory =
    ego_trajectory_cache.get_or_compute_trajectory_data(limit_ego_traj_params);
  auto collision_result = find_collision_timing(
    limit_ego_trajectory, object_trajectory, params.pet_margin, global_params.time_resolution);
  if (!collision_result.has_value()) {
    return {ego_acceleration_list.back(), last_detected_collision};
  }

  return {std::nullopt, collision_result};
}

// Assesses whether the object decelerating (instead of ego braking) is enough to avoid the
// collision. The object trajectory for each assumed deceleration is memoized in the object cache.
std::pair<std::optional<double>, std::optional<CollisionDetail>> assess_object_drac(
  const trajectory::ObjectTrajectoryCache & object_trajectory_cache,
  const TrajectoryData & ego_nominal_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & object, const size_t predicted_path_index,
  const ObjectDracAssessmentParams & params, const GlobalParams & global_params)
{
  const std::vector<double> object_acceleration_list{
    params.acceleration.safe_limit, params.acceleration.low_caution_limit};

  std::optional<CollisionDetail> last_detected_collision{};
  for (const auto object_acceleration : object_acceleration_list) {
    const auto & object_trajectory =
      object_trajectory_cache.get_or_compute_predicted_path_trajectory(
        object, predicted_path_index, 0.0, object_acceleration, 8.0);

    auto detected_collision = find_collision_timing(
      ego_nominal_trajectory, object_trajectory, params.pet_margin, global_params.time_resolution);
    if (!detected_collision.has_value()) {
      return {object_acceleration, last_detected_collision};
    }
    last_detected_collision = std::move(detected_collision);
  }
  return {std::nullopt, last_detected_collision};
}

DracEvaluation assess_drac_constant_curvature_object_first(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const TrajectoryData & object_constant_curvature_trajectory, const DracParams & drac_params,
  const GlobalParams & global_params, CollisionDetail && nominal_collision_result)
{
  const auto ego_drac_params = EgoDracAssessmentParams{
    drac_params.pet_margin, drac_params.ego_footprint_margin,
    drac_params.constant_curvature.object_earlier.ego_drac_assessment,
    drac_params.ego_reaction_braking_delay.nominal};
  auto [required_acceleration, last_collision] = assess_ego_drac(
    ego_trajectory_cache, object_constant_curvature_trajectory, ego_drac_params, global_params);

  DracEvaluation evaluation{};
  evaluation.method = "constant_curvature, object earlier";
  evaluation.risk = identify_risk_level(required_acceleration, ego_drac_params.acceleration);
  evaluation.ego_drac_acceleration = required_acceleration;
  evaluation.detail = std::move(last_collision).value_or(std::move(nominal_collision_result));
  return evaluation;
}

DracArtifact assess_constant_curvature(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const trajectory::ObjectTrajectoryCache & object_trajectory_cache,
  const autoware_perception_msgs::msg::PredictedObject & object, const DracParams & drac_params,
  const GlobalParams & global_params)
{
  DracArtifact drac_artifact{};

  // todo (takagi): use departure value is necessary.
  const double braking_delay = drac_params.ego_reaction_braking_delay.nominal;
  trajectory::EgoTrajectoryGenerationParams ego_traj_params{
    braking_delay, 0.0, drac_params.ego_footprint_margin};

  const auto & ego_nominal_trajectory =
    ego_trajectory_cache.get_or_compute_trajectory_data(ego_traj_params);

  const auto & object_constant_curvature_trajectory =
    object_trajectory_cache.get_or_compute_constant_curvature_trajectory(
      object, 0.0, 0.0, drac_params.constant_curvature.object_time_horizon);

  auto nominal_collision_result = find_collision_timing(
    ego_nominal_trajectory, object_constant_curvature_trajectory, drac_params.pet_margin,
    global_params.time_resolution);
  if (!nominal_collision_result.has_value()) {
    return drac_artifact;
  }

  if (nominal_collision_result.value().first_collision_timing.pet > 0.0) {
    if (drac_params.constant_curvature.ego_earlier.enable_assessment) {
      throw std::invalid_argument("constant_curvature.ego_earlier is not implemented.");
    }
  } else {
    if (drac_params.constant_curvature.object_earlier.enable_assessment) {
      drac_artifact.merge(assess_drac_constant_curvature_object_first(
        ego_trajectory_cache, object_constant_curvature_trajectory, drac_params, global_params,
        std::move(nominal_collision_result.value())));
    }
  }
  return drac_artifact;
}

DracEvaluation assess_drac_object_prioritized_ego_earlier(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const trajectory::ObjectTrajectoryCache & object_trajectory_cache,
  const TrajectoryData & ego_nominal_trajectory, const TrajectoryData & object_map_based_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & object, const size_t predicted_path_index,
  const DracParams & drac_params, const GlobalParams & global_params,
  CollisionDetail && nominal_collision_result)
{
  const auto ego_drac_params = EgoDracAssessmentParams{
    drac_params.pet_margin, drac_params.ego_footprint_margin,
    drac_params.map_based.object_prioritized_ego_earlier.ego_drac_assessment,
    drac_params.ego_reaction_braking_delay.nominal};

  DracEvaluation evaluation{};
  evaluation.method = "map_based, object prioritized, ego earlier";

  // If the object decelerating on its own resolves the collision, ego need not brake. The assumed
  // object decelerations are searched from mild to hard, mirroring the ego DRAC acceleration list,
  // and the required object deceleration is mapped to a risk level.
  const auto object_drac_params = ObjectDracAssessmentParams{
    drac_params.pet_margin,
    drac_params.map_based.object_prioritized_ego_earlier.object_drac_acceleration};

  auto [object_required_acceleration, object_deceleration_last_collision] = assess_object_drac(
    object_trajectory_cache, ego_nominal_trajectory, object, predicted_path_index,
    object_drac_params, global_params);
  if (object_required_acceleration.has_value()) {
    evaluation.risk = identify_object_risk_level(
      object_required_acceleration.value(), object_drac_params.acceleration);
    evaluation.ego_drac_acceleration = 0.0;
    // The mildest searched deceleration may avoid the collision without producing a collision
    // detail of its own; fall back to the nominal (constant-velocity) collision for reporting in
    // that case.
    evaluation.detail = object_deceleration_last_collision.has_value()
                          ? std::move(object_deceleration_last_collision.value())
                          : std::move(nominal_collision_result);
    return evaluation;
  }

  auto [required_acceleration, last_collision] = assess_ego_drac(
    ego_trajectory_cache, object_map_based_trajectory, ego_drac_params, global_params);

  evaluation.risk = identify_risk_level(required_acceleration, ego_drac_params.acceleration);
  evaluation.ego_drac_acceleration = required_acceleration;
  evaluation.detail = std::move(last_collision).value_or(std::move(nominal_collision_result));
  return evaluation;
}

DracEvaluation assess_drac_object_prioritized_object_earlier(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const TrajectoryData & object_map_based_trajectory, const DracParams & drac_params,
  const GlobalParams & global_params, CollisionDetail && nominal_collision_result)
{
  const auto ego_drac_params = EgoDracAssessmentParams{
    drac_params.pet_margin, drac_params.ego_footprint_margin,
    drac_params.map_based.object_prioritized_object_earlier.ego_drac_assessment,
    drac_params.ego_reaction_braking_delay.nominal};
  auto [required_acceleration, last_collision] = assess_ego_drac(
    ego_trajectory_cache, object_map_based_trajectory, ego_drac_params, global_params);

  DracEvaluation evaluation{};
  evaluation.method = "map_based, object prioritized, object earlier";
  evaluation.risk = identify_risk_level(required_acceleration, ego_drac_params.acceleration);
  evaluation.ego_drac_acceleration = required_acceleration;
  evaluation.detail = std::move(last_collision).value_or(std::move(nominal_collision_result));
  return evaluation;
}

DracArtifact assess_map_based(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const trajectory::ObjectTrajectoryCache & object_trajectory_cache,
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & ego_turn_indicator,
  const autoware_perception_msgs::msg::PredictedObject & object, const StopTrackers & stop_trackers,
  const DracParams & drac_params, const GlobalParams & global_params,
  const lanelet::LaneletMap & lanelet_map)
{
  DracArtifact drac_artifact{};

  const double braking_delay = drac_params.ego_reaction_braking_delay.nominal;
  trajectory::EgoTrajectoryGenerationParams ego_traj_params{
    braking_delay, 0.0, drac_params.ego_footprint_margin};

  for (size_t predicted_path_index = 0;
       predicted_path_index < object.kinematics.predicted_paths.size(); ++predicted_path_index) {
    const auto & predicted_path_nominal_trajectory =
      object_trajectory_cache.get_or_compute_predicted_path_trajectory(
        object, predicted_path_index, 0.0, 0.0, 8.0);

    const auto & ego_nominal_trajectory =
      ego_trajectory_cache.get_or_compute_trajectory_data(ego_traj_params);

    auto nominal_collision_result = find_collision_timing(
      ego_nominal_trajectory, predicted_path_nominal_trajectory, drac_params.pet_margin,
      global_params.time_resolution);
    if (!nominal_collision_result.has_value()) {
      continue;
    }
    const bool use_object_prioritized_assessment = [&]() {
      if (
        ego_turn_indicator.command != autoware_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE) {
        return true;
      }
      if (!is_pedestrian_or_bicycle(object)) {
        return false;
      }
      return is_vru_prioritized_at_collision(nominal_collision_result.value(), lanelet_map);
    }();

    if (!use_object_prioritized_assessment) {
      if (nominal_collision_result.value().first_collision_timing.pet > 0.0) {
        if (!drac_params.map_based.ego_prioritized_ego_earlier.enable_assessment) {
          continue;
        }
        throw std::invalid_argument("map_based.ego_prioritized_ego_earlier is not implemented.");

      } else {
        if (!drac_params.map_based.ego_prioritized_object_earlier.enable_assessment) {
          continue;
        }
        throw std::invalid_argument("map_based.ego_prioritized_object_earlier is not implemented.");
      }
    } else {
      if (nominal_collision_result.value().first_collision_timing.pet > 0.0) {
        if (!drac_params.map_based.object_prioritized_ego_earlier.enable_assessment) {
          continue;
        }
        drac_artifact.merge(assess_drac_object_prioritized_ego_earlier(
          ego_trajectory_cache, object_trajectory_cache, ego_nominal_trajectory,
          predicted_path_nominal_trajectory, object, predicted_path_index, drac_params,
          global_params, std::move(nominal_collision_result.value())));
      } else {
        if (!drac_params.map_based.object_prioritized_object_earlier.enable_assessment) {
          continue;
        }
        drac_artifact.merge(assess_drac_object_prioritized_object_earlier(
          ego_trajectory_cache, predicted_path_nominal_trajectory, drac_params, global_params,
          std::move(nominal_collision_result.value())));
      }
    }
  }

  if (drac_params.map_based.mutual_yield_timeout_arbitration.enabled) {
    const auto stopped_duration = stop_trackers.get_stopped_duration(object.object_id);
    if (
      stopped_duration.has_value() &&
      stopped_duration.value() >
        rclcpp::Duration::from_seconds(
          drac_params.map_based.mutual_yield_timeout_arbitration.min_wait_time)) {
      for (auto & evaluation : drac_artifact.evaluations) {
        evaluation.risk = RiskLevel::SAFE;
      }
      drac_artifact.risk = RiskLevel::SAFE;
    }
  }
  return drac_artifact;
}

DracArtifact assess(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache,
  const trajectory::ObjectTrajectoryCache & object_trajectory_cache,
  const autoware_vehicle_msgs::msg::TurnIndicatorsCommand & ego_turn_indicator,
  const nav_msgs::msg::Odometry & odometry,
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
  const lanelet::LaneletMap & lanelet_map, StopTrackers & stop_trackers,
  const DracParamMap & drac_param_map, const GlobalParams & global_params)
{
  DracArtifact drac_artifact{};

  stop_trackers.ego.update(odometry);
  stop_trackers.object.update(predicted_objects);

  for (const auto & predicted_object : predicted_objects.objects) {
    const auto & drac_params = drac_param_map.at(to_type_string(predicted_object.classification));
    if (!drac_params.target_shape_types.contains(predicted_object.shape.type)) {
      continue;
    }

    if (drac_params.constant_curvature.enable_assessment) {
      drac_artifact.merge(assess_constant_curvature(
        ego_trajectory_cache, object_trajectory_cache, predicted_object, drac_params,
        global_params));
    }

    if (drac_params.map_based.enable_assessment) {
      drac_artifact.merge(assess_map_based(
        ego_trajectory_cache, object_trajectory_cache, ego_turn_indicator, predicted_object,
        stop_trackers, drac_params, global_params, lanelet_map));
    }
  }
  return drac_artifact;
}

}  // namespace autoware::trajectory_validator::plugin::safety::collision_timing_assessment

namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
{
std::optional<double> compute_distance_to_collision(
  const TrajectoryData & ego_trajectory,
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto object_footprint =
    geometry::to_polygon2d(object.kinematics.initial_pose_with_covariance.pose, object.shape);
  const auto object_envelope = boost::geometry::return_envelope<Box2d>(object_footprint);

  if (!boost::geometry::intersects(
        ego_trajectory.get_or_compute_overall_envelope(), object_envelope)) {
    return std::nullopt;
  }

  for (size_t i = 0; i < ego_trajectory.size(); ++i) {
    const auto prev_i = (i == 0) ? 0 : (i - 1);
    const auto & ego_footprint = ego_trajectory.get_or_compute_convex(IndexRange{prev_i, i});
    if (geometry::intersects_sat(ego_footprint, object_footprint)) {
      return ego_trajectory.getDistances().at(i);
    }
  }

  return std::nullopt;
}

RssDetail assess_required_acceleration(
  const TrajectoryData & ego_trajectory, const geometry_msgs::msg::Twist & ego_twist,
  const autoware_perception_msgs::msg::PredictedObject & object, const RssParams & rss_params,
  const rclcpp::Time & stamp)
{
  const auto ego_long_vel = ego_twist.linear.x;
  if (ego_long_vel <= 0.0) {
    return RssDetail{TrajectoryIdentification{object, stamp}, 0.0};
  }

  const auto distance_to_collision = compute_distance_to_collision(ego_trajectory, object);
  if (!distance_to_collision.has_value()) {
    return RssDetail{TrajectoryIdentification{object, stamp}, 0.0};
  }

  const double obj_long_vel =
    std::clamp(compute_longitudinal_velocity(ego_trajectory.getPoses(), object), 0.0, 30.0);
  const double safe_distance =
    distance_to_collision.value() - rss_params.stop_distance_margin +
    obj_long_vel * obj_long_vel * 0.5 / -rss_params.object_assumed_acceleration -
    ego_long_vel * rss_params.ego_total_braking_delay;

  const double required_deceleration = safe_distance <= 0.0
                                         ? std::numeric_limits<double>::infinity()
                                         : ego_long_vel * ego_long_vel * 0.5 / safe_distance;

  return RssDetail{TrajectoryIdentification{object, stamp}, -required_deceleration};
}

RssArtifact assess(
  const trajectory::EgoTrajectoryCache & ego_trajectory_cache, const FilterContext & context,
  const RssParamMap & rss_param_map)
{
  if (!context.predicted_objects || context.predicted_objects->objects.empty()) {
    return {};
  }

  std::vector<RssEvaluation> rss_evaluations{};
  rss_evaluations.reserve(context.predicted_objects->objects.size());

  for (const auto & object : context.predicted_objects->objects) {
    const auto & rss_params = rss_param_map.at(to_type_string(object.classification));
    if (!rss_params.target_shape_types.contains(object.shape.type)) {
      continue;
    }

    trajectory::EgoTrajectoryGenerationParams ego_traj_params{
      0.0, 0.0, rss_params.ego_footprint_margin};
    const auto & ego_trajectory =
      ego_trajectory_cache.get_or_compute_trajectory_data(ego_traj_params);

    const auto rss_detail = assess_required_acceleration(
      ego_trajectory, context.odometry->twist.twist, object, rss_params,
      context.predicted_objects->header.stamp);
    // todo(takagi): fix risk level
    const auto risk_level =
      rss_detail.rss_acceleration < rss_params.error_threshold.ego_acceleration ? RiskLevel::DANGER
                                                                                : RiskLevel::SAFE;
    rss_evaluations.push_back(RssEvaluation{risk_level, rss_detail});
  }

  return RssArtifact{calc_worst_risk(rss_evaluations), std::move(rss_evaluations)};
}
}  // namespace autoware::trajectory_validator::plugin::safety::rss_deceleration
