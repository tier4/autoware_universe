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

#include "autoware/ml_planner/preprocessing/scene_input_builder.hpp"

#include "autoware/ml_planner/constants.hpp"
#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/preprocessing/items/ego_shape.hpp"
#include "autoware/ml_planner/preprocessing/items/traffic_light_features.hpp"

#include <Eigen/Geometry>
#include <xtensor/xbuilder.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::ml_planner::preprocess
{
namespace
{
constexpr int64_t kStepNanoseconds =
  static_cast<int64_t>(constants::PREDICTION_TIME_STEP_S * 1'000'000'000);
constexpr int64_t kHistoryNanoseconds =
  static_cast<int64_t>(HISTORY_WINDOW_S * 1'000'000'000 + 0.5);
constexpr int64_t kTimestampToleranceNanoseconds = 1'000;
constexpr auto kHistorySteps = static_cast<size_t>(kHistoryNanoseconds / kStepNanoseconds);
constexpr auto kInputRows = static_cast<size_t>(INPUT_T_WITH_CURRENT);
constexpr auto kOutputRows = static_cast<size_t>(OUTPUT_T);
using PoseMatrix = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using ModelPose = std::array<float, POSE_DIM>;
using TimestampedRows = std::map<int64_t, size_t>;

void require(const bool condition, const char * message)
{
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

template <typename T>
std::optional<T> value_at(const std::optional<t4scene::Array<T>> & column, const size_t index)
{
  return column ? column->get(index) : std::nullopt;
}

Eigen::Matrix4d rigid_matrix(const double * values)
{
  Eigen::Matrix4d result = Eigen::Map<const PoseMatrix>(values);
  const auto rotation = result.block<3, 3>(0, 0);
  require(
    result.allFinite() && result.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1)) &&
      rotation.isUnitary(1e-5) && std::abs(rotation.determinant() - 1.0) < 1e-5,
    "expected a finite rigid transform in T4Scene");
  return result;
}

std::optional<Eigen::Matrix4d> pose_at(
  const std::optional<t4scene::Array<double>> & column, const size_t row)
{
  if (!column) {
    return std::nullopt;
  }
  const auto pose = column->row(row);
  require(pose.shape() == std::vector<uint64_t>{4, 4}, "T4Scene pose rows must have shape [4, 4]");
  for (size_t i = 0; i < pose.size(); ++i) {
    if (pose.masked(i)) {
      return std::nullopt;
    }
  }
  return rigid_matrix(pose.data());
}

Eigen::Matrix4d scene_to_map(const t4scene::T4Scene & scene)
{
  const auto map_to_scene = rigid_matrix(t4scene::map_to_scene(scene).data());
  return Eigen::Isometry3d(map_to_scene).inverse().matrix();
}

ModelPose model_pose(const Eigen::Matrix4d & pose)
{
  const float yaw = std::atan2(pose(1, 0), pose(0, 0));
  return {
    static_cast<float>(pose(0, 3)), static_cast<float>(pose(1, 3)), std::cos(yaw), std::sin(yaw)};
}

// A missing source timestamp leaves the sampled grid timestamp as the only
// time available. Known source times retain the model's existing freshness rules.
int64_t measurement_time(const std::optional<int64_t> timestamp, const int64_t grid_time)
{
  const auto stamp = timestamp.value_or(grid_time);
  require(
    stamp <= grid_time || static_cast<uint64_t>(stamp) - static_cast<uint64_t>(grid_time) <=
                            kTimestampToleranceNanoseconds,
    "T4Scene observation is ahead of its grid row");
  return stamp;
}

bool eligible_for_grid(const int64_t stamp, const int64_t grid_time)
{
  return stamp <= grid_time || static_cast<uint64_t>(stamp) - static_cast<uint64_t>(grid_time) <=
                                 kTimestampToleranceNanoseconds;
}

bool within_history(const int64_t stamp, const int64_t newest)
{
  return static_cast<uint64_t>(newest) - static_cast<uint64_t>(stamp) <= kHistoryNanoseconds;
}

// SLERP and velocity resampling belong to Scene generation. Project each known
// vector in Scene axes onto the corresponding axis of the sampled ego pose.
std::optional<double> body_component(
  const std::optional<t4scene::Array<double>> & vector, const size_t row,
  const Eigen::Vector3d & axis)
{
  if (!vector) {
    return std::nullopt;
  }
  const auto sample = vector->row(row);
  require(sample.shape() == std::vector<uint64_t>{3}, "T4Scene motion rows must have shape [3]");
  double result = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    if (std::abs(axis(i)) <= 1e-12) {
      continue;
    }
    const auto component = sample.get(i);
    if (!component) {
      return std::nullopt;
    }
    require(std::isfinite(*component), "T4Scene ego motion must be finite");
    result += axis(i) * *component;
  }
  return result;
}

void add_ego_tensors(
  TensorMap & tensors, const t4scene::T4Scene & scene, const Eigen::Matrix4d & scene_to_ego)
{
  const auto & ego = *scene.ego;
  const auto current = scene.current_index;
  xt::xarray<float> history = xt::zeros<float>({kInputRows, static_cast<size_t>(EGO_HISTORY_DIM)});
  xt::xarray<float> turn_indicators = xt::zeros<float>({kInputRows});

  for (size_t i = 0; i < kInputRows; ++i) {
    const size_t row = current - INPUT_T + i;
    history(i, EGO_AGENT_PAST_IDX_COS) = 1.0F;
    if (!value_at(ego.available, row).value_or(true)) {
      continue;
    }
    if (const auto pose = pose_at(ego.poses_3d, row)) {
      const auto projected = model_pose(scene_to_ego * *pose);
      std::copy(projected.begin(), projected.end(), history.begin() + i * EGO_HISTORY_DIM);
      const auto velocity = body_component(ego.velocities_3d, row, pose->block<3, 1>(0, 0));
      const auto yaw_rate = body_component(ego.angular_velocities_3d, row, pose->block<3, 1>(0, 2));
      require(
        row != current || (velocity && yaw_rate),
        "T4Scene current ego motion is missing its longitudinal velocity or yaw rate");
      history(i, EGO_AGENT_PAST_IDX_VELOCITY) = velocity.value_or(0.0);
      if (std::abs(velocity.value_or(0.0)) >= constants::MOVING_VELOCITY_THRESHOLD_MPS) {
        history(i, EGO_AGENT_PAST_IDX_YAW_RATE) = std::clamp(yaw_rate.value_or(0.0), -0.95, 0.95);
      }
    }
    const auto steering = value_at(ego.tire_steering_angles, row).value_or(0.0);
    const auto indicator = value_at(ego.turn_indicators, row).value_or(0);
    require(std::isfinite(steering), "invalid T4Scene steering angle");
    require(indicator >= 0 && indicator <= 3, "invalid T4Scene turn indicator");
    history(i, EGO_AGENT_PAST_IDX_STEERING) = static_cast<float>(steering);
    turn_indicators(i) = static_cast<float>(indicator);
  }
  tensors.emplace("ego_agent_past", std::move(history));
  tensors.emplace("turn_indicators", std::move(turn_indicators));

  const auto & shape = ego.shape;
  require(
    shape && shape->base_link_to_front && shape->length && shape->width && shape->wheel_base,
    "T4Scene ego shape requires front offset, length, width and wheelbase");
  const std::array dimensions{
    *shape->base_link_to_front, *shape->length, *shape->width, *shape->wheel_base};
  require(
    std::all_of(
      dimensions.begin(), dimensions.end(),
      [](double dimension) { return std::isfinite(dimension) && dimension > 0.0; }),
    "T4Scene vehicle dimensions must be finite and positive");
  tensors.emplace(
    "ego_shape", create_ego_shape(dimensions[0], dimensions[1], dimensions[2], dimensions[3]));
}

TimestampedRows observation_rows(
  const t4scene::T4Scene & scene, const size_t first, const size_t last,
  const t4scene::Array<bool> & available, const std::optional<t4scene::Array<int64_t>> & timestamps,
  const bool trim_history = true)
{
  TimestampedRows rows;
  for (size_t row = first; row <= last; ++row) {
    if (available.at(row)) {
      const auto grid_time = scene.timestamps_ns.at(row);
      rows.try_emplace(measurement_time(value_at(timestamps, row), grid_time), row);
    }
  }
  while (trim_history && !rows.empty() &&
         !within_history(rows.begin()->first, rows.rbegin()->first)) {
    rows.erase(rows.begin());
  }
  return rows;
}

template <typename Group, typename Visitor>
void visit_group(
  const std::optional<Group> & group, const std::optional<t4scene::Array<int64_t>> & offsets,
  const size_t row, const Visitor & visit)
{
  if (!group) {
    return;
  }
  require(offsets.has_value(), "packed T4Scene annotations require row offsets");
  const auto first = offsets->at(row);
  const auto last = offsets->at(row + 1);
  require(first >= 0 && last >= first, "invalid T4Scene annotation offsets");
  for (auto i = first; i < last; ++i) {
    visit(*group, static_cast<size_t>(i));
  }
}

std::optional<size_t> label_channel(const int64_t label)
{
  // Public T4Scene ObjectCategory values, in the model's vehicle/pedestrian/bicycle order.
  switch (label) {
    case 1:  // vehicle
    case 2:  // car
    case 3:  // truck
    case 4:  // bus
    case 5:  // motorcycle
    case 8:  // trailer
      return 0;
    case 7:  // pedestrian
      return 1;
    case 6:  // bicycle
      return 2;
    default:
      return std::nullopt;
  }
}

template <typename Group>
std::optional<Eigen::Matrix4d> object_pose(
  const Group & group, const size_t i, const Eigen::Matrix4d & scene_to_ego)
{
  if (!group.positions || !group.orientations) {
    return std::nullopt;
  }
  const auto & positions = *group.positions;
  const auto & orientations = *group.orientations;
  const auto & position_shape = positions.shape();
  const auto & orientation_shape = orientations.shape();
  if (
    position_shape.empty() || orientation_shape.empty() || i >= position_shape[0] ||
    i >= orientation_shape[0]) {
    throw std::out_of_range("array row slice out of range");
  }
  require(
    position_shape.size() == 2 && position_shape[1] == 3 && orientation_shape.size() == 2 &&
      orientation_shape[1] == 4,
    "invalid T4Scene object pose columns");
  // T4Scene stores each column contiguously, including sliced views.
  const size_t position_offset = i * 3;
  const size_t orientation_offset = i * 4;
  if (positions.has_mask() || orientations.has_mask()) {
    for (size_t component = 0; component < 4; ++component) {
      if (
        orientations.masked(orientation_offset + component) ||
        (component < 3 && positions.masked(position_offset + component))) {
        return std::nullopt;
      }
    }
  }
  Eigen::Map<const Eigen::Vector3d> translation(positions.data() + position_offset);
  Eigen::Quaterniond quaternion(
    Eigen::Map<const Eigen::Vector4d>(orientations.data() + orientation_offset));
  require(
    translation.allFinite() && std::abs(quaternion.squaredNorm() - 1.0) < 1e-5,
    "T4Scene object pose must be finite with a unit quaternion");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = quaternion.toRotationMatrix();
  pose.translation() = translation;
  return scene_to_ego * pose.matrix();
}

struct AgentCandidate
{
  std::string track_id;
  double distance;
  size_t label;
  float width;
  float length;
};

template <typename Group>
std::optional<AgentCandidate> candidate_at(
  const Group & group, const size_t i, const Eigen::Matrix4d & scene_to_ego)
{
  const auto label = label_channel(value_at(group.labels, i).value_or(0));
  if (!label || !group.track_ids || !group.track_ids->at(i) || group.track_ids->at(i)->empty()) {
    return std::nullopt;
  }
  const auto pose = object_pose(group, i, scene_to_ego);
  if (!pose || !group.dimensions) {
    return std::nullopt;
  }
  const auto dimensions = group.dimensions->row(i);
  require(dimensions.shape() == std::vector<uint64_t>{3}, "invalid T4Scene object dimensions");
  const auto length = dimensions.get(1);
  auto width = dimensions.get(0);
  if constexpr (std::is_same_v<Group, t4scene::CylinderAnnotations>) {
    // Measured sides stay separate. A missing width uses the measured diameter.
    if (!width) {
      width = length;
    }
  }
  if (!width || !length) {
    return std::nullopt;
  }
  require(
    std::isfinite(*width) && *width >= 0.0 && std::isfinite(*length) && *length >= 0.0,
    "invalid T4Scene object width or length");
  return AgentCandidate{
    *group.track_ids->at(i), std::hypot((*pose)(0, 3), (*pose)(1, 3)), *label,
    static_cast<float>(*width), static_cast<float>(*length)};
}

std::vector<std::string> add_neighbor_tensors(
  TensorMap & tensors, const t4scene::T4Scene & scene, const size_t first,
  const Eigen::Matrix4d & scene_to_ego)
{
  constexpr auto kNeighbors = static_cast<size_t>(MAX_NUM_NEIGHBORS);
  auto past =
    xt::xarray<float>(xt::zeros<float>({kNeighbors, kInputRows, static_cast<size_t>(POSE_DIM)}));
  auto shapes = xt::xarray<float>(xt::zeros<float>({kNeighbors, size_t{2}}));
  auto labels = xt::xarray<float>(xt::zeros<float>({kNeighbors, size_t{3}}));
  std::vector<std::string> track_ids;

  if (scene.annotations && scene.annotations->available.at(scene.current_index)) {
    const auto current = scene.current_index;
    const auto anchor = scene.timestamps_ns.at(current);
    const auto & annotations = *scene.annotations;
    auto rows =
      observation_rows(scene, first, current, annotations.available, annotations.timestamps_ns);
    rows.erase(rows.upper_bound(anchor), rows.end());
    const auto current_stamp =
      measurement_time(value_at(annotations.timestamps_ns, current), anchor);
    if (current_stamp <= anchor) {
      rows.insert_or_assign(current_stamp, current);
      std::vector<AgentCandidate> candidates;
      std::unordered_set<std::string> seen;
      const auto select = [&](const auto & group, const size_t i) {
        if (auto candidate = candidate_at(group, i, scene_to_ego)) {
          if (seen.insert(candidate->track_id).second) {
            candidates.push_back(std::move(*candidate));
          }
        }
      };
      visit_group(annotations.values.boxes, annotations.box_offsets, current, select);
      visit_group(annotations.values.cylinders, annotations.cylinder_offsets, current, select);
      std::stable_sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
        return a.distance < b.distance;
      });
      candidates.resize(std::min(kNeighbors, candidates.size()));

      std::unordered_map<std::string, size_t> slots;
      for (size_t i = 0; i < candidates.size(); ++i) {
        const auto & agent = candidates[i];
        track_ids.push_back(agent.track_id);
        slots.emplace(agent.track_id, i);
        shapes(i, 0) = agent.width;
        shapes(i, 1) = agent.length;
        labels(i, agent.label) = 1.0F;
      }

      using Observation = std::pair<int64_t, ModelPose>;
      std::vector<std::vector<Observation>> histories(candidates.size());
      for (const auto & [stamp, row] : rows) {
        const auto collect = [&](const auto & group, const size_t i) {
          if (!group.track_ids || !group.track_ids->at(i)) {
            return;
          }
          const auto slot = slots.find(*group.track_ids->at(i));
          if (slot != slots.end()) {
            if (const auto pose = object_pose(group, i, scene_to_ego)) {
              histories[slot->second].emplace_back(stamp, model_pose(*pose));
            }
          }
        };
        visit_group(annotations.values.boxes, annotations.box_offsets, row, collect);
        visit_group(annotations.values.cylinders, annotations.cylinder_offsets, row, collect);
        visit_group(annotations.values.polygons, annotations.polygon_offsets, row, collect);
      }

      for (size_t slot = 0; slot < histories.size(); ++slot) {
        const auto & observations = histories[slot];
        auto next = observations.begin();
        for (size_t i = 0; i < kInputRows; ++i) {
          const auto grid_time = scene.timestamps_ns.at(current - INPUT_T + i);
          while (next != observations.end() && eligible_for_grid(next->first, grid_time)) {
            ++next;
          }
          if (next != observations.begin()) {
            const auto & pose = std::prev(next)->second;
            std::copy(pose.begin(), pose.end(), past.begin() + (slot * kInputRows + i) * POSE_DIM);
          }
        }
      }
    }
  }
  tensors.emplace("neighbor_agents_past", std::move(past));
  tensors.emplace("agent_shape", std::move(shapes));
  tensors.emplace("agent_label", std::move(labels));
  return track_ids;
}

using SignalHistory = std::unordered_map<
  int64_t, std::vector<std::pair<int64_t, std::vector<TrafficLightObservation>>>>;

SignalHistory signal_history(
  const t4scene::T4Scene & scene, const size_t first, const size_t last,
  const bool trim_history = true)
{
  SignalHistory history;
  if (!scene.traffic_lights) {
    return history;
  }
  const auto & sequence = *scene.traffic_lights;
  const auto & data = sequence.values;
  const auto rows =
    observation_rows(scene, first, last, sequence.available, sequence.timestamps_ns, trim_history);
  for (const auto & [stamp, row] : rows) {
    const auto first_group = sequence.group_offsets.at(row);
    const auto last_group = sequence.group_offsets.at(row + 1);
    require(
      first_group >= 0 && last_group >= first_group, "invalid T4Scene traffic light group offsets");
    for (auto group = first_group; group < last_group; ++group) {
      require(
        data.group_ids && data.element_offsets,
        "T4Scene traffic light groups require IDs and offsets");
      const auto first_element = data.element_offsets->at(group);
      const auto last_element = data.element_offsets->at(group + 1);
      require(
        first_element >= 0 && last_element >= first_element,
        "invalid T4Scene traffic light element offsets");
      std::vector<TrafficLightObservation> elements;
      elements.reserve(last_element - first_element);
      for (auto i = first_element; i < last_element; ++i) {
        const auto color = value_at(data.element_colors, i).value_or(0);
        const auto shape = value_at(data.element_shapes, i).value_or(0);
        const auto confidence = value_at(data.element_confidences, i).value_or(0.0);
        require(
          color >= 0 && color <= 4 && shape >= 0 && shape <= 10 && std::isfinite(confidence) &&
            confidence >= 0.0 && confidence <= 1.0,
          "invalid T4Scene traffic light element");
        // The model consumes a batch clock and cannot infer when independent
        // per-element clocks were jointly observed.
        require(
          value_at(data.element_timestamps_ns, i).value_or(stamp) == stamp,
          "T4Scene traffic light element time differs from its batch time");
        elements.push_back(
          {static_cast<TrafficLightColor>(color), static_cast<TrafficLightShape>(shape),
           static_cast<float>(confidence)});
      }
      history[data.group_ids->at(group)].emplace_back(stamp, std::move(elements));
    }
  }
  return history;
}

xt::xarray<float> signal_tensor(
  const t4scene::T4Scene & scene, const SignalHistory & history,
  const std::vector<std::pair<lanelet::Id, int64_t>> & lane_signals, const size_t start,
  const size_t count, const double timeout_s)
{
  const auto rows = static_cast<size_t>(NUM_SEGMENTS_IN_LANE);
  constexpr auto kChannels = static_cast<size_t>(TRAFFIC_LIGHT_ONE_HOT_DIM);
  auto result = xt::xarray<float>(xt::zeros<float>({rows, count, kChannels}));
  // Model channel indices for T4Scene colors unknown, red, amber, green and white.
  constexpr std::array<size_t, 5> kColorChannels{3, 2, 1, 0, 4};
  for (size_t slot = 0; slot < std::min(rows, lane_signals.size()); ++slot) {
    const auto & [id, turn_direction] = lane_signals[slot];
    const auto found = history.find(id);
    if (id == LaneSegment::TRAFFIC_LIGHT_ID_NONE || found == history.end()) {
      const size_t channel = id == LaneSegment::TRAFFIC_LIGHT_ID_NONE ? 4 : 3;
      for (size_t i = 0; i < count; ++i) {
        result(slot, i, channel) = 1.0F;
      }
      continue;
    }
    const auto & observations = found->second;
    auto next = observations.begin();
    for (size_t i = 0; i < count; ++i) {
      const auto grid_time = scene.timestamps_ns.at(start + i);
      while (next != observations.end() && eligible_for_grid(next->first, grid_time)) {
        ++next;
      }
      if (next == observations.begin()) {
        result(slot, i, 3) = 1.0F;
        continue;
      }
      const auto & [stamp, elements] = *std::prev(next);
      if (
        stamp <= grid_time &&
        static_cast<double>(static_cast<uint64_t>(grid_time) - static_cast<uint64_t>(stamp)) >
          timeout_s * 1'000'000'000.0) {
        result(slot, i, 3) = 1.0F;
        continue;
      }
      const auto selected = select_traffic_light(turn_direction, elements);
      result(slot, i, kColorChannels.at(static_cast<size_t>(selected.color))) = 1.0F;
      result(slot, i, 5) = selected.is_arrow ? 1.0F : 0.0F;
    }
  }
  return result;
}

std::vector<lanelet::Id> current_route_lane_ids(const t4scene::T4Scene & scene)
{
  require(scene.routes.has_value(), "T4Scene requires a current route");
  const auto route_index = scene.routes->indices.at(scene.current_index);
  require(route_index >= 0, "T4Scene requires a current route");
  const auto & route = scene.routes->rows.at(route_index);
  require(!route.segments.empty(), "T4Scene route has no segments");

  std::vector<lanelet::Id> lane_ids;
  lane_ids.reserve(route.segments.size());
  for (const auto & segment : route.segments) {
    require(segment.preferred_lane_id.has_value(), "T4Scene route has no preferred lane");
    require(
      std::find(segment.lane_ids.begin(), segment.lane_ids.end(), *segment.preferred_lane_id) !=
        segment.lane_ids.end(),
      "T4Scene preferred lane is not in its route segment");
    lane_ids.push_back(*segment.preferred_lane_id);
  }
  return lane_ids;
}
}  // namespace

SceneInputBuilderResult build_scene_input(
  const t4scene::T4Scene & scene, const LaneSegmentContext & map_context,
  const TrafficLightFuture traffic_light_future, const double traffic_light_timeout_s)
{
  try {
    const auto current = scene.current_index;
    require(
      current >= static_cast<size_t>(INPUT_T) && current < scene.size(),
      "ML-Planner requires 30 T4Scene history rows plus the anchor");
    require(
      std::isfinite(traffic_light_timeout_s) && traffic_light_timeout_s >= 0.0,
      "traffic light timeout must be finite and nonnegative");
    const auto first = current - std::min(current, kHistorySteps);
    auto last = current;
    if (traffic_light_future == TrafficLightFuture::Recorded) {
      require(
        scene.size() - current > kOutputRows,
        "recorded traffic lights require 80 future T4Scene rows");
      require(scene.traffic_lights.has_value(), "T4Scene has no recorded traffic light sequence");
      last += kOutputRows;
    }
    for (auto i = first + 1; i <= last; ++i) {
      const auto previous = scene.timestamps_ns.at(i - 1);
      require(
        previous <= std::numeric_limits<int64_t>::max() - kStepNanoseconds &&
          scene.timestamps_ns.at(i) == previous + kStepNanoseconds,
        "ML-Planner requires a regular 0.1 s T4Scene grid");
    }

    require(
      scene.map.has_value() && scene.ego.has_value(), "T4Scene requires a map and ego history");
    const auto ego_pose = pose_at(scene.ego->poses_3d, current);
    require(
      ego_pose && value_at(scene.ego->available, current).value_or(true),
      "T4Scene requires a known current ego pose");
    const auto goal_pose = pose_at(scene.goal_poses_3d, current);
    require(goal_pose.has_value(), "T4Scene requires a known current goal pose");

    const Eigen::Matrix4d ego_to_map = scene_to_map(scene) * *ego_pose;
    const Eigen::Matrix4d map_to_ego = Eigen::Isometry3d(ego_to_map).inverse().matrix();
    const Eigen::Matrix4d scene_to_ego = Eigen::Isometry3d(*ego_pose).inverse().matrix();

    TensorMap tensors;
    add_ego_tensors(tensors, scene, scene_to_ego);
    auto track_ids = add_neighbor_tensors(tensors, scene, first, scene_to_ego);

    const auto lane_segments =
      map_context.select_lane_segment_indices(map_to_ego, NUM_SEGMENTS_IN_LANE);
    auto [lanes, lane_types, speed_limits] =
      map_context.create_tensor_data_from_indices(map_to_ego, lane_segments, NUM_SEGMENTS_IN_LANE);
    tensors.emplace("lanes", std::move(lanes));
    tensors.emplace("lane_types", std::move(lane_types));
    tensors.emplace("lanes_speed_limit", std::move(speed_limits));

    const auto lane_signals = map_context.get_traffic_light_ids(lane_segments);
    auto past = signal_tensor(
      scene, signal_history(scene, first, current), lane_signals, current - INPUT_T, kInputRows,
      traffic_light_timeout_s);
    auto future = traffic_light_future == TrafficLightFuture::Recorded
                    ? signal_tensor(
                        scene, signal_history(scene, first, last, false), lane_signals, current + 1,
                        kOutputRows, traffic_light_timeout_s)
                    : infer_traffic_light_future(past);
    tensors.emplace("lane_traffic_light_past", std::move(past));
    tensors.emplace("lane_traffic_light_future", std::move(future));

    const auto route = current_route_lane_ids(scene);
    const double center_yaw = std::atan2(ego_to_map(1, 0), ego_to_map(0, 0));
    const auto route_segments = map_context.select_route_segment_indices(
      route, map_to_ego, static_cast<float>(ego_to_map(0, 3)), static_cast<float>(ego_to_map(1, 3)),
      static_cast<float>(ego_to_map(2, 3)), center_yaw, NUM_SEGMENTS_IN_ROUTE);
    tensors.emplace(
      "route_waypoints",
      map_context.create_route_waypoint_tensor(
        map_to_ego, route_segments, NUM_ROUTE_WAYPOINTS, ROUTE_WAYPOINT_SPACING_M));
    tensors.emplace("intersection_area", map_context.create_intersection_area_tensor(map_to_ego));
    tensors.emplace("stop_lines", map_context.create_stop_line_tensor(map_to_ego));
    tensors.emplace("road_borders", map_context.create_road_border_tensor(map_to_ego));

    const auto goal = model_pose(scene_to_ego * *goal_pose);
    tensors.emplace("goal_pose", xt::xarray<float>{goal[0], goal[1], goal[2], goal[3]});
    return SceneInputBuilderOutput{std::move(tensors), std::move(track_ids), ego_to_map};
  } catch (const std::logic_error & error) {
    return tl::unexpected(std::string(error.what()));
  }
}
}  // namespace autoware::ml_planner::preprocess
