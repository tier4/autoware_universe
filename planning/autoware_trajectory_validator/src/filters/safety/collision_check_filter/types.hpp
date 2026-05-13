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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__TYPES_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__TYPES_HPP_

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/expand.hpp>

#include <algorithm>
#include <cassert>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::MultiPoint2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

using TimeTrajectory = std::vector<double>;
using TravelDistanceTrajectory = std::vector<double>;
using PoseTrajectory = std::vector<geometry_msgs::msg::Pose>;
using FootprintTrajectory = std::vector<Polygon2d>;
using StepPolygonTrajectory = std::vector<Polygon2d>;
using IndexRange = std::pair<size_t, size_t>;
using TimeRange = std::pair<double, double>;

static constexpr double TIME_INDEX_EPSILON = 1e-3;

struct TrajectoryIdentification
{
  std::string classification;
  builtin_interfaces::msg::Time stamp{};
  unique_identifier_msgs::msg::UUID uuid{};
  std::string trajectory_type{};
  double acceleration{};

  TrajectoryIdentification() = default;
  explicit TrajectoryIdentification(std::string classification)
  : classification(std::move(classification))
  {
  }

  TrajectoryIdentification(
    const autoware_perception_msgs::msg::PredictedObject & object,
    const builtin_interfaces::msg::Time stamp, std::string trajectory_type = {},
    double acceleration = 0.0)
  : classification(autoware::object_recognition_utils::convertLabelToString(
      autoware::object_recognition_utils::getHighestProbLabel(object.classification))),
    stamp(stamp),
    uuid(object.object_id),
    trajectory_type(std::move(trajectory_type)),
    acceleration(acceleration)
  {
  }

  std::string object_id_string() const { return autoware_utils_uuid::to_hex_string(uuid); }
  std::string trajectory_id_string() const
  {
    return object_id_string() + "_" + trajectory_type + " acc: " + std::to_string(acceleration);
  }
};

struct DracArtifacts
{
  TrajectoryIdentification object_identification;
  double pet;
  double ttc;
  PoseTrajectory ego_trajectory;
  PoseTrajectory object_trajectory;
  Polygon2d ego_hull;
  Polygon2d object_hull;
};

struct PetArtifacts
{
  TrajectoryIdentification object_identification;
  double pet;
  double ttc;
  PoseTrajectory ego_trajectory;
  PoseTrajectory object_trajectory;
  Polygon2d ego_hull;
  Polygon2d object_hull;
};

struct RssArtifacts
{
  TrajectoryIdentification object_identification;
  double pet;
  double ttc;
  PoseTrajectory ego_trajectory;
  PoseTrajectory object_trajectory;
  Polygon2d ego_hull;
  Polygon2d object_hull;
};

struct BlackboardLogger
{
  std::vector<DracArtifacts> drac_artifacts;
  std::vector<PetArtifacts> pet_artifacts;
  std::vector<RssArtifacts> rss_artifacts;
};

}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__TYPES_HPP_
