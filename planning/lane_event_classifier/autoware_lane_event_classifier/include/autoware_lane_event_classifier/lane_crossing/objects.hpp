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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__OBJECTS_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__OBJECTS_HPP_

#include <autoware_lane_event_classifier/detail/lane_tracker.hpp>
#include <autoware_lane_event_classifier/types.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <string>
#include <vector>

namespace autoware::lane_event_classifier
{

/**
 * @brief The perceived-object half of the lane-crossing policy layer.
 *
 * A lane crossing is only meaningful when the ego has an obstacle to pass, so this class turns the
 * tracker's forward lane sequence and the per-cycle perceived objects into the candidate objects
 * the trajectory must go around: those touching the reference straight sequence ahead of the ego
 * within the window. It is a plain value type holding the object window and is injected into the
 * classifier (dependency injection), kept separate from the boundary geometry so each has a single
 * responsibility. The geometry layer decides whether the trajectory brackets one of these
 * candidates (out before it, back after it). See docs/lane_crossing.md, "Onset".
 */
class LaneCrossingObjects
{
public:
  explicit LaneCrossingObjects(double object_longitudinal_window_m);

  /** @brief The candidate objects for one cycle (returned by value; no out-params). */
  struct Result
  {
    // Map-frame poses of objects touching the reference straight sequence ahead of the ego within
    // the window - the candidates the trajectory must bracket to onset. An object need not be
    // static: a moving object the ego follows straight produces no departure, so it cannot onset a
    // crossing on its own; the bracket geometry, not a speed threshold, does the discrimination.
    std::vector<geometry_msgs::msg::Pose> candidate_object_poses;
    // Human-readable breakdown of the candidate-object scan for this cycle (debug logging only).
    std::string diagnostic;
  };

  /** @brief Observes the candidate objects the ego might cross to avoid this cycle. */
  [[nodiscard]] Result observe(const LaneTracker & tracker, const LaneEventInput & input) const;

private:
  using PredictedObject = autoware_perception_msgs::msg::PredictedObject;

  /** @brief Per-cycle tally of how the perceived objects sit relative to the forward lane sequence,
   * kept so the diagnostic can explain the candidate decision. */
  struct LaneSequenceScan
  {
    std::vector<geometry_msgs::msg::Pose>
      candidate_object_poses;           // touch sequence, ahead in window
    int lane_sequence_object_count{0};  // touch the lane sequence at all (any position)
    double nearest_ahead_m{-1.0};       // arc distance to the nearest object ahead (-1 when none)
    double nearest_ahead_speed_mps{0.0};
  };

  /** @brief Scans every object against the forward lane sequence and collects the candidate poses
   * plus the nearest-ahead diagnostic (returned by value). */
  [[nodiscard]] LaneSequenceScan scan_lane_sequence_objects(
    const lanelet::ConstLanelets & lane_sequence, const std::vector<PredictedObject> & objects,
    double ego_arc_length_m) const;

  /** @brief True when the object lies ahead of the ego within the longitudinal window. */
  [[nodiscard]] bool object_is_ahead_within_window(double arc_distance_ahead_m) const;

  double object_longitudinal_window_m_;  // ahead-of-ego arc window for a candidate object
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__OBJECTS_HPP_
