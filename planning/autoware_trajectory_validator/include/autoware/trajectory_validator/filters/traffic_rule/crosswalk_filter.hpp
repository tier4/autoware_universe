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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__CROSSWALK_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__CROSSWALK_FILTER_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>
namespace autoware::trajectory_validator::plugin::traffic_rule
{


struct CrosswalkOnTrajectory
{
  lanelet::CrosswalkConstPtr crosswalk;
  double arc_length_to_stop_line_m{0.0};
  lanelet::BasicLineString2d stop_line;

  CrosswalkOnTrajectory(
    lanelet::CrosswalkConstPtr crosswalk, double arc_length_to_stop_line_m, lanelet::BasicLineString2d stop_line)
  : crosswalk(crosswalk), arc_length_to_stop_line_m(arc_length_to_stop_line_m), stop_line(stop_line)
  {
  }
};

  struct TargetCrosswalk
{
  CrosswalkOnTrajectory crosswalk_info;
  lanelet::BasicPolygon2d crosswalk_polygon;

  TargetCrosswalk(
    CrosswalkOnTrajectory crosswalk_info, lanelet::BasicPolygon2d crosswalk_polygon)
  : crosswalk_info(crosswalk_info), crosswalk_polygon(crosswalk_polygon)
  {
  }
};

class CrosswalkFilter : public ValidatorInterface
{
public:
  CrosswalkFilter();

  result_t is_feasible(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) final;

  void update_parameters(const validator::Params & params) final;

  void set_vehicle_info(const VehicleInfo & vehicle_info) final;

private:
  validator::Params::Crosswalk params_;

  std::vector<TargetCrosswalk> get_target_crosswalks(const TrajectoryPoints & traj_points, const FilterContext & context);
};

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__TRAFFIC_RULE__CROSSWALK_FILTER_HPP_
