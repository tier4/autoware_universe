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

#ifndef AUTOWARE__SAFETY_PLANNER__CONSTRAINT_GENERATOR__VEHICLE_KINEMATICS_HPP_
#define AUTOWARE__SAFETY_PLANNER__CONSTRAINT_GENERATOR__VEHICLE_KINEMATICS_HPP_

#include "constraint_generator_interface.hpp"

#include <string>

namespace autoware::safety_planner
{

//! Emits the ScalarBound constraints of the vehicle kinematics: only the hard limits of the
//! vehicle (v, a, j, a_lat, steer angle, steer rate), the steer angle taken from vehicle_info.
//! They hold everywhere and at all times, so the region and the time window are left at default.
class VehicleKinematicsConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "vehicle_kinematics"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__CONSTRAINT_GENERATOR__VEHICLE_KINEMATICS_HPP_
