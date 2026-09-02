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

#ifndef CONSTRAINT_GENERATOR__VEHICLE_KINEMATICS_HPP_
#define CONSTRAINT_GENERATOR__VEHICLE_KINEMATICS_HPP_

#include "constraint_generator_interface.hpp"

#include <string>

namespace autoware::safety_planner
{

//! 車両運動の ScalarBound を生成するプラグイン。
//! 車両ハード上限 (v, a, j, a_lat, δ, δ̇) のみを発行する。δ は vehicle_info から取る。
//! 場所・時間によらない全域制約なので region / 時間窓は既定 (常時・全域) のまま
class VehicleKinematicsConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "vehicle_kinematics"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;
};

}  // namespace autoware::safety_planner

#endif  // CONSTRAINT_GENERATOR__VEHICLE_KINEMATICS_HPP_
