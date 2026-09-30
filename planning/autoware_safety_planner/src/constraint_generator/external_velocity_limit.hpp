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

#ifndef CONSTRAINT_GENERATOR__EXTERNAL_VELOCITY_LIMIT_HPP_
#define CONSTRAINT_GENERATOR__EXTERNAL_VELOCITY_LIMIT_HPP_

#include "constraint_generator_interface.hpp"

#include <optional>
#include <string>

namespace autoware::safety_planner
{

//! Turns the speed limit given from outside (the API, the operator of the vehicle, an MRM) into the
//! VELOCITY bounds of the IR. It is the only source of the global speed bound, so it emits one on
//! every cycle, from its own parameter while no limit has arrived.
//!
//! A limit below the current speed is emitted in two parts, as the velocity_smoother of the rule
//! based stack did: the global bound is held at the current speed, and the limit itself comes as a
//! SpeedLimitZone over the reference path from the braking distance onwards. That split is only the
//! way in to a limit the ego does not meet yet; once the ego has been at or below the limit, the
//! bound alone carries it for the rest of the limit's life.
class ExternalVelocityLimitConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "external_velocity_limit"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;

private:
  //! The limit the ego has already been seen to meet, held until the limit itself changes
  std::optional<double> satisfied_limit_mps_{std::nullopt};
};

}  // namespace autoware::safety_planner

#endif  // CONSTRAINT_GENERATOR__EXTERNAL_VELOCITY_LIMIT_HPP_
