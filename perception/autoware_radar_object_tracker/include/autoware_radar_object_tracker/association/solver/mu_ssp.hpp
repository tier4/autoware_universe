// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE_RADAR_OBJECT_TRACKER__ASSOCIATION__SOLVER__MU_SSP_HPP_
#define AUTOWARE_RADAR_OBJECT_TRACKER__ASSOCIATION__SOLVER__MU_SSP_HPP_

#include "autoware_radar_object_tracker/association/solver/gnn_solver_interface.hpp"

#include <unordered_map>
#include <vector>

namespace autoware::radar_object_tracker::gnn_solver
{
class MuSSP : public GnnSolverInterface
{
public:
  MuSSP() = default;
  ~MuSSP() = default;

  void maximizeLinearAssignment(
    const std::vector<std::vector<double>> & cost, std::unordered_map<int, int> * direct_assignment,
    std::unordered_map<int, int> * reverse_assignment) override;
};
}  // namespace autoware::radar_object_tracker::gnn_solver

#endif  // AUTOWARE_RADAR_OBJECT_TRACKER__ASSOCIATION__SOLVER__MU_SSP_HPP_
