// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__TRACKING_OBJECT_MERGER__ASSOCIATION__SOLVER__SSP_HPP_
#define AUTOWARE__TRACKING_OBJECT_MERGER__ASSOCIATION__SOLVER__SSP_HPP_

#include "autoware/tracking_object_merger/association/solver/gnn_solver_interface.hpp"

#include <unordered_map>
#include <vector>

namespace autoware::tracking_object_merger::gnn_solver
{
class SSP : public GnnSolverInterface
{
public:
  SSP() = default;
  ~SSP() = default;

  void maximizeLinearAssignment(
    const std::vector<std::vector<double>> & cost, std::unordered_map<int, int> * direct_assignment,
    std::unordered_map<int, int> * reverse_assignment) override
  {
    const bool sparse_cost = true;
    maximizeLinearAssignment(cost, direct_assignment, reverse_assignment, sparse_cost);
  }

  void maximizeLinearAssignment(
    const std::vector<std::vector<double>> & cost, std::unordered_map<int, int> * direct_assignment,
    std::unordered_map<int, int> * reverse_assignment, const bool sparse_cost = true);
};
}  // namespace autoware::tracking_object_merger::gnn_solver

#endif  // AUTOWARE__TRACKING_OBJECT_MERGER__ASSOCIATION__SOLVER__SSP_HPP_
