// Copyright 2023 LeoDrive A.Ş. All rights reserved.
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

#ifndef AUTOWARE__PREDICTED_PATH_CHECKER__PREDICTED_PATH_CHECKER_NODE_HPP_
#define AUTOWARE__PREDICTED_PATH_CHECKER__PREDICTED_PATH_CHECKER_NODE_HPP_

#include "autoware/predicted_path_checker/predicted_path_checker_core.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::predicted_path_checker
{

class PredictedPathCheckerNode : public rclcpp::Node
{
public:
  explicit PredictedPathCheckerNode(const rclcpp::NodeOptions & node_options);

private:
  void check_vehicle_state(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void on_timer();

  std::unique_ptr<PredictedPathCheckerCore> core_;
  diagnostic_updater::Updater updater_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autoware::predicted_path_checker

#endif  // AUTOWARE__PREDICTED_PATH_CHECKER__PREDICTED_PATH_CHECKER_NODE_HPP_
