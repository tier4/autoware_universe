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

#include <autoware/trajectory_ranker/trajectory_ranker_wrapper.hpp>

#include <memory>
#include <utility>

namespace autoware::trajectory_ranker
{

TrajectoryRankerWrapper::TrajectoryRankerWrapper(
  rclcpp::Node & node,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_interface,
  VehicleInfo vehicle_info, std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: node_ptr_(&node),
  logger_(node.get_logger().get_child(interface_name_)),
  vehicle_info_(std::make_shared<VehicleInfo>(vehicle_info)),
  route_handler_(std::make_shared<RouteHandler>()),
  time_keeper_(std::move(time_keeper)),
  param_listener_(
    std::make_unique<trajectory_ranker_params::ParamListener>(node_parameters_interface))
{
  if (!time_keeper_) {
    throw std::runtime_error("TimeKeeper is required for TrajectoryRankerWrapper");
  }

  params_ = param_listener_->get_params();

  evaluator_ = std::make_shared<Evaluator>(
    route_handler_, vehicle_info_, node.get_logger(), params_.evaluation, node_ptr_);

  ranker_ptr_ = std::make_unique<TrajectoryRanker>(evaluator_, params_);
}

void TrajectoryRankerWrapper::update_parameters()
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger_, "Trajectory Ranker parameters are updated.");
  }
}

ScoredCandidateTrajectories TrajectoryRankerWrapper::rank_trajectories(
  const CandidateTrajectories & input_trajectories, const RankerContext & context)
{
  update_parameters();

  auto result = ranker_ptr_->process(input_trajectories, context);
  if (!result) {
    RCLCPP_ERROR(logger_, "Failed to rank trajectories: %s", result.error().c_str());
    return ScoredCandidateTrajectories();
  }

  return result.value();
}

}  // namespace autoware::trajectory_ranker
