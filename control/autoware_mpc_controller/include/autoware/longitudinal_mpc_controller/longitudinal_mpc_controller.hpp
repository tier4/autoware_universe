// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use it except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__LONGITUDINAL_MPC_CONTROLLER__LONGITUDINAL_MPC_CONTROLLER_HPP_
#define AUTOWARE__LONGITUDINAL_MPC_CONTROLLER__LONGITUDINAL_MPC_CONTROLLER_HPP_

#include "autoware/trajectory_follower_base/longitudinal_controller_base.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include <array>
#include <memory>

class AcadosLongitudinalInterface;

namespace autoware::motion::control::longitudinal_mpc_controller
{

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

/// Longitudinal-only MPC using Acados (FOPDT model: s, v, a).
/// Outputs velocity (from reference) and acceleration (MPC u_cmd) for the trajectory follower.
class LongitudinalMpcController : public trajectory_follower::LongitudinalControllerBase
{
public:
  explicit LongitudinalMpcController(
    rclcpp::Node & node,
    std::shared_ptr<diagnostic_updater::Updater> diag_updater);

  ~LongitudinalMpcController();

  bool isReady(const trajectory_follower::InputData & input_data) override;
  trajectory_follower::LongitudinalOutput run(
    trajectory_follower::InputData const & input_data) override;

private:
  rclcpp::Logger logger_;
  double ctrl_period_;
  double tau_equiv_;
  double delay_compensation_time_;
  double accel_state_lpf_alpha_;
  double timeout_trajectory_sec_;
  double max_acc_;
  double min_acc_;
  double traj_resample_dist_;

  bool a_state_initialized_{false};
  double a_state_filtered_{0.0};
  double last_u_cmd_{0.0};

  std::unique_ptr<::AcadosLongitudinalInterface> acados_;
};

}  // namespace autoware::motion::control::longitudinal_mpc_controller

#endif  // AUTOWARE__LONGITUDINAL_MPC_CONTROLLER__LONGITUDINAL_MPC_CONTROLLER_HPP_
