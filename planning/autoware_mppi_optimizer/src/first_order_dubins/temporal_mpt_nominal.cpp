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

#include "autoware/mppi_optimizer/detail/temporal_mpt_nominal.hpp"

#include <rclcpp/logging.hpp>
#include <temporal_mpt/path_tracking_solver.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::mppi_optimizer::detail
{
namespace
{
rclcpp::Logger mptNominalLogger()
{
  return rclcpp::get_logger("mppi_temporal_mpt_nominal");
}
}  // namespace

struct TemporalMptNominalSeeder::Impl
{
  temporal_mpt::PathTrackingSolver solver;
};

TemporalMptNominalSeeder::TemporalMptNominalSeeder() : impl_(std::make_unique<Impl>())
{
}

TemporalMptNominalSeeder::~TemporalMptNominalSeeder() = default;

TemporalMptNominalSeeder::TemporalMptNominalSeeder(TemporalMptNominalSeeder &&) noexcept = default;

TemporalMptNominalSeeder & TemporalMptNominalSeeder::operator=(
  TemporalMptNominalSeeder &&) noexcept = default;

void TemporalMptNominalSeeder::setBicycleParameters(
  const float wheel_base_m, const float rear_axle_to_cg_m, const float accel_time_constant_s,
  const float steer_time_constant_s, const float max_steer_rate_rad_s)
{
  const double wb = static_cast<double>(std::max(wheel_base_m, 1.0e-3F));
  double lr = static_cast<double>(rear_axle_to_cg_m);
  lr = std::clamp(lr, 1.0e-3, wb - 1.0e-3);
  const double lf = wb - lr;
  impl_->solver.setModelParameters(
    lf, lr, static_cast<double>(std::max(accel_time_constant_s, 1.0e-4F)),
    static_cast<double>(std::max(steer_time_constant_s, 1.0e-4F)),
    static_cast<double>(std::max(max_steer_rate_rad_s, 1.0e-6F)));
}

void TemporalMptNominalSeeder::resetWarmStart()
{
  impl_->solver.resetWarmStart();
}

void TemporalMptNominalSeeder::setWarmStartControls(
  const std::vector<float> & accel_cmd, const std::vector<float> & steer_cmd)
{
  std::vector<double> a(accel_cmd.begin(), accel_cmd.end());
  std::vector<double> d(steer_cmd.begin(), steer_cmd.end());
  impl_->solver.setWarmStartControls(a, d);
}

std::optional<std::vector<FirstOrderDubinsMppiControl>> TemporalMptNominalSeeder::solve(
  const Trajectory & reference, const InitialState & ego,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params, const int horizon)
{
  if (reference.points.size() < 2U || horizon <= 0) {
    return std::nullopt;
  }

  constexpr float kStoppedVelocityMps = 0.05F;
  if (std::abs(ego.velocity) < kStoppedVelocityMps) {
    impl_->solver.resetWarmStart();
  }

  temporal_mpt::PathTrackingReference mpt_ref;
  mpt_ref.x.reserve(reference.points.size());
  mpt_ref.y.reserve(reference.points.size());
  mpt_ref.yaw.reserve(reference.points.size());
  mpt_ref.v.reserve(reference.points.size());
  for (const auto & point : reference.points) {
    mpt_ref.x.push_back(point.pose.position.x);
    mpt_ref.y.push_back(point.pose.position.y);
    mpt_ref.yaw.push_back(tf2::getYaw(point.pose.orientation));
    mpt_ref.v.push_back(std::max(0.0, static_cast<double>(point.longitudinal_velocity_mps)));
  }

  temporal_mpt::PathTrackingInitialState x0;
  x0.x = ego.x;
  x0.y = ego.y;
  x0.yaw = ego.yaw;
  x0.v = std::max(0.0, static_cast<double>(ego.velocity));
  x0.accel = static_cast<double>(ego.acceleration);
  x0.steer = static_cast<double>(ego.steering);

  const temporal_mpt::PathTrackingResult mpt = impl_->solver.solve(x0, mpt_ref);
  if (!mpt.ok || mpt.accel_cmd.empty() || mpt.steer_cmd.size() != mpt.accel_cmd.size()) {
    impl_->solver.resetWarmStart();
    RCLCPP_WARN(
      mptNominalLogger(),
      "Temporal MPT nominal solve failed (status=%d); falling back to diffusion seed", mpt.status);
    return std::nullopt;
  }

  const int control_count = std::max(0, horizon);
  std::vector<FirstOrderDubinsMppiControl> nominal(static_cast<std::size_t>(control_count));
  const auto n_mpt = static_cast<int>(mpt.accel_cmd.size());
  for (int t = 0; t < control_count; ++t) {
    const int src = std::min(t, n_mpt - 1);
    nominal[static_cast<std::size_t>(t)].accel_cmd = std::clamp(
      static_cast<float>(mpt.accel_cmd[static_cast<std::size_t>(src)]), vehicle_params.min_accel(),
      vehicle_params.max_accel());
    nominal[static_cast<std::size_t>(t)].steer_cmd = std::clamp(
      static_cast<float>(mpt.steer_cmd[static_cast<std::size_t>(src)]),
      -vehicle_params.max_steer_angle, vehicle_params.max_steer_angle);
  }
  return nominal;
}

}  // namespace autoware::mppi_optimizer::detail
