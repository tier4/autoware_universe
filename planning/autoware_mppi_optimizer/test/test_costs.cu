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

#include "cost_test_report.hpp"
#include "projection_test_cases.hpp"

#include <mppi/core/mppi_common.cuh>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

#include <memory>
#include <utility>

namespace autoware::mppi_optimizer::cost_test
{
namespace
{
class CostEvaluation : public ::testing::Test
{
protected:
  std::unique_ptr<Cost> cost;
  Params params = disabledParams();
  Report report;
  std::array<float, H> ref_x{}, ref_y{}, ref_yaw{}, ref_v{};

  void SetUp() override
  {
    cost = std::make_unique<Cost>();
    apply();
    for (int i = 0; i < H; ++i) {
      ref_x[i] = 0.2F * i;
      ref_v[i] = 2.0F;
    }
    reference();
  }
  void TearDown() override
  {
    try {
      report.write(HasFailure(), IsSkipped());
    } catch (const std::exception & e) {
      ADD_FAILURE() << "Cost CSV export failed: " << e.what();
    }
  }
  void apply() { cost->setParams(params); }
  void reference(bool corridor = true)
  {
    cost->beginDataUpdate();
    cost->setReferenceTrajectory(ref_x.data(), ref_y.data(), ref_v.data(), H, ref_yaw.data());
    if (corridor)
      cost->setLateralCorridor(ref_x.data(), ref_y.data(), H, nullptr, ref_v.data());
    else
      cost->clearLateralCorridor();
    cost->commitDataUpdate();
  }
  void line(float x_end = 10.0F, float y_offset = 0.0F, float v_start = 2.0F, float v_end = 2.0F)
  {
    for (int i = 0; i < H; ++i) {
      float fraction = static_cast<float>(i) / (H - 1);
      ref_x[i] = fraction * x_end;
      ref_y[i] = y_offset;
      ref_yaw[i] = 0.0F;
      ref_v[i] = v_start + fraction * (v_end - v_start);
    }
    reference();
  }
  Breakdown record(
    const Cost::output_array & y, const Cost::control_array & u, int stage,
    const std::string & label, const std::string & kind = "sample")
  {
    int crash = 0;
    Breakdown b;
    double direct = 0.0;
    if (kind == "terminal") {
      b = cost->computeTerminalCostBreakdown(y);
      direct = b.total;
    } else if (kind != "initial") {
      b = cost->computeRunningCostBreakdown(y, u, stage, &crash);
      int direct_crash = 0;
      direct = cost->computeRunningCost(y, u, stage, &direct_crash);
      // Recorded below with this row's ID, including failure cases.
      report.rows.push_back({kind, label, stage, y, u, b, params, crash, direct});
      report.expect("direct_crash", direct_crash, crash, 0.0);
    }
    if (kind == "terminal" || kind == "initial")
      report.rows.push_back({kind, label, stage, y, u, b, params, crash, direct});
    report.expect("finite_output", y.allFinite(), 1, 0);
    report.expect("finite_control", u.allFinite(), 1, 0);
    double sum = 0;
    for (const auto & c : components) {
      const float value = b.*(c.value);
      report.expect(
        std::string("finite_nonnegative.") + c.name, std::isfinite(value) && value >= 0, 1, 0);
      sum += value;
    }
    report.expect("component_sum", sum, b.total, 1.0E-5 + 1.0E-5 * std::abs(b.total));
    if (kind != "terminal")
      report.expect("direct_total", direct, b.total, 1.0E-5 + 1.0E-5 * std::abs(b.total));
    if (kind == "sample" || kind == "running") {
      if (stage == 0) {
        report.expect("first_acceleration_command_change_omitted", b.acceleration_command_rate, 0);
        report.expect("first_steering_command_change_omitted", b.steering_command_rate, 0);
      } else {
        report.expect("initial_steering_anchor_only_at_stage_zero", b.initial_steering_rate, 0);
      }
    }
    report.capture(*cost);
    return b;
  }
  void expected(
    const Breakdown & b, float Breakdown::* member, const char * name, double value,
    double tol = 1.0E-4)
  {
    report.expect(std::string("cost.") + name, b.*member, value, tol);
  }
  void obstacle(
    float x, float y = 0, float yaw = 0, float half_length = 0.5F, float half_width = 0.5F)
  {
    cost->setOrientedBoxObstacles(&x, &y, &yaw, &half_length, &half_width, 1);
  }
};

struct QuadraticCase
{
  const char * name;
  float Params::* weight;
  float Breakdown::* component;
  int output_index;
  int control_index;
  bool terminal;
  float gain;
};
class QuadraticCosts : public CostEvaluation, public ::testing::WithParamInterface<QuadraticCase>
{
};
TEST_P(QuadraticCosts, ZeroSymmetryAndWeightScaling)
{
  const auto & c = GetParam();
  for (float weight : {0.0F, 1.0F, 3.0F})
    for (float value : {0.0F, 0.5F, -0.5F, 1.0F}) {
      params.*(c.weight) = weight;
      apply();
      auto y = Cost::output_array::Zero().eval();
      auto u = Cost::control_array::Zero().eval();
      if (c.terminal) y(static_cast<int>(O::BASELINK_POS_I_X)) = ref_x.back();
      if (c.output_index >= 0) y(c.output_index) = value;
      if (c.control_index >= 0) u(c.control_index) = value;
      const int t = c.terminal                                                      ? H - 1
                    : std::string(c.name).find("command_rate") != std::string::npos ? 1
                                                                                    : 0;
      const auto b = record(
        y, u, t,
        std::string(c.name) + " weight=" + std::to_string(weight) +
          " input=" + std::to_string(value),
        c.terminal ? "terminal" : "sample");
      const double want = weight * value * value * c.gain;
      expected(b, c.component, c.name, want, 1.0E-4 + std::abs(want) * 1.0E-5);
      report.expect("isolated_total", b.total, want, 1.0E-4 + std::abs(want) * 1.0E-5);
    }
}
#define QO(name, weight, output)                                                                  \
  QuadraticCase{#name, &Params::weight, &Breakdown::name, static_cast<int>(O::output), -1, false, \
                1.0F}
#define QC(name, weight, control)                                                                  \
  QuadraticCase{#name, &Params::weight, &Breakdown::name, -1, static_cast<int>(C::control), false, \
                1.0F}
INSTANTIATE_TEST_SUITE_P(
  Components, QuadraticCosts,
  ::testing::Values(
    QO(track, track_coeff, BASELINK_POS_I_Y), QO(heading, heading_coeff, YAW),
    QO(track_center, track_center_coeff, BASELINK_POS_I_Y),
    QO(lateral_distance, lateral_distance_coeff, BASELINK_POS_I_Y),
    QO(lateral_yaw_error, lateral_yaw_error_coeff, YAW),
    QO(steering_rate, steer_rate_coeff, STEERING_RATE),
    QO(lateral_jerk, lateral_jerk_coeff, LATERAL_JERK),
    QO(longitudinal_jerk, longitudinal_jerk_coeff, LONGITUDINAL_JERK),
    QO(acceleration_command_rate, accel_cmd_rate_coeff, ACCEL_COMMAND_RATE),
    QO(steering_command_rate, steer_cmd_rate_coeff, STEER_COMMAND_RATE),
    QC(acceleration_command, accel_cmd_coeff, ACCELERATION_CMD),
    QC(steering_command, steer_cmd_coeff, STEER_CMD),
    QuadraticCase{
      "initial_steering_rate", &Params::initial_steer_rate_coeff, &Breakdown::initial_steering_rate,
      -1, static_cast<int>(C::STEER_CMD), false, H / (dt * dt)},
    QuadraticCase{
      "terminal_error", &Params::terminal_error_coeff, &Breakdown::terminal_error,
      static_cast<int>(O::BASELINK_POS_I_Y), -1, true, 1.0F},
    QuadraticCase{
      "terminal_heading", &Params::terminal_heading_coeff, &Breakdown::terminal_heading,
      static_cast<int>(O::YAW), -1, true, 1.0F}),
  [](const ::testing::TestParamInfo<QuadraticCase> & p) { return p.param.name; });
#undef QO
#undef QC

TEST_F(CostEvaluation, SpatialOverspeedUsesCorridorProgressAndInterpolation)
{
  params.spatial_overspeed_coeff = 1;
  apply();
  line(10, 0, 2, 4);
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  for (float x : {0.0F, 5.0F, 10.0F})
    for (float excess : {-1.0F, 0.0F, 2.0F}) {
      y(static_cast<int>(O::BASELINK_POS_I_X)) = x;
      y(static_cast<int>(O::TOTAL_VELOCITY)) = 2 + 0.2F * x + excess;
      const auto b = record(
        y, u, 0, "spatial progress=" + std::to_string(x) + " excess=" + std::to_string(excess));
      expected(
        b, &Breakdown::spatial_overspeed, "spatial_overspeed",
        excess > 0 ? x / 10 * excess * excess : 0);
    }
  cost->clearLateralCorridor();
  expected(
    record(y, u, 0, "missing corridor"), &Breakdown::spatial_overspeed, "spatial_overspeed", 0);
}

TEST_F(CostEvaluation, PositionHeadingAndFootprintAnchors)
{
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  params.track_center_coeff = 1;
  params.ego_axle_to_box_center = 0.2F;
  apply();
  y(static_cast<int>(O::BASELINK_POS_I_Y)) = 1.5F;
  expected(
    record(y, u, 0, "center offset along x"), &Breakdown::track_center, "track_center", 2.29);
  y(static_cast<int>(O::YAW)) = static_cast<float>(M_PI_2);
  expected(
    record(y, u, 0, "rotated center offset"), &Breakdown::track_center, "track_center", 2.89);
  params = disabledParams();
  params.heading_coeff = 1;
  params.terminal_heading_coeff = 1;
  apply();
  ref_yaw.fill(static_cast<float>(M_PI) - 0.1F);
  reference();
  y(static_cast<int>(O::YAW)) = -static_cast<float>(M_PI) + 0.1F;
  expected(record(y, u, 0, "heading wrap"), &Breakdown::heading, "heading", 0.04);
  expected(
    record(y, u, H - 1, "terminal heading wrap", "terminal"), &Breakdown::terminal_heading,
    "terminal_heading", 0.04);
}

TEST_F(CostEvaluation, PathProgressOvershootAndDegenerateReference)
{
  line(10);
  params.remaining_distance_coeff = 1;
  params.path_overshoot_coeff = 1;
  params.lateral_distance_coeff = 1;
  apply();
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  for (float x : {0.0F, 5.0F, 10.0F, 12.0F}) {
    y(static_cast<int>(O::BASELINK_POS_I_X)) = x;
    y(static_cast<int>(O::BASELINK_POS_I_Y)) = 1;
    const auto b = record(y, u, 0, "progress x=" + std::to_string(x));
    expected(
      b, &Breakdown::remaining_distance, "remaining_distance", std::pow(std::max(0.0F, 10 - x), 2));
    expected(b, &Breakdown::path_overshoot, "path_overshoot", std::pow(std::max(0.0F, x - 10), 2));
    expected(b, &Breakdown::lateral_distance, "lateral_distance", 1);
  }
  ref_x.fill(0);
  ref_y.fill(0);
  reference();
  y.setZero();
  report.expect("degenerate_zero_total", record(y, u, 0, "coincident reference points").total, 0);
}

TEST_F(CostEvaluation, LateralAccelerationAndTerminalScaling)
{
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  params.lateral_acceleration_coeff = 1;
  apply();
  for (float v : {0.0F, 2.0F, 4.0F})
    for (float curvature : {-0.1F, 0.0F, 0.1F}) {
      y(static_cast<int>(O::BASELINK_VEL_B_X)) = v;
      y(static_cast<int>(O::STEER_ANGLE)) = std::atan(2 * curvature);
      expected(
        record(y, u, 0, "lateral acceleration"), &Breakdown::lateral_acceleration,
        "lateral_acceleration", v * v * v * v * curvature * curvature);
    }
  params = disabledParams();
  params.track_coeff = 2;
  params.track_terminal_scale = 3;
  params.terminal_error_coeff = 5;
  apply();
  float terminal[3] = {20, 0, 0};
  cost->setReferenceTrajectory(
    ref_x.data(), ref_y.data(), ref_v.data(), H, ref_yaw.data(), nullptr, nullptr, terminal);
  y.setZero();
  y(static_cast<int>(O::BASELINK_POS_I_X)) = ref_x.back();
  y(static_cast<int>(O::BASELINK_POS_I_Y)) = 1;
  const auto b = record(y, u, H - 1, "independent terminal target", "terminal");
  expected(b, &Breakdown::track, "track", 6);
  expected(b, &Breakdown::terminal_error, "terminal_error", 5 * (4.2 * 4.2 + 1), 0.001);
}

TEST_F(CostEvaluation, LateralBarrierThresholdsAreSymmetric)
{
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  params.lateral_boundary_soft_margin = 0.5F;
  for (float w : {0.0F, 1.0F, 3.0F}) {
    params.lateral_boundary_barrier_weight = w;
    apply();
    for (float offset : {0.0F, 1.49F, 1.5F, 1.51F, 2.0F, 2.1F})
      for (float sign : {-1.0F, 1.0F}) {
        y(static_cast<int>(O::BASELINK_POS_I_Y)) = sign * offset;
        const auto b = record(y, u, 0, "lateral clearance=" + std::to_string(2 - offset));
        expected(
          b, &Breakdown::lateral_boundary, "lateral_boundary",
          w * std::pow(std::max(0.0F, offset - 1.5F), 2));
        report.expect("lateral_crash", report.rows.back().crash, w > 0 && offset >= 2, 0);
      }
  }
}

TEST_F(CostEvaluation, GeometryClearanceSweeps)
{
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  // Four equal slices of a 4x2 ego rectangle have radius sqrt(0.5^2+1^2).
  // A wide obstacle / long horizontal wall gives an independent one-dimensional clearance.
  const float radius = std::sqrt(1.25F);
  for (int kind = 0; kind < 4; ++kind)
    for (float w : {0.0F, 1.0F, 3.0F}) {
      params = disabledParams();
      cost->clearObstacles();
      cost->setRoadBorderSegments({});
      cost->setDrivableAreaSegments({});
      float Params::* weight = kind == 0   ? &Params::obstacle_barrier_weight
                               : kind == 1 ? &Params::road_border_barrier_weight
                               : kind == 2 ? &Params::drivable_area_barrier_weight
                                           : &Params::corner_buffer_coeff;
      float Breakdown::* value = kind == 0   ? &Breakdown::obstacle
                                 : kind == 1 ? &Breakdown::road_border
                                 : kind == 2 ? &Breakdown::drivable_area
                                             : &Breakdown::corner_buffer;
      const char * name = kind == 0   ? "obstacle"
                          : kind == 1 ? "road_border"
                          : kind == 2 ? "drivable_area"
                                      : "corner_buffer";
      params.*weight = w;
      params.obstacle_safe_margin = 1;
      params.road_border_safe_margin = 1;
      params.drivable_area_safe_margin = 1;
      params.corner_safe_margin = 1;
      apply();
      for (float clearance : {1.1F, 1.0F, 0.99F, 0.5F, 0.0F, -0.1F}) {
        const float wall_y = (kind == 3 ? 1.0F : radius) + clearance;
        if (kind == 0)
          obstacle(0, wall_y + 0.5F, 0, 10, 0.5F);
        else if (kind == 1)
          cost->setRoadBorderSegments({Segment{-20, wall_y, 20, wall_y}});
        else
          cost->setDrivableAreaSegments({Segment{-20, wall_y, 20, wall_y}});
        const auto b =
          record(y, u, 0, std::string(name) + " clearance=" + std::to_string(clearance));
        // Road clearance clamps at contact; corner clearance is unsigned point-to-segment distance.
        const float effective = kind == 1   ? std::max(0.0F, clearance)
                                : kind == 3 ? std::abs(clearance)
                                            : clearance;
        const float want = w * std::pow(std::max(0.0F, 1 - effective), 2) * (kind == 3 ? 2 : 1);
        expected(b, value, name, want);
        if (kind < 2 && std::abs(clearance) > 0.01F)
          report.expect("soft_contact_flag", report.rows.back().crash, w > 0 && clearance < 0, 0);
      }
    }
  cost->setRoadBorderSegments({});
  cost->setDrivableAreaSegments({});
  cost->clearObstacles();
  params.obstacle_barrier_weight = params.road_border_barrier_weight =
    params.drivable_area_barrier_weight = params.corner_buffer_coeff = 1;
  apply();
  report.expect("empty_geometry", record(y, u, 0, "empty geometry").total, 0);
}

TEST_F(CostEvaluation, CollisionMarginRotationAndMovingObstacleTiming)
{
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  params.obstacle_barrier_weight = 1;
  params.obstacle_safe_margin = 1;
  apply();
  const float radius = std::sqrt(1.25F), clearance = 0.5F;
  obstacle(0, radius + clearance + 0.5F, 0, 10, 0.5F);
  expected(record(y, u, 0, "zero collision margin"), &Breakdown::obstacle, "obstacle", 0.25);
  params.obstacle_collision_margin = 0.2F;
  apply();
  expected(record(y, u, 0, "inflated collision margin"), &Breakdown::obstacle, "obstacle", 0.49);
  // Rotate both ego and scene 90 degrees: clearance and cost must be invariant.
  obstacle(-(radius + clearance + 0.5F), 0, static_cast<float>(M_PI_2), 10, 0.5F);
  y(static_cast<int>(O::YAW)) = static_cast<float>(M_PI_2);
  expected(record(y, u, 0, "rotated ego and obstacle"), &Breakdown::obstacle, "obstacle", 0.49);
  std::array<float, H> x{}, oy{}, yaw{};
  y.setZero();
  for (int t = 0; t < H; ++t) oy[t] = radius + 0.5F + 1.5F - static_cast<float>(t) / (H - 1);
  float half_length = 10, half_width = 0.5F;
  cost->setOrientedBoxObstacleTrajectories(
    x.data(), oy.data(), yaw.data(), &half_length, &half_width, 1, H);
  for (int t : {0, H / 2, H - 1}) {
    const float gap = 1.5F - static_cast<float>(t) / (H - 1);
    expected(
      record(y, u, t, "moving obstacle temporal clearance"), &Breakdown::obstacle, "obstacle",
      std::pow(std::max(0.0F, 1.2F - gap), 2));
    report.expect("moving_object_hard_clear", cost->egoIntersectsObstacleAtStep(0, 0, 0, t), 0, 0);
  }
  // Independent timing check: a small moving box meets the ego only at the last stored sample.
  half_length = half_width = 0.1F;
  oy.fill(0);
  for (int t = 0; t < H; ++t) x[t] = 20.0F * (1.0F - static_cast<float>(t) / (H - 1));
  cost->setOrientedBoxObstacleTrajectories(
    x.data(), oy.data(), yaw.data(), &half_length, &half_width, 1, H);
  record(y, u, 0, "moving object initial sample");
  report.expect("initial_obstacle_clear", cost->egoIntersectsObstacleAtStep(0, 0, 0, 0), 0, 0);
  record(y, u, H - 1, "moving object final sample");
  report.expect(
    "terminal_obstacle_contact", cost->egoIntersectsObstacleAtStep(0, 0, 0, H - 1), 1, 0);
}

TEST_F(CostEvaluation, KinematicBoundsDeadbandsScalingAndCap)
{
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  for (int kind = 0; kind < 3; ++kind) {
    FirstOrderDubinsBicycleKinematicLimitData limits;
    limits.active_mask = kind == 0   ? kVelocityLimitActive
                         : kind == 1 ? kAccelerationLimitActive
                                     : kJerkLimitActive;
    limits.min_velocity = limits.min_longitudinal_acceleration = limits.min_longitudinal_jerk = -1;
    limits.max_velocity = limits.max_longitudinal_acceleration = limits.max_longitudinal_jerk = 1;
    cost->setKinematicLimits(limits);
    const O field = kind == 0   ? O::BASELINK_VEL_B_X
                    : kind == 1 ? O::ACCELERATION
                                : O::LONGITUDINAL_JERK;
    auto member = kind == 0   ? &Breakdown::kinematic_velocity_overlimit
                  : kind == 1 ? &Breakdown::kinematic_acceleration_overlimit
                              : &Breakdown::kinematic_jerk_overlimit;
    const char * name = kind == 0   ? "kinematic_velocity_overlimit"
                        : kind == 1 ? "kinematic_acceleration_overlimit"
                                    : "kinematic_jerk_overlimit";
    const float normalization = kind == 0 ? 1.0F : kind == 1 ? 0.5F : 0.2F;
    for (float weight : {0.0F, 1.0F, 3.0F})
      for (float v : {-2.0F, -1.01F, -1.0F, -0.99F, 0.0F, 0.99F, 1.0F, 1.01F, 2.0F}) {
        params.overlimit_coeff = weight;
        apply();
        y.setZero();
        y(static_cast<int>(field)) = v;
        const auto b = record(y, u, 0, std::string(name) + " value=" + std::to_string(v));
        expected(
          b, member, name, weight * std::pow(std::max(0.0F, std::abs(v) - 1) * normalization, 2));
      }
    cost->setKinematicLimits({});
    expected(record(y, u, 0, "disabled bound"), member, name, 0);
  }
  FirstOrderDubinsBicycleKinematicLimitData limits;
  limits.active_mask = kVelocityLimitActive | kAccelerationLimitActive | kJerkLimitActive;
  cost->setKinematicLimits(limits);
  params.overlimit_coeff = 100;
  params.crash_contact_penalty = 10;
  apply();
  y(static_cast<int>(O::BASELINK_VEL_B_X)) = 10;
  y(static_cast<int>(O::ACCELERATION)) = 10;
  y(static_cast<int>(O::LONGITUDINAL_JERK)) = 10;
  const auto b = record(y, u, 0, "aggregate cap");
  report.expect(
    "kinematic_cap",
    b.kinematic_velocity_overlimit + b.kinematic_acceleration_overlimit +
      b.kinematic_jerk_overlimit,
    10);
}

TEST_F(CostEvaluation, PointwiseVelocityOverridesScalarBound)
{
  params.overlimit_coeff = 1;
  apply();
  FirstOrderDubinsBicycleKinematicLimitData limits;
  limits.active_mask = kVelocityLimitActive;
  limits.max_velocity = 4;
  cost->setKinematicLimits(limits);
  std::array<float, H> maximum{};
  maximum.fill(2);
  std::array<std::uint8_t, H> active{};
  active[1] = 1;
  cost->setReferenceTrajectory(
    ref_x.data(), ref_y.data(), ref_v.data(), H, ref_yaw.data(), maximum.data(), active.data());
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  y(static_cast<int>(O::BASELINK_VEL_B_X)) = 3;
  expected(
    record(y, u, 0, "scalar bound"), &Breakdown::kinematic_velocity_overlimit,
    "kinematic_velocity_overlimit", 0);
  expected(
    record(y, u, 1, "active pointwise bound"), &Breakdown::kinematic_velocity_overlimit,
    "kinematic_velocity_overlimit", 1);
  expected(
    record(y, u, 2, "inactive pointwise bound"), &Breakdown::kinematic_velocity_overlimit,
    "kinematic_velocity_overlimit", 0);
}

class CostRollouts : public CostEvaluation, public ::testing::WithParamInterface<std::string>
{
};
TEST_P(CostRollouts, PhysicalTrajectoryAndObjective)
{
  const auto name = GetParam();
  FirstOrderDubinsBicycleParams mp;
  mp.wheel_base = params.wheel_base;
  mp.accel_time_constant = mp.steer_time_constant = 0.2F;
  mp.max_steer_rate = 0.4F;
  if (name == "DelayedStep") {
    mp.acc_delay_steps = 2;
    mp.steer_delay_steps = 3;
  }
  Model model(mp);
  auto state = model.getZeroState();
  auto next = state;
  auto derivative = state;
  state(static_cast<int>(S::VEL_X)) = name == "StopStart" ? 0 : 2;
  // Circular enters the turn from straight steering; SteadyTurn starts at equilibrium.
  if (name == "SteadyTurn") state(static_cast<int>(S::STEER_ANGLE)) = 0.2F;
  for (const auto & c : components) params.*(c.weight) = 1;
  params.accel_time_constant = mp.accel_time_constant;
  params.steer_time_constant = mp.steer_time_constant;
  params.max_steer_rate = mp.max_steer_rate;
  apply();
  cost->setInitialSteeringAngle(state(static_cast<int>(S::STEER_ANGLE)));
  const double theta = dt * std::tan(0.2);  // yaw increment for v=2 and L=2
  for (int i = 0; i < H; ++i) {
    const int n = i + 1;
    if (name == "Circular" || name == "SteadyTurn") {
      // Closed form of the constant-turn forward-Euler orbit, independent of model.step().
      const double factor = 0.2 * std::sin(n * theta / 2) / std::sin(theta / 2);
      ref_x[i] = factor * std::cos((n - 1) * theta / 2);
      ref_y[i] = factor * std::sin((n - 1) * theta / 2);
      ref_yaw[i] = n * theta;
    } else {
      ref_x[i] = 0.2F * n;
      ref_y[i] = name == "Offset" ? 1 : 0;
      ref_yaw[i] = 0;
    }
  }
  reference();
  if (name == "Corridor") {
    const float wall = std::sqrt(1.25F) + 0.5F;
    params.road_border_safe_margin = 1;
    apply();
    cost->setRoadBorderSegments({Segment{-20, wall, 40, wall}, Segment{-20, -wall, 40, -wall}});
  }
  Cost::output_array y = Cost::output_array::Zero();
  auto u = Cost::control_array::Zero().eval();
  model.stateToOutput(state, y);
  record(y, u, 0, name, "initial");
  double running = 0;
  for (int t = 0; t < H; ++t) {
    u.setZero();
    if (name == "Circular" || name == "SteadyTurn") u(1) = 0.2F;
    if (name == "DelayedStep") {
      u(0) = 0.5F;
      u(1) = 0.1F;
    }
    if (name == "Alternating") {
      u(0) = t % 2 ? -0.1F : 0.1F;
      u(1) = t % 2 ? -0.02F : 0.02F;
    }
    if (name == "StopStart") u(0) = t < 20 ? 1 : t < 45 ? -1 : 0;
    model.enforceConstraints(state, u);
    model.step(state, next, derivative, u, y, t * dt, dt);
    const auto b = record(y, u, t, name, "running");
    running += b.total;
    report.captureModel(mp, state);
    report.expect(
      "physical_jerk_increment", y(static_cast<int>(O::LONGITUDINAL_JERK)),
      (next(static_cast<int>(S::ACCELERATION)) - state(static_cast<int>(S::ACCELERATION))) / dt);
    report.expect(
      "physical_steer_increment", y(static_cast<int>(O::STEERING_RATE)),
      (next(static_cast<int>(S::STEER_ANGLE)) - state(static_cast<int>(S::STEER_ANGLE))) / dt);
    if (t > 0) {
      expected(
        b, &Breakdown::acceleration_command_rate, "acceleration_command_rate",
        std::pow((u(0) - state(static_cast<int>(S::PREVIOUS_ACCEL_CMD))) / dt, 2));
      expected(
        b, &Breakdown::steering_command_rate, "steering_command_rate",
        std::pow((u(1) - state(static_cast<int>(S::PREVIOUS_STEER_CMD))) / dt, 2));
    }
    report.expect("nonnegative_velocity", next(static_cast<int>(S::VEL_X)) >= 0, 1, 0);
    report.expect(
      "steer_rate_limit",
      std::abs(y(static_cast<int>(O::STEERING_RATE))) <= mp.max_steer_rate + 1.0E-5F, 1, 0);
    if (name == "Straight" || name == "SteadyTurn") {
      expected(b, &Breakdown::track, "track", 0, 1.0E-4);
      expected(b, &Breakdown::longitudinal_jerk, "longitudinal_jerk", 0);
      expected(b, &Breakdown::steering_rate, "steering_rate", 0);
      expected(b, &Breakdown::steering_command_rate, "steering_command_rate", 0);
      expected(b, &Breakdown::initial_steering_rate, "initial_steering_rate", 0);
    }
    if (name == "Circular") {
      // Independent closed-form response for u=0.2, tau=0.2, dt=0.1, rate limit=0.4:
      // four 0.04-rad increments, then the remaining steering error halves each step.
      const auto steering_after = [](int stage) {
        if (stage < 0) return 0.0;
        return stage < 4 ? 0.04 * (stage + 1) : 0.2 - 0.04 * std::pow(0.5, stage - 3);
      };
      const double previous_steering = steering_after(t - 1);
      const double physical_rate = t < 4 ? 0.4 : 0.2 * std::pow(0.5, t - 4);
      const double lateral_jerk = 2 * physical_rate / std::pow(std::cos(previous_steering), 2);
      report.expect(
        "turn_entry_actuator_response", y(static_cast<int>(O::STEER_ANGLE)), steering_after(t));
      expected(b, &Breakdown::steering_rate, "steering_rate", physical_rate * physical_rate);
      expected(b, &Breakdown::lateral_jerk, "lateral_jerk", lateral_jerk * lateral_jerk);
      expected(b, &Breakdown::longitudinal_jerk, "longitudinal_jerk", 0);
      expected(b, &Breakdown::steering_command_rate, "steering_command_rate", 0);
      expected(b, &Breakdown::initial_steering_rate, "initial_steering_rate", t == 0 ? 320 : 0);
      if (t == H - 1) report.expect("turn_entry_tracking_offset", b.track > 0, 1, 0);
    }
    if (name == "Offset") expected(b, &Breakdown::lateral_distance, "lateral_distance", 1);
    if (name == "Corridor") expected(b, &Breakdown::road_border, "road_border", 0.25);
    if (name == "DelayedStep") {
      if (t < 2) expected(b, &Breakdown::longitudinal_jerk, "longitudinal_jerk", 0);
      if (t < 3) expected(b, &Breakdown::steering_rate, "steering_rate", 0);
      if (t == 2) expected(b, &Breakdown::longitudinal_jerk, "longitudinal_jerk", 6.25);
      if (t == 0) expected(b, &Breakdown::steering_command_rate, "steering_command_rate", 0);
    }
    state = next;
  }
  const auto terminal = record(y, u, H - 1, name, "terminal");
  double ledger = 0;
  for (const auto & row : report.rows) ledger += row.cost.total;
  report.expect("objective_divided_by_H", ledger / H, (running + terminal.total) / H, 1.0E-5);
}
INSTANTIATE_TEST_SUITE_P(
  Scenarios, CostRollouts,
  ::testing::Values(
    "Straight", "Circular", "SteadyTurn", "Offset", "StopStart", "DelayedStep", "Alternating",
    "Corridor"),
  [](const ::testing::TestParamInfo<std::string> & p) { return p.param; });

TEST_F(CostEvaluation, TerminalIsCountedOnceInHorizonObjective)
{
  params.terminal_error_coeff = 1;
  apply();
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  for (int t = 0; t < H; ++t) {
    y(static_cast<int>(O::BASELINK_POS_I_X)) = ref_x[t];
    y(static_cast<int>(O::BASELINK_POS_I_Y)) = t == H - 1 ? 3 : 0;
    report.expect(
      "running_zero", record(y, u, t, "prescribed terminal-error trajectory", "running").total, 0);
  }
  const auto b = record(y, u, H - 1, "independent terminal-only contribution", "terminal");
  expected(b, &Breakdown::terminal_error, "terminal_error", 9);
  double ledger = 0;
  for (const auto & r : report.rows) ledger += r.cost.total;
  report.expect("objective_divided_by_H", ledger / H, 0.1125);
}

TEST_F(CostEvaluation, QueuedCommandChangeDoesNotCreatePhysicalJerkOrJerkLimitCost)
{
  FirstOrderDubinsBicycleParams mp;
  mp.acc_delay_steps = mp.steer_delay_steps = 2;
  mp.accel_time_constant = mp.steer_time_constant = 0.2F;
  Model model(mp);
  auto state = model.getZeroState();
  state(static_cast<int>(S::VEL_X)) = 2.0F;
  state(static_cast<int>(S::ACCELERATION)) = 1.0F;
  state(static_cast<int>(S::STEER_ANGLE)) = 0.2F;
  state(static_cast<int>(S::PREVIOUS_ACCEL_CMD)) = 1.0F;
  state(static_cast<int>(S::PREVIOUS_STEER_CMD)) = 0.2F;
  for (int i = 0; i < 2; ++i) {
    state(static_cast<int>(S::ACCEL_CMD_D0) + i) = 1.0F;
    state(static_cast<int>(S::STEER_CMD_D0) + i) = 0.2F;
  }
  params.longitudinal_jerk_coeff = params.steer_rate_coeff = 1.0F;
  params.accel_cmd_rate_coeff = params.steer_cmd_rate_coeff = 1.0F;
  params.overlimit_coeff = 1.0F;
  apply();
  FirstOrderDubinsBicycleKinematicLimitData limits;
  limits.active_mask = kJerkLimitActive;
  limits.min_longitudinal_jerk = -1.0F;
  limits.max_longitudinal_jerk = 1.0F;
  cost->setKinematicLimits(limits);
  auto next = state;
  auto derivative = state;
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  u << 2.0F, -0.1F;
  for (int stage = 1; stage <= 3; ++stage) {
    model.step(state, next, derivative, u, y, stage * dt, dt);
    const auto b = record(y, u, stage, "queued command versus physical jerk", "running");
    report.captureModel(mp, state);
    if (stage <= 2) {
      expected(b, &Breakdown::longitudinal_jerk, "longitudinal_jerk", 0.0);
      expected(b, &Breakdown::steering_rate, "steering_rate", 0.0);
      expected(b, &Breakdown::kinematic_jerk_overlimit, "kinematic_jerk_overlimit", 0.0);
    } else {
      // The queued acceleration reaches the actuator: (2 - 1) / tau = 5 m/s^3.
      expected(b, &Breakdown::longitudinal_jerk, "longitudinal_jerk", 25.0);
      report.expect("jerk_limit_detects_realized_change", b.kinematic_jerk_overlimit > 0.0F, 1, 0);
    }
    expected(
      b, &Breakdown::acceleration_command_rate, "acceleration_command_rate",
      stage == 1 ? 100.0 : 0.0);
    expected(b, &Breakdown::steering_command_rate, "steering_command_rate", stage == 1 ? 9.0 : 0.0);
    state = next;
  }
}

TEST_F(CostEvaluation, ActuatorSaturationAndConstantTurnJerkConvention)
{
  FirstOrderDubinsBicycleParams mp;
  mp.wheel_base = 2;
  mp.accel_time_constant = mp.steer_time_constant = 0.05F;
  mp.max_accel = 1;
  mp.max_steer_angle = 0.45F;
  Model model(mp);
  auto state = model.getZeroState();
  auto next = state;
  auto derivative = state;
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  params.longitudinal_jerk_coeff = params.steer_rate_coeff = 1;
  params.accel_time_constant = mp.accel_time_constant;
  params.steer_time_constant = mp.steer_time_constant;
  params.max_steer_rate = mp.max_steer_rate;
  apply();
  state(static_cast<int>(S::ACCELERATION)) = 0.9F;
  state(static_cast<int>(S::STEER_ANGLE)) = 0.44F;
  u << 1, 0.45F;
  model.step(state, next, derivative, u, y, 0, dt);
  const auto b = record(y, u, 0, "actuator state saturation");
  report.captureModel(mp, state);
  expected(b, &Breakdown::longitudinal_jerk, "longitudinal_jerk", 1);
  expected(b, &Breakdown::steering_rate, "steering_rate", 0.01);
  state.setZero();
  state(static_cast<int>(S::VEL_X)) = 2;
  state(static_cast<int>(S::ACCELERATION)) = 1;
  state(static_cast<int>(S::STEER_ANGLE)) = std::atan(0.2F);
  u << 1, std::atan(0.2F);
  params.lateral_jerk_coeff = 1;
  apply();
  model.step(state, next, derivative, u, y, 0, dt);
  // Inertial lateral jerk: 3*v*a*kappa = 3*2*1*0.1 = 0.6, cost=0.36.
  expected(
    record(y, u, 0, "constant-turn accelerating inertial jerk"), &Breakdown::lateral_jerk,
    "lateral_jerk", 0.36);
  report.captureModel(mp, state);
}

template <class T>
struct DeviceBuffer
{
  T * data = nullptr;
  explicit DeviceBuffer(std::size_t n)
  {
    HANDLE_ERROR(cudaMalloc(reinterpret_cast<void **>(&data), n * sizeof(T)));
  }
  ~DeviceBuffer() { cudaFreeNoThrow(data); }
  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer & operator=(const DeviceBuffer &) = delete;
};
__global__ void evaluateDevice(
  Cost * cost, float * y, float * u, int t, float * result, const bool use_shared)
{
  extern __shared__ float theta[];
  cost->initializeCosts(y, u, theta, 0.0F, dt);
  int crash = 0;
  result[0] = cost->computeRunningCost(y, u, t, use_shared ? theta : nullptr, &crash);
  result[1] = cost->terminalCost(y, use_shared ? theta : nullptr);
  result[2] = static_cast<float>(crash);
}

__global__ void enableProjectionTexture(Cost * cost, const bool enabled)
{
  cost->texture_state_.nearest_segment_texture_valid_ = enabled;
}
class GpuCostEvaluation : public CostEvaluation
{
protected:
  void SetUp() override
  {
    int count = 0;
    if (cudaGetDeviceCount(&count) != cudaSuccess || count == 0)
      GTEST_SKIP() << "CUDA device required for parity";
    CostEvaluation::SetUp();
    cost->GPUSetup();
    reference();
    int device = 0, runtime = 0, driver = 0;
    cudaDeviceProp properties{};
    HANDLE_ERROR(cudaGetDevice(&device));
    HANDLE_ERROR(cudaGetDeviceProperties(&properties, device));
    HANDLE_ERROR(cudaRuntimeGetVersion(&runtime));
    HANDLE_ERROR(cudaDriverGetVersion(&driver));
    report.metadata = {
      {"backend", "CPU/GPU parity"},
      {"gpu", properties.name},
      {"cuda_runtime", std::to_string(runtime)},
      {"cuda_driver", std::to_string(driver)}};
  }
  void parity(
    const Cost::output_array & y, const Cost::control_array & u, int t, const std::string & label,
    double tolerance = 1.0E-4, const bool use_shared = false)
  {
    const auto b = record(y, u, t, label);
    DeviceBuffer<float> dy(Cost::OUTPUT_DIM), du(Cost::CONTROL_DIM), result(3);
    HANDLE_ERROR(
      cudaMemcpy(dy.data, y.data(), sizeof(float) * Cost::OUTPUT_DIM, cudaMemcpyHostToDevice));
    HANDLE_ERROR(
      cudaMemcpy(du.data, u.data(), sizeof(float) * Cost::CONTROL_DIM, cudaMemcpyHostToDevice));
    const auto shared_bytes = mppi::kernels::calcClassSharedMemSize(cost.get(), dim3(1, 1, 1));
    evaluateDevice<<<1, 1, shared_bytes>>>(
      cost->cost_d_, dy.data, du.data, t, result.data, use_shared);
    HANDLE_ERROR(cudaGetLastError());
    HANDLE_ERROR(cudaDeviceSynchronize());
    std::array<float, 3> actual{};
    HANDLE_ERROR(cudaMemcpy(actual.data(), result.data, sizeof(actual), cudaMemcpyDeviceToHost));
    report.expect("gpu_running_total", actual[0], b.total, tolerance + 1.0E-5 * std::abs(b.total));
    report.expect("gpu_crash", actual[2], report.rows.back().crash, 0);
    const auto terminal = record(y, u, H - 1, label, "terminal");
    report.expect(
      "gpu_terminal_total", actual[1], terminal.total,
      tolerance + 1.0E-5 * std::abs(terminal.total));
  }
};

TEST_F(GpuCostEvaluation, AdversarialProjectionCostsAndTerminalMatchHost)
{
  auto y = Cost::output_array::Zero().eval();
  const auto u = Cost::control_array::Zero().eval();
  y(static_cast<int>(O::YAW)) = 0.3F;
  y(static_cast<int>(O::TOTAL_VELOCITY)) = 5.0F;
  y(static_cast<int>(O::BASELINK_VEL_B_X)) = 5.0F;
  for (const auto & c : projection_test::cases()) {
    SCOPED_TRACE(c.name);
    std::vector<float> velocity(c.x.size());
    for (std::size_t i = 0; i < velocity.size(); ++i) {
      velocity[i] = 1.0F + 0.01F * static_cast<float>(i);
    }
    cost->setLateralCorridor(
      c.x.data(), c.y.data(), static_cast<int>(c.x.size()), nullptr, velocity.data());
    y(static_cast<int>(O::BASELINK_POS_I_X)) = c.query_x;
    y(static_cast<int>(O::BASELINK_POS_I_Y)) = c.query_y;
    for (const auto & component : components) {
      const std::string name = component.name;
      if (
        name != "lateral_distance" && name != "lateral_boundary" && name != "lateral_yaw_error" &&
        name != "remaining_distance" && name != "path_overshoot" && name != "spatial_overspeed") {
        continue;
      }
      params = disabledParams();
      params.*(component.weight) = 1.0F;
      apply();
      for (bool texture : {true, false}) {
        enableProjectionTexture<<<1, 1>>>(cost->cost_d_, texture);
        HANDLE_ERROR(cudaGetLastError());
        for (bool shared : {false, true}) {
          SCOPED_TRACE(texture);
          SCOPED_TRACE(shared);
          parity(y, u, 1, std::string(c.name) + " " + name, 1.0E-4, shared);
        }
      }
    }
  }
}

TEST_F(GpuCostEvaluation, CombinedAndSplitRolloutsUseGlobalProjection)
{
  // Deterministic zero controls isolate projection from random sampling. Both production
  // launchers use the same bicycle/cost specialization and the deployed block dimensions.
  using Sampler = mppi::sampling_distributions::GaussianDistribution<FirstOrderDubinsBicycleParams>;
  constexpr int rollouts = 32;
  Sampler::SAMPLING_PARAMS_T sampling_params;
  sampling_params.num_rollouts = rollouts;
  sampling_params.num_timesteps = H;
  sampling_params.num_distributions = 1;
  for (auto & sigma : sampling_params.std_dev) {
    sigma = 1.0F;
  }
  Sampler sampler(sampling_params);
  Model model;
  model.GPUSetup();
  sampler.GPUSetup();
  const auto mean = Eigen::Matrix<float, Model::CONTROL_DIM, H>::Zero().eval();
  sampler.copyImportanceSamplerToDevice(mean.data(), 0, true);
  HANDLE_ERROR(cudaMemset(
    sampler.getControlSample(0, 0, 0), 0, rollouts * H * Model::CONTROL_DIM * sizeof(float)));
  params = disabledParams();
  params.lateral_distance_coeff = 1.0F;
  apply();
  const float x[] = {0, 10, 10, 0};
  const float y[] = {0, 0, 10, 10};
  cost->setLateralCorridor(x, y, 4);
  auto state = model.getZeroState();
  state(static_cast<int>(S::POS_X)) = 1.0F;
  state(static_cast<int>(S::POS_Y)) = 9.0F;
  DeviceBuffer<float> initial(Model::STATE_DIM), totals(rollouts);
  DeviceBuffer<float> outputs(rollouts * H * Model::OUTPUT_DIM);
  HANDLE_ERROR(cudaMemcpy(
    initial.data, state.data(), Model::STATE_DIM * sizeof(float), cudaMemcpyHostToDevice));
  for (bool texture : {true, false}) {
    enableProjectionTexture<<<1, 1>>>(cost->cost_d_, texture);
    HANDLE_ERROR(cudaGetLastError());
    for (bool split : {false, true}) {
      SCOPED_TRACE(texture);
      SCOPED_TRACE(split);
      if (split) {
        mppi::kernels::launchSplitRolloutKernel(
          &model, cost.get(), &sampler, dt, H, rollouts, 1.0F, 1.0F, initial.data, outputs.data,
          totals.data, dim3(32, 2, 1), dim3(80, 1, 1), 0, true);
      } else {
        mppi::kernels::launchRolloutKernel(
          &model, cost.get(), &sampler, dt, H, rollouts, 1.0F, 1.0F, initial.data, totals.data,
          dim3(32, 2, 1), 0, true);
      }
      std::array<float, rollouts> actual{};
      HANDLE_ERROR(cudaMemcpy(actual.data(), totals.data, sizeof(actual), cudaMemcpyDeviceToHost));
      for (float total : actual) {
        // Stationary at (1,9): lateral error is 1 m for H running stages and terminal.
        EXPECT_NEAR(total, 1.0F + 1.0F / H, 1.0E-4F);
      }
    }
  }
}

TEST_F(GpuCostEvaluation, EveryComponentAndTerminalMatchesHost)
{
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  y(static_cast<int>(O::BASELINK_POS_I_X)) = 5;
  y(static_cast<int>(O::BASELINK_POS_I_Y)) = 0.3F;
  y(static_cast<int>(O::YAW)) = 0.2F;
  y(static_cast<int>(O::BASELINK_VEL_B_X)) = 4;
  y(static_cast<int>(O::TOTAL_VELOCITY)) = 4;
  y(static_cast<int>(O::STEER_ANGLE)) = 0.1F;
  y(static_cast<int>(O::ACCELERATION)) = 1;
  y(static_cast<int>(O::LONGITUDINAL_JERK)) = 2;
  y(static_cast<int>(O::LATERAL_JERK)) = 0.5F;
  y(static_cast<int>(O::STEERING_RATE)) = 0.2F;
  y(static_cast<int>(O::ACCEL_COMMAND_RATE)) = 1;
  y(static_cast<int>(O::STEER_COMMAND_RATE)) = 0.3F;
  u << 0.5F, 0.2F;
  FirstOrderDubinsBicycleKinematicLimitData limits;
  limits.active_mask = kVelocityLimitActive | kAccelerationLimitActive | kJerkLimitActive;
  cost->setKinematicLimits(limits);
  for (const auto & c : components) {
    params = disabledParams();
    params.*(c.weight) = 1;
    apply();
    parity(y, u, 0, std::string(c.name) + " stage0");
    parity(y, u, 1, std::string(c.name) + " stage1");
  }
}
TEST_F(GpuCostEvaluation, TextureGeometryCostsMatchWithinResolutionBound)
{
  params.obstacle_barrier_weight = params.road_border_barrier_weight =
    params.drivable_area_barrier_weight = params.corner_buffer_coeff = 1;
  params.obstacle_safe_margin = params.road_border_safe_margin = params.drivable_area_safe_margin =
    params.corner_safe_margin = 2;
  apply();
  cost->setRoadBorderSegments({Segment{-20, 2, 40, 2}});
  cost->setDrivableAreaSegments({Segment{-20, -2, 40, -2}});
  obstacle(6, 0, 0, 0.5F, 0.5F);
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  y(static_cast<int>(O::BASELINK_POS_I_X)) = 2;
  // These SDF faces are affine throughout the interpolation cells. Bound the texture's
  // 8-bit interpolation-fraction error conservatively by two coordinate errors, plus roundoff.
  // Seven quadratic terms (three spine clearances and four corners), each violation <= 2 m,
  // give |delta cost| <= 7*(4*epsilon + epsilon^2). General curved SDFs need a spatial bound too.
  const float resolution = std::max(
    cost->texture_state_.static_distance_map_grid_.resolution,
    cost->texture_state_.obstacle_distance_map_grid_.resolution);
  const double error = resolution / 128.0 + 1.0E-5;
  parity(y, u, 0, "texture geometry", 7 * (4 * error + error * error));
  y(static_cast<int>(O::BASELINK_POS_I_X)) = 1000;
  parity(y, u, 1, "out-of-map analytical fallback");
}

TEST_F(GpuCostEvaluation, MovingObstacleTextureUsesRequestedTimestep)
{
  params.obstacle_barrier_weight = 1;
  params.obstacle_safe_margin = 1;
  apply();
  std::array<float, H> x{}, oy{}, yaw{};
  for (int t = 0; t < H; ++t)
    oy[t] = std::sqrt(1.25F) + 0.5F + 1.5F - static_cast<float>(t) / (H - 1);
  float half_length = 10, half_width = 0.5F;
  cost->setOrientedBoxObstacleTrajectories(
    x.data(), oy.data(), yaw.data(), &half_length, &half_width, 1, H);
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  for (int t : {0, H / 2, H - 1}) {
    const float clearance = 1.5F - static_cast<float>(t) / (H - 1);
    expected(
      record(y, u, t, "moving obstacle analytical oracle"), &Breakdown::obstacle, "obstacle",
      std::pow(std::max(0.0F, 1 - clearance), 2));
    // Flat box faces have affine SDFs; allow interpolation quantization, far from contact.
    parity(y, u, t, "moving obstacle texture slice", 0.01);
  }
}

__global__ void replayDevice(
  Model * model, Cost * cost, float * initial, float * commands, float * outputs, float * costs)
{
  __shared__ float state[Model::STATE_DIM], next[Model::STATE_DIM], derivative[Model::STATE_DIM];
  __shared__ float y[Model::OUTPUT_DIM], u[Model::CONTROL_DIM];
  for (int i = threadIdx.y; i < Model::STATE_DIM; i += blockDim.y) state[i] = initial[i];
  __syncthreads();
  for (int t = 0; t < H; ++t) {
    for (int i = threadIdx.y; i < Model::CONTROL_DIM; i += blockDim.y)
      u[i] = commands[t * Model::CONTROL_DIM + i];
    __syncthreads();
    model->enforceConstraints(state, u);
    __syncthreads();
    model->step(state, next, derivative, u, y, nullptr, t * dt, dt);
    __syncthreads();
    if (threadIdx.y == 0) {
      int crash = 0;
      costs[t] = cost->computeRunningCost(y, u, t, nullptr, &crash);
      if (t == H - 1) costs[H] = cost->terminalCost(y, nullptr);
    }
    for (int i = threadIdx.y; i < Model::OUTPUT_DIM; i += blockDim.y)
      outputs[t * Model::OUTPUT_DIM + i] = y[i];
    __syncthreads();
    for (int i = threadIdx.y; i < Model::STATE_DIM; i += blockDim.y) state[i] = next[i];
    __syncthreads();
  }
}

__global__ void propagateSteeringRateDevice(Model * model, const float velocity, float * result)
{
  float state[Model::STATE_DIM];
  float next[Model::STATE_DIM];
  float derivative[Model::STATE_DIM];
  float output[Model::OUTPUT_DIM];
  float control[Model::CONTROL_DIM];
  for (int index = 0; index < Model::STATE_DIM; ++index) {
    state[index] = 0.0F;
    next[index] = 0.0F;
    derivative[index] = 0.0F;
  }
  for (int index = 0; index < Model::OUTPUT_DIM; ++index) output[index] = 0.0F;
  for (int index = 0; index < Model::CONTROL_DIM; ++index) control[index] = 0.0F;
  state[static_cast<int>(S::VEL_X)] = velocity;
  control[static_cast<int>(C::STEER_CMD)] = 0.45F;
  model->enforceConstraints(state, control);
  model->step(state, next, derivative, control, output, nullptr, 0.0F, dt);
  result[0] = output[static_cast<int>(O::STEERING_RATE)];
}

TEST_F(GpuCostEvaluation, VelocityDependentSteeringRatePropagationMatchesHost)
{
  FirstOrderDubinsBicycleParams mp;
  mp.wheel_base = 2.8F;
  mp.max_steer_rate = 5.0F;
  mp.max_lateral_jerk_mps3 = 2.5F;
  mp.standstill_steer_rate_lim = 0.15F;
  mp.restart_velocity_threshold_mps = 0.5F;
  mp.steer_time_constant = 0.01F;
  Model model(mp);
  model.GPUSetup();

  for (const float velocity : {0.2F, 25.0F}) {
    auto state = model.getZeroState();
    auto next = state;
    auto derivative = state;
    auto output = Model::output_array::Zero().eval();
    auto control = Model::control_array::Zero().eval();
    state(static_cast<int>(S::VEL_X)) = velocity;
    control(static_cast<int>(C::STEER_CMD)) = 0.45F;
    model.enforceConstraints(state, control);
    model.step(state, next, derivative, control, output, 0.0F, dt);

    const float expected = velocity < mp.restart_velocity_threshold_mps
                             ? mp.standstill_steer_rate_lim
                             : mp.max_lateral_jerk_mps3 * mp.wheel_base / (velocity * velocity);
    EXPECT_NEAR(output(static_cast<int>(O::STEERING_RATE)), expected, 1.0E-5F);

    DeviceBuffer<float> device_result(1U);
    propagateSteeringRateDevice<<<1, 1>>>(model.model_d_, velocity, device_result.data);
    HANDLE_ERROR(cudaGetLastError());
    HANDLE_ERROR(cudaDeviceSynchronize());
    float device_rate = 0.0F;
    HANDLE_ERROR(
      cudaMemcpy(&device_rate, device_result.data, sizeof(device_rate), cudaMemcpyDeviceToHost));
    EXPECT_NEAR(device_rate, expected, 1.0E-5F);
    EXPECT_NEAR(device_rate, output(static_cast<int>(O::STEERING_RATE)), 1.0E-5F);
  }
}

TEST_F(GpuCostEvaluation, DelayedSteadyTurnAndCommandStepReplayMatchesHost)
{
  FirstOrderDubinsBicycleParams mp;
  mp.wheel_base = 2;
  mp.acc_delay_steps = 2;
  mp.steer_delay_steps = 3;
  mp.accel_time_constant = mp.steer_time_constant = 0.2F;
  Model model(mp);
  model.GPUSetup();
  params.longitudinal_jerk_coeff = params.steer_rate_coeff = params.lateral_jerk_coeff = 1;
  params.accel_cmd_rate_coeff = params.steer_cmd_rate_coeff = params.initial_steer_rate_coeff = 3;
  params.accel_time_constant = mp.accel_time_constant;
  params.steer_time_constant = mp.steer_time_constant;
  params.max_steer_rate = mp.max_steer_rate;
  params.overlimit_coeff = 1.0F;
  apply();
  FirstOrderDubinsBicycleKinematicLimitData limits;
  limits.active_mask = kJerkLimitActive;
  limits.min_longitudinal_jerk = -1.0F;
  limits.max_longitudinal_jerk = 1.0F;
  cost->setKinematicLimits(limits);
  cost->setInitialSteeringAngle(0.2F);
  auto state = model.getZeroState();
  state(static_cast<int>(S::VEL_X)) = 2;
  state(static_cast<int>(S::STEER_ANGLE)) = 0.2F;
  for (int i = 0; i < 3; ++i) state(static_cast<int>(S::STEER_CMD_D0) + i) = 0.2F;
  // Leave PREVIOUS_* at zero deliberately: stage zero must still have no command-rate penalty.
  std::array<float, H * Model::CONTROL_DIM> commands{};
  for (int t = 0; t < H; ++t) {
    commands[2 * t] = t >= 10 ? 0.5F : 0;
    commands[2 * t + 1] = t >= 10 ? 0.25F : 0.2F;
  }
  DeviceBuffer<float> dx(Model::STATE_DIM), du(commands.size()), dy(H * Model::OUTPUT_DIM),
    dc(H + 1);
  HANDLE_ERROR(
    cudaMemcpy(dx.data, state.data(), sizeof(float) * Model::STATE_DIM, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(du.data, commands.data(), sizeof(commands), cudaMemcpyHostToDevice));
  replayDevice<<<1, dim3(1, 2, 1)>>>(
    model.model_d_, cost->cost_d_, dx.data, du.data, dy.data, dc.data);
  HANDLE_ERROR(cudaGetLastError());
  HANDLE_ERROR(cudaDeviceSynchronize());
  std::array<float, H * Model::OUTPUT_DIM> device_outputs{};
  std::array<float, H + 1> device_costs{};
  HANDLE_ERROR(
    cudaMemcpy(device_outputs.data(), dy.data, sizeof(device_outputs), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(
    cudaMemcpy(device_costs.data(), dc.data, sizeof(device_costs), cudaMemcpyDeviceToHost));
  auto next = model.getZeroState();
  auto derivative = next;
  auto y = Cost::output_array::Zero().eval();
  auto u = Cost::control_array::Zero().eval();
  double host_sum = 0, device_sum = 0;
  for (int t = 0; t < H; ++t) {
    u << commands[2 * t], commands[2 * t + 1];
    model.enforceConstraints(state, u);
    model.step(state, next, derivative, u, y, t * dt, dt);
    const auto b = record(y, u, t, "CUDA delayed steady turn", "running");
    report.captureModel(mp, state);
    for (int j = 0; j < Model::OUTPUT_DIM; ++j)
      report.expect(
        "gpu_output_" + std::to_string(j), device_outputs[t * Model::OUTPUT_DIM + j], y(j),
        1.0E-4 + std::abs(y(j)) * 1.0E-5);
    report.expect(
      "gpu_running_total", device_costs[t], b.total, 1.0E-4 + std::abs(b.total) * 1.0E-5);
    if (t < 10) {
      expected(b, &Breakdown::steering_command_rate, "steering_command_rate", 0);
      expected(b, &Breakdown::steering_rate, "steering_rate", 0);
    }
    if (t == 10) {
      expected(b, &Breakdown::longitudinal_jerk, "longitudinal_jerk", 0);
      expected(b, &Breakdown::steering_rate, "steering_rate", 0);
      expected(b, &Breakdown::acceleration_command_rate, "acceleration_command_rate", 75);
      expected(b, &Breakdown::steering_command_rate, "steering_command_rate", 0.75);
    }
    if (t == 10 || t == 11) {
      expected(b, &Breakdown::kinematic_jerk_overlimit, "kinematic_jerk_overlimit", 0);
    }
    if (t == 12) {
      expected(b, &Breakdown::longitudinal_jerk, "longitudinal_jerk", 6.25);
      report.expect("gpu_replay_jerk_limit_after_delay", b.kinematic_jerk_overlimit > 0.0F, 1, 0);
    }
    host_sum += b.total;
    device_sum += device_costs[t];
    state = next;
  }
  const auto terminal = record(y, u, H - 1, "CUDA terminal", "terminal");
  report.expect("gpu_terminal_total", device_costs[H], terminal.total, 1.0E-4);
  report.expect(
    "gpu_objective_divided_by_H", (device_sum + device_costs[H]) / H,
    (host_sum + terminal.total) / H, 1.0E-4);
}
}  // namespace
}  // namespace autoware::mppi_optimizer::cost_test
