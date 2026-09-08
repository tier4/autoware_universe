// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer::cost_test
{
constexpr int H = 80;
constexpr float dt = 0.1F;
using Cost = FirstOrderDubinsBicycleCost<H>;
using Params = FirstOrderDubinsBicycleCostParams<H>;
using Model = FirstOrderDubinsBicycle;
using Breakdown = FirstOrderDubinsMppiCostBreakdown;
using O = FirstOrderDubinsBicycleParams::OutputIndex;
using S = FirstOrderDubinsBicycleParams::StateIndex;
using C = FirstOrderDubinsBicycleParams::ControlIndex;

struct Component
{
  const char * name;
  float Breakdown::* value;
  float Params::* weight;
};
// One registry drives disabled fixtures, CSV columns and completeness checks.
inline constexpr Component components[] = {
  {"spatial_overspeed", &Breakdown::spatial_overspeed, &Params::spatial_overspeed_coeff},
  {"track", &Breakdown::track, &Params::track_coeff},
  {"heading", &Breakdown::heading, &Params::heading_coeff},
  {"terminal_error", &Breakdown::terminal_error, &Params::terminal_error_coeff},
  {"terminal_heading", &Breakdown::terminal_heading, &Params::terminal_heading_coeff},
  {"lateral_distance", &Breakdown::lateral_distance, &Params::lateral_distance_coeff},
  {"lateral_boundary", &Breakdown::lateral_boundary, &Params::lateral_boundary_barrier_weight},
  {"lateral_yaw_error", &Breakdown::lateral_yaw_error, &Params::lateral_yaw_error_coeff},
  {"remaining_distance", &Breakdown::remaining_distance, &Params::remaining_distance_coeff},
  {"path_overshoot", &Breakdown::path_overshoot, &Params::path_overshoot_coeff},
  {"track_center", &Breakdown::track_center, &Params::track_center_coeff},
  {"corner_buffer", &Breakdown::corner_buffer, &Params::corner_buffer_coeff},
  {"drivable_area", &Breakdown::drivable_area, &Params::drivable_area_barrier_weight},
  {"obstacle", &Breakdown::obstacle, &Params::obstacle_barrier_weight},
  {"road_border", &Breakdown::road_border, &Params::road_border_barrier_weight},
  {"acceleration_command", &Breakdown::acceleration_command, &Params::accel_cmd_coeff},
  {"steering_command", &Breakdown::steering_command, &Params::steer_cmd_coeff},
  {"lateral_acceleration", &Breakdown::lateral_acceleration, &Params::lateral_acceleration_coeff},
  {"lateral_jerk", &Breakdown::lateral_jerk, &Params::lateral_jerk_coeff},
  {"longitudinal_jerk", &Breakdown::longitudinal_jerk, &Params::longitudinal_jerk_coeff},
  {"steering_rate", &Breakdown::steering_rate, &Params::steer_rate_coeff},
  {"initial_steering_rate", &Breakdown::initial_steering_rate, &Params::initial_steer_rate_coeff},
  {"acceleration_command_rate", &Breakdown::acceleration_command_rate,
   &Params::accel_cmd_rate_coeff},
  {"steering_command_rate", &Breakdown::steering_command_rate, &Params::steer_cmd_rate_coeff},
  {"kinematic_velocity_overlimit", &Breakdown::kinematic_velocity_overlimit,
   &Params::overlimit_coeff},
  {"kinematic_acceleration_overlimit", &Breakdown::kinematic_acceleration_overlimit,
   &Params::overlimit_coeff},
  {"kinematic_jerk_overlimit", &Breakdown::kinematic_jerk_overlimit, &Params::overlimit_coeff}};
inline Params disabledParams()
{
  Params p;
  for (const auto & c : components) p.*(c.weight) = 0.0F;
  p.track_terminal_scale = 1.0F;
  p.wheel_base = 2.0F;
  p.ego_length = 4.0F;
  p.ego_width = 2.0F;
  p.ego_axle_to_box_center = 0.0F;
  p.obstacle_collision_margin = 0.0F;
  p.road_border_collision_margin = 0.0F;
  p.boundary_threshold = 2.0F;
  return p;
}
struct Check
{
  std::size_t row;
  std::string name;
  double actual, expected, tolerance;
  bool passed;
};
struct Row
{
  std::string kind, label;
  int stage;
  Cost::output_array output;
  Cost::control_array control;
  Breakdown cost;
  Params params;
  int crash;
  double direct;
};
inline std::string quoted(const std::string & text)
{
  std::string result = "\"";
  for (char c : text) result += c == '"' ? "\"\"" : std::string(1, c);
  return result + '"';
}
inline std::ofstream csv(const std::filesystem::path & path)
{
  std::ofstream out;
  out.exceptions(std::ios::failbit | std::ios::badbit);
  out.open(path);
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  return out;
}
inline bool reporting()
{
  const char * flag = std::getenv("MPPI_REPORT");
  return flag && std::string(flag) != "0" && std::string(flag) != "";
}
inline const std::filesystem::path & reportDirectory()
{
  static const auto directory = [] {
    const char * root = std::getenv("MPPI_REPORT_DIR");
    std::filesystem::path base = root ? root : "/tmp/mppi_cost_reports";
    std::filesystem::create_directories(base);
    const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
    for (int suffix = 0;; ++suffix) {
      auto path = base / ("run_" + std::to_string(stamp) + "_" + std::to_string(suffix));
      if (std::filesystem::create_directory(path)) {
        std::cout << "MPPI cost report directory: " << path << '\n';
        return path;
      }
    }
  }();
  return directory;
}

// Capture references and geometry at evaluation time; later parameter sweeps cannot rewrite them.
class Report
{
public:
  std::vector<Row> rows;
  std::vector<Check> checks;
  std::vector<std::string> references, geometry, parameters;
  std::vector<std::pair<std::string, std::string>> metadata{{"backend", "CPU"}};

  void captureModel(const FirstOrderDubinsBicycleParams & p, const Model::state_array & pre_state)
  {
    if (!reporting()) return;
    const auto param = [&](const std::string & name, double value) {
      std::ostringstream line;
      line << std::setprecision(std::numeric_limits<double>::max_digits10) << rows.size() - 1U
           << ',' << name << ',' << value;
      parameters.push_back(line.str());
    };
#define REPORT_MODEL(field) param("model_" #field, p.field)
    REPORT_MODEL(wheel_base);
    REPORT_MODEL(steer_angle_scale);
    REPORT_MODEL(steer_command_angle_scale);
    REPORT_MODEL(accel_time_constant);
    REPORT_MODEL(steer_time_constant);
    REPORT_MODEL(max_steer_angle);
    REPORT_MODEL(max_steer_rate);
    REPORT_MODEL(min_accel);
    REPORT_MODEL(max_accel);
    REPORT_MODEL(prevent_reverse_velocity);
    REPORT_MODEL(acc_delay_steps);
    REPORT_MODEL(steer_delay_steps);
#undef REPORT_MODEL
    for (int i = 0; i < Model::STATE_DIM; ++i)
      param("pre_state_" + std::to_string(i), pre_state(i));
  }

  void expect(const std::string & name, double actual, double expected, double tolerance = 1.0E-5)
  {
    const bool passed =
      std::isfinite(actual) && std::isfinite(expected) && std::abs(actual - expected) <= tolerance;
    checks.push_back(
      {rows.empty() ? 0U : rows.size() - 1U, name, actual, expected, tolerance, passed});
    EXPECT_TRUE(passed) << name << ": actual=" << actual << " expected=" << expected
                        << " tolerance=" << tolerance;
  }

  void capture(const Cost & cost)
  {
    if (!reporting()) return;
    const auto row = rows.size() - 1U;
    const auto & data = cost.runtimeData();
    const auto add = [row](std::vector<std::string> & target, const auto & write) {
      std::ostringstream line;
      line << std::setprecision(std::numeric_limits<double>::max_digits10) << row << ',';
      write(line);
      target.push_back(line.str());
    };
    for (int i = 0; i < H; ++i) {
      add(references, [&](auto & s) {
        s << "reference," << i << ',' << data.ref_x_[i] << ',' << data.ref_y_[i] << ','
          << data.ref_yaw_[i] << ',' << data.ref_v_[i];
      });
    }
    for (int i = 0; i < data.num_lateral_corridor_points_; ++i) {
      add(references, [&](auto & s) {
        s << "corridor," << i << ',' << data.lateral_corridor_x_[i] << ','
          << data.lateral_corridor_y_[i] << ",0," << data.lateral_corridor_ref_velocity_[i];
      });
    }
    add(references, [&](auto & s) {
      s << "terminal,0," << data.terminal_reference_[0] << ',' << data.terminal_reference_[1] << ','
        << data.terminal_reference_[2] << ",0";
    });
    const int t = std::clamp(rows.back().stage, 0, H - 1);
    for (int i = 0; i < data.num_obstacles_; ++i) {
      add(geometry, [&](auto & s) {
        s << "obstacle," << i << ',' << data.obs_x_[i][t] << ',' << data.obs_y_[i][t] << ",0,0,"
          << data.obs_yaw_[i][t] << ',' << data.obs_half_length_[i] << ','
          << data.obs_half_width_[i] << ',' << data.obs_is_static_[i];
      });
    }
    for (int i = 0; i < data.num_road_border_segments_; ++i) {
      add(geometry, [&](auto & s) {
        s << "road_border," << i << ',' << data.road_border_x0_[i] << ',' << data.road_border_y0_[i]
          << ',' << data.road_border_x1_[i] << ',' << data.road_border_y1_[i] << ",0,0,0,1";
      });
    }
    for (int i = 0; i < data.num_drivable_area_segments_; ++i) {
      add(geometry, [&](auto & s) {
        s << "drivable_area," << i << ',' << data.drivable_area_x0_[i] << ','
          << data.drivable_area_y0_[i] << ',' << data.drivable_area_x1_[i] << ','
          << data.drivable_area_y1_[i] << ",0,0,0,1";
      });
    }
    const auto param = [&](const char * name, double value) {
      add(parameters, [&](auto & s) { s << name << ',' << value; });
    };
    const auto & p = rows.back().params;
#define REPORT_PARAM(field) param(#field, p.field)
    REPORT_PARAM(track_terminal_scale);
    REPORT_PARAM(wheel_base);
    REPORT_PARAM(ego_length);
    REPORT_PARAM(ego_width);
    REPORT_PARAM(ego_axle_to_box_center);
    REPORT_PARAM(boundary_threshold);
    REPORT_PARAM(lateral_boundary_soft_margin);
    REPORT_PARAM(obstacle_safe_margin);
    REPORT_PARAM(obstacle_collision_margin);
    REPORT_PARAM(road_border_safe_margin);
    REPORT_PARAM(road_border_collision_margin);
    REPORT_PARAM(drivable_area_safe_margin);
    REPORT_PARAM(corner_safe_margin);
    REPORT_PARAM(crash_contact_penalty);
    REPORT_PARAM(accel_time_constant);
    REPORT_PARAM(steer_time_constant);
    REPORT_PARAM(max_steer_rate);
#undef REPORT_PARAM
    param("initial_steering_angle", data.initial_steering_angle_);
    param("static_map_resolution", cost.texture_state_.static_distance_map_grid_.resolution);
    param("obstacle_map_resolution", cost.texture_state_.obstacle_distance_map_grid_.resolution);
#define REPORT_LIMIT(field) param(#field, data.kinematic_limits_.field)
    REPORT_LIMIT(active_mask);
    REPORT_LIMIT(min_velocity);
    REPORT_LIMIT(max_velocity);
    REPORT_LIMIT(min_longitudinal_acceleration);
    REPORT_LIMIT(max_longitudinal_acceleration);
    REPORT_LIMIT(min_longitudinal_jerk);
    REPORT_LIMIT(max_longitudinal_jerk);
#undef REPORT_LIMIT
    param(
      "pointwise_velocity_active",
      data.has_pointwise_velocity_limits_ && data.ref_velocity_limit_active_[t]);
    param("pointwise_max_velocity", data.ref_max_velocity_[t]);
  }

  void write(bool failed, bool skipped) const
  {
    if (!reporting()) return;
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::string name = std::string(info->test_suite_name()) + "." + info->name();
    for (char & c : name)
      if (c == '/' || c == '\\') c = '_';
    const auto base = reportDirectory() / name;
    auto steps = csv(base.string() + ".steps.csv");
    steps << "row,kind,label,stage,time_s,command_rate_valid,x,y,yaw,v,speed,steering,acceleration,"
             "steer_cmd,accel_cmd,physical_steer_rate,physical_lateral_jerk,physical_longitudinal_"
             "jerk,accel_command_rate,steer_command_rate,crash,direct_available,direct_total,total";
    for (const auto & c : components) steps << ",cost_" << c.name << ",weight_" << c.name;
    steps << '\n';
    for (std::size_t i = 0; i < rows.size(); ++i) {
      const auto & r = rows[i];
      const auto get = [&](O index) { return r.output(static_cast<int>(index)); };
      steps << i << ',' << r.kind << ',' << quoted(r.label) << ',' << r.stage << ',';
      if (r.kind != "sample") steps << (r.kind == "initial" ? 0.0 : (r.stage + 1) * dt);
      steps << ',' << (r.kind != "initial" && r.kind != "terminal" && r.stage > 0) << ','
            << get(O::BASELINK_POS_I_X) << ',' << get(O::BASELINK_POS_I_Y) << ',' << get(O::YAW)
            << ',' << get(O::BASELINK_VEL_B_X) << ',' << get(O::TOTAL_VELOCITY) << ','
            << get(O::STEER_ANGLE) << ',' << get(O::ACCELERATION) << ','
            << r.control(static_cast<int>(C::STEER_CMD)) << ','
            << r.control(static_cast<int>(C::ACCELERATION_CMD)) << ',' << get(O::STEERING_RATE)
            << ',' << get(O::LATERAL_JERK) << ',' << get(O::LONGITUDINAL_JERK) << ',';
      // Unknown pre-horizon command history must never appear as a measured spike.
      if (r.kind != "initial" && r.kind != "terminal" && r.stage > 0)
        steps << get(O::ACCEL_COMMAND_RATE);
      steps << ',';
      if (r.kind != "initial" && r.kind != "terminal" && r.stage > 0)
        steps << get(O::STEER_COMMAND_RATE);
      steps << ',' << r.crash << ',' << (r.kind == "sample" || r.kind == "running") << ','
            << r.direct << ',' << r.cost.total;
      for (const auto & c : components)
        steps << ',' << r.cost.*(c.value) << ',' << r.params.*(c.weight);
      steps << '\n';
    }
    steps.close();
    auto assertions = csv(base.string() + ".checks.csv");
    assertions << "row,check,actual,expected,tolerance,passed\n";
    for (const auto & c : checks)
      assertions << c.row << ',' << quoted(c.name) << ',' << c.actual << ',' << c.expected << ','
                 << c.tolerance << ',' << c.passed << '\n';
    assertions.close();
    const auto sidecar = [&](const char * suffix, const char * header, const auto & lines) {
      auto out = csv(base.string() + suffix);
      out << header << '\n';
      for (const auto & line : lines) out << line << '\n';
      out.close();
    };
    sidecar(".reference.csv", "row,kind,index,x,y,yaw,v", references);
    sidecar(
      ".geometry.csv", "row,kind,index,x0,y0,x1,y1,yaw,half_length,half_width,is_static", geometry);
    sidecar(".params.csv", "row,parameter,value", parameters);
    // Write metadata last: absence signals an incomplete export, not a passing scenario.
    auto meta = csv(base.string() + ".meta.csv");
    meta << "key,value\nschema_version,2\nhorizon," << H << "\ndt," << dt << "\nname,"
         << quoted(name) << "\nstatus,"
         << (skipped  ? "SKIPPED"
             : failed ? "FAIL"
                      : "PASS")
         << '\n';
    for (const auto & item : metadata)
      meta << quoted(item.first) << ',' << quoted(item.second) << '\n';
    meta.close();
  }
};
}  // namespace autoware::mppi_optimizer::cost_test
