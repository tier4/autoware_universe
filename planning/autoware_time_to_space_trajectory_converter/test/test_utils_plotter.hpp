// Copyright 2025 TIER IV, Inc.
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

#ifndef TEST_UTILS_PLOTTER_HPP_
#define TEST_UTILS_PLOTTER_HPP_

#include "data_types.hpp"
#include "hermite_spline.hpp"

#include <autoware/pyplot/patches.hpp>
#include <autoware/pyplot/pyplot.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <string>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
void plot_trajectory(autoware::pyplot::Axes & ax, const std::vector<PlannerPoint> & trajectory);

void plot_and_save(
  const std::vector<PlannerPoint> & raw, const SplineData & knots, const std::string & test_dir,
  const std::string & filename);

void plot_hermite_spline(
  const HermiteSpline & spline, const std::vector<double> & bases,
  const std::vector<double> & values, const std::string & test_name);

void plot_stop_state_results(
  const std::string & title, const std::vector<double> & t_log, const std::vector<double> & x_log,
  const std::vector<double> & v_log, const std::vector<double> & wait_log, double stop_line_x,
  std::optional<double> required_wait, double stop_vel_thresh);

void plot_conversion_result(
  const autoware_planning_msgs::msg::Trajectory & input,
  const autoware_planning_msgs::msg::Trajectory & output, const std::string & filename);
}  // namespace autoware::time_to_space_trajectory_converter
#endif  // TEST_UTILS_PLOTTER_HPP_
