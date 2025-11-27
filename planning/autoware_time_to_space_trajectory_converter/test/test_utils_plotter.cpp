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

#include "test_utils_plotter.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <fmt/format.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
static pybind11::scoped_interpreter guard{};

// -----------------------------------------------------------------------------
// Internal Helpers
// -----------------------------------------------------------------------------
namespace
{
void save_figure(const std::string & filename, const std::string & sub_dir)
{
  auto plt = pyplot::import();
  const std::string file_path = __FILE__;
  const std::string package_name = "autoware_time_to_space_trajectory_converter";

  // Find the package root to construct the correct output path
  size_t pos = file_path.rfind(package_name);
  if (pos != std::string::npos) {
    std::string output_path =
      file_path.substr(0, pos) + package_name + "/test_results/" + sub_dir + "/";

    // Directory creation is handled by CMake (create_test_directories target)
    plt.savefig(Args(output_path + filename), Kwargs("dpi"_a = 150));
  }
}

std::vector<double> extract_planner_field(
  const std::vector<PlannerPoint> & data, double PlannerPoint::* member)
{
  std::vector<double> res;
  res.reserve(data.size());
  for (const auto & p : data) res.push_back(p.*member);
  return res;
}

std::vector<double> extract_planner_pos(
  const std::vector<PlannerPoint> & data, double Vec3::* member)
{
  std::vector<double> res;
  res.reserve(data.size());
  for (const auto & p : data) res.push_back(p.pos.*member);
  return res;
}

std::pair<std::vector<double>, std::vector<double>> extract_msg_xy(
  const autoware_planning_msgs::msg::Trajectory & traj)
{
  std::vector<double> x;
  std::vector<double> y;
  x.reserve(traj.points.size());
  y.reserve(traj.points.size());

  for (const auto & p : traj.points) {
    x.push_back(p.pose.position.x);
    y.push_back(p.pose.position.y);
  }
  return {x, y};
}
}  // namespace

// -----------------------------------------------------------------------------
// Public Plotting Functions
// -----------------------------------------------------------------------------

void plot_trajectory(pyplot::Axes & ax, const std::vector<PlannerPoint> & trajectory)
{
  auto xs = extract_planner_pos(trajectory, &Vec3::x);
  auto ys = extract_planner_pos(trajectory, &Vec3::y);
  ax.plot(Args(xs, ys), Kwargs("color"_a = "grey", "linewidth"_a = 0.5));
}

void plot_and_save(
  const std::vector<PlannerPoint> & raw, const SplineData & knots, const std::string & test_dir,
  const std::string & filename)
{
  auto plt = pyplot::import();
  auto [fig, ax] = plt.subplots(2, 2, Kwargs("figsize"_a = std::vector<int>{12, 10}));

  // --- 1. Geometry (Y vs X) ---
  {
    ax[0].set_title(Args("Path Geometry (Knots)"));
    ax[0].grid(Args(true));

    auto rx = extract_planner_pos(raw, &Vec3::x);
    auto ry = extract_planner_pos(raw, &Vec3::y);
    ax[0].plot(
      Args(rx, ry),
      Kwargs("color"_a = "red", "marker"_a = ".", "linestyle"_a = "None", "label"_a = "Raw Input"));

    ax[0].plot(
      Args(knots.x, knots.y),
      Kwargs("color"_a = "blue", "marker"_a = "x", "label"_a = "Spline Knots"));
    ax[0].set_xlabel(Args("pose (x-axis)"));
    ax[0].set_ylabel(Args("pose (y-axis)"));
    ax[0].legend();
  }

  // --- 2. Velocity (V vs S) ---
  {
    ax[1].set_title(Args("Velocity Knots (V vs S)"));
    ax[1].grid(Args(true));

    // Calculate approximate S for raw input just for plotting
    std::vector<double> raw_s;
    double s = 0;
    raw_s.push_back(0);
    for (size_t i = 1; i < raw.size(); ++i) {
      s += (raw[i].v == 0.0 && raw[i - 1].v == 0.0)
             ? 0.0
             : std::hypot(raw[i].pos.x - raw[i - 1].pos.x, raw[i].pos.y - raw[i - 1].pos.y);
      raw_s.push_back(s);
    }
    auto raw_v = extract_planner_field(raw, &PlannerPoint::v);

    ax[1].plot(Args(raw_s, raw_v), Kwargs("color"_a = "red", "alpha"_a = 0.5, "label"_a = "Raw"));
    ax[1].plot(
      Args(knots.s, knots.v), Kwargs("color"_a = "blue", "marker"_a = "o", "label"_a = "Knots"));
    ax[1].legend();
    ax[1].set_xlabel(Args("arclength, s ([m])"));
    ax[1].set_ylabel(Args("velocity (m/s)"));
  }

  // --- 3. Time (T vs S) ---
  {
    ax[2].set_title(Args("Time Knots (T vs S)"));
    ax[2].grid(Args(true));
    ax[2].plot(Args(knots.s, knots.t), Kwargs("color"_a = "green", "marker"_a = "o"));
    ax[2].set_xlabel(Args("arclength, s ([m])"));
    ax[2].set_ylabel(Args("time from start (s)"));
  }

  // --- 4. Wait Times ---
  {
    ax[3].set_title(Args("Wait Times"));
    ax[3].stem(Args(knots.s, knots.wait_times));
    ax[3].set_xlabel(Args("arclength, s ([m])"));
    ax[3].set_ylabel(Args("wait times (s)"));
  }

  save_figure(filename, test_dir);
}

void plot_hermite_spline(
  const HermiteSpline & spline, const std::vector<double> & bases,
  const std::vector<double> & values, const std::string & test_name)
{
  // 1. Sample the spline finely
  std::vector<double> s_dense;
  std::vector<double> v_dense;
  std::vector<double> d1_dense;
  std::vector<double> d2_dense;
  const double start = bases.front();
  const double end = bases.back();
  const double step = 0.05;

  for (double s = start; s <= end; s += step) {
    s_dense.push_back(s);
    v_dense.push_back(spline.compute(s));
    d1_dense.push_back(spline.compute_first_derivative(s));
    d2_dense.push_back(spline.compute_second_derivative(s));
  }

  // 2. Setup Plot
  auto plt = pyplot::import();
  auto [fig, ax] =
    plt.subplots(3, 1, Kwargs("figsize"_a = std::vector<int>{10, 12}, "sharex"_a = true));

  // --- Plot 1: Value f(s) ---
  ax[0].set_title(Args("Value f(s)"));
  ax[0].grid(Args(true));
  ax[0].plot(Args(s_dense, v_dense), Kwargs("color"_a = "blue", "label"_a = "Spline"));
  ax[0].plot(
    Args(bases, values),
    Kwargs("color"_a = "red", "marker"_a = "o", "linestyle"_a = "None", "label"_a = "Knots"));
  ax[0].legend();

  // --- Plot 2: 1st Derivative f'(s) ---
  ax[1].set_title(Args("1st Derivative f'(s) [Slope]"));
  ax[1].grid(Args(true));
  ax[1].plot(Args(s_dense, d1_dense), Kwargs("color"_a = "green", "linestyle"_a = "--"));

  // --- Plot 3: 2nd Derivative f''(s) ---
  ax[2].set_title(Args("2nd Derivative f''(s) [Curvature]"));
  ax[2].grid(Args(true));
  ax[2].plot(Args(s_dense, d2_dense), Kwargs("color"_a = "orange", "linestyle"_a = ":"));
  ax[2].set_xlabel(Args("s"));

  save_figure(test_name + ".png", "test_hermite_spline");
}

void plot_stop_state_results(
  const std::string & title, const std::vector<double> & t_log, const std::vector<double> & x_log,
  const std::vector<double> & v_log, const std::vector<double> & wait_log, double stop_line_x,
  std::optional<double> required_wait, double stop_vel_thresh)
{
  if (t_log.empty()) return;

  auto plt = pyplot::import();
  auto [fig, ax] =
    plt.subplots(3, 1, Kwargs("figsize"_a = std::vector<int>{10, 12}, "sharex"_a = true));

  // Helper to draw horizontal lines
  auto draw_hline = [&](auto & axis, double y_val, std::string color, std::string label) {
    std::vector<double> line_t = {t_log.front(), t_log.back()};
    std::vector<double> line_y = {y_val, y_val};
    axis.plot(
      Args(line_t, line_y), Kwargs("color"_a = color, "linestyle"_a = "--", "label"_a = label));
  };

  // Plot 1: Position
  ax[0].set_title(Args(title + " - Position"));
  ax[0].plot(Args(t_log, x_log), Kwargs("label"_a = "Ego X", "color"_a = "blue"));
  draw_hline(ax[0], stop_line_x, "red", "Stop Line");
  ax[0].set_ylabel(Args("Position (m)"));
  ax[0].legend();
  ax[0].grid(Args(true));

  // Plot 2: Velocity
  ax[1].set_title(Args("Velocity"));
  ax[1].plot(Args(t_log, v_log), Kwargs("label"_a = "Velocity", "color"_a = "green"));
  draw_hline(ax[1], stop_vel_thresh, "orange", "Stop Thresh");
  ax[1].set_ylabel(Args("Velocity (m/s)"));
  ax[1].legend();
  ax[1].grid(Args(true));

  // Plot 3: Accumulated Wait
  ax[2].set_title(Args("Accumulated Wait Time"));
  ax[2].plot(Args(t_log, wait_log), Kwargs("label"_a = "Accumulated Wait", "color"_a = "purple"));
  if (required_wait) {
    draw_hline(ax[2], required_wait.value(), "red", "Required Wait");
  }
  ax[2].set_ylabel(Args("Wait Time (s)"));
  ax[2].set_xlabel(Args("Simulation Time (s)"));
  ax[2].legend();
  ax[2].grid(Args(true));

  save_figure(title + ".png", "test_stop_state_machine");
}

void plot_conversion_result(
  const autoware_planning_msgs::msg::Trajectory & input,
  const autoware_planning_msgs::msg::Trajectory & output, const std::string & filename)
{
  auto plt = pyplot::import();
  auto [fig, ax] = plt.subplots(1, 1, Kwargs("figsize"_a = std::vector<int>{10, 6}));

  auto [in_x, in_y] = extract_msg_xy(input);
  auto [out_x, out_y] = extract_msg_xy(output);

  // Plot Input (Sparse red dots)
  ax[0].plot(
    Args(in_x, in_y),
    Kwargs(
      "color"_a = "red", "marker"_a = "o", "linestyle"_a = "None", "label"_a = "Input (Sparse)"));

  // Plot Output (Dense blue line)
  ax[0].plot(
    Args(out_x, out_y),
    Kwargs(
      "color"_a = "blue", "marker"_a = ".", "markersize"_a = 2, "label"_a = "Output (Resampled)"));

  ax[0].set_title(Args("Trajectory Conversion Node Result"));
  ax[0].set_xlabel(Args("X [m]"));
  ax[0].set_ylabel(Args("Y [m]"));
  ax[0].legend();
  ax[0].grid(Args(true));

  save_figure(filename, "test_time_to_space_converter_node");
}

}  // namespace autoware::time_to_space_trajectory_converter
