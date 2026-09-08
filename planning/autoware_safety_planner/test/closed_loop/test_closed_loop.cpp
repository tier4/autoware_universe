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

#include "closed_loop_simulator.hpp"
#include "test_output_utils.hpp"
#include "test_plot_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef EXPORT_TEST_PLOT_FIGURE
#include <autoware_test_utils/visualization.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#endif

namespace autoware::safety_planner::testing
{

// One scenario = test_data/scenarios/<name>.yaml; it alone defines the map, the vehicle and the
// route
struct Scenario
{
  LaneletMapBin map_bin;
  VehicleInfo vehicle_info;
  Params params;
  LaneletRoute route;
  PredictedObjects predicted_objects;  //!< optional key "predicted_objects"; empty if absent
  //! optional key "expectation": "goal_reached" (default) or "stop" (ego must come to a halt
  //! before the goal, e.g. blocked by an obstacle)
  std::string expectation{"goal_reached"};
};

// Parameters and vehicle_info can only be read through a node (generate_parameter_library /
// VehicleInfoUtils), so one node is created just for reading them; it never subscribes or spins
Scenario load_scenario(const std::string & yaml_filename)
{
  const auto yaml_path = ament_index_cpp::get_package_share_directory("autoware_safety_planner") +
                         "/test_data/scenarios/" + yaml_filename;
  const auto config = YAML::LoadFile(yaml_path);

  // package://<pkg>/<path> or a plain filesystem path (for local maps that are not installed)
  const auto resolve = [](const std::string & uri) -> std::optional<std::string> {
    if (const auto resolved = autoware::test_utils::resolve_pkg_share_uri(uri)) {
      return resolved;
    }
    return std::filesystem::exists(uri) ? std::make_optional(uri) : std::nullopt;
  };
  const auto map_path = resolve(config["map_path_uri"].as<std::string>());
  const auto vehicle_info_param_path = resolve(config["vehicle_info_param_uri"].as<std::string>());
  if (!map_path || !vehicle_info_param_path) {
    throw std::runtime_error("failed to resolve map/vehicle uri in " + yaml_path);
  }

  // The production config: the core file plus one file per plugin
  // (config/<group>/<plugin>.param.yaml)
  std::vector<std::string> param_files{*vehicle_info_param_path};
  for (const auto & relative_path :
       {"safety_planner.param.yaml", "constraint_generator/vehicle_kinematics.param.yaml",
        "constraint_generator/lane_following_drivable_area.param.yaml",
        "constraint_generator/obstacle_stop.param.yaml",
        "constraint_generator/simple_drivable_area.param.yaml",
        "trajectory_planner/frenet_sampler.param.yaml",
        "trajectory_planner/rough_optimizer.param.yaml",
        "trajectory_planner/nlp_trajectory_optimizer.param.yaml",
        "trajectory_planner/ssc_qp_trajectory_optimizer.param.yaml"}) {
    param_files.push_back(
      autoware::test_utils::get_absolute_path_to_config("autoware_safety_planner", relative_path));
  }
  rclcpp::NodeOptions node_options;
  autoware::test_utils::updateNodeOptions(node_options, param_files);
  auto node = std::make_shared<rclcpp::Node>("safety_planner_closed_loop_test", node_options);

  Scenario scenario;
  scenario.map_bin = autoware::test_utils::make_map_bin_msg(*map_path);
  scenario.vehicle_info = vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();
  scenario.params =
    ::safety_planner::ParamListener(node->get_node_parameters_interface()).get_params();
  scenario.route = autoware::test_utils::parse<LaneletRoute>(config["route"]);
  if (config["predicted_objects"]) {
    scenario.predicted_objects =
      autoware::test_utils::parse<PredictedObjects>(config["predicted_objects"]);
  }
  if (config["expectation"]) {
    scenario.expectation = config["expectation"].as<std::string>();
  }
  return scenario;
}

class ClosedLoopTest : public ::testing::TestWithParam<std::string>
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

#ifdef EXPORT_TEST_PLOT_FIGURE
  static void plot_footprint(
    autoware::pyplot::Axes & ax, const Pose & pose, const VehicleInfo & vehicle_info,
    const std::string & color)
  {
    const double yaw = autoware_utils_geometry::get_rpy(pose).z;
    std::vector<double> xs, ys;
    for (const auto & p : vehicle_info.createFootprint()) {
      xs.push_back(pose.position.x + p.x() * std::cos(yaw) - p.y() * std::sin(yaw));
      ys.push_back(pose.position.y + p.x() * std::sin(yaw) + p.y() * std::cos(yaw));
    }
    ax.plot(Args(xs, ys), Kwargs("color"_a = color, "linewidth"_a = 0.6));
  }

  static void plot(
    const Scenario & scenario, const ClosedLoopSimulator & simulator,
    const ClosedLoopResult & result)
  {
    constexpr size_t footprint_interval_steps = 10;  // every 1 s

    auto plt = autoware::pyplot::import();
    auto [fig, axes] = plt.subplots(2, 1, Kwargs("figsize"_a = py::make_tuple(10, 12)));
    auto & ax_xy = axes[0];
    auto & ax_v = axes[1];

    if (simulator.input().route_manager) {
      const auto map = simulator.input().route_manager->lanelet_map_ptr();
      for (const auto & segment : scenario.route.segments) {
        for (const auto & primitive : segment.primitives) {
          autoware::test_utils::plot_lanelet2_object(
            map->laneletLayer.get(primitive.id), ax_xy,
            autoware::test_utils::LaneConfig{
              std::nullopt, autoware::test_utils::LineConfig{"k", "solid", 0.5}});
        }
      }
    }

    // Reference path and output trajectory of every cycle (thin lines), ego trace, footprints
    // at fixed intervals
    std::vector<double> ego_x, ego_y, t, v;
    for (size_t i = 0; i < result.steps.size(); ++i) {
      const auto & step = result.steps[i];
      std::vector<double> rx, ry;
      for (const auto & [x, y] : step.reference_path_xy) {
        rx.push_back(x);
        ry.push_back(y);
      }
      ax_xy.plot(
        Args(rx, ry), Kwargs("color"_a = "tab:green", "linewidth"_a = 0.3, "alpha"_a = 0.3));
      std::vector<double> tx, ty;
      for (const auto & p : step.trajectory.points) {
        tx.push_back(p.pose.position.x);
        ty.push_back(p.pose.position.y);
      }
      ax_xy.plot(
        Args(tx, ty), Kwargs("color"_a = "tab:blue", "linewidth"_a = 0.3, "alpha"_a = 0.3));
      ego_x.push_back(step.odometry.pose.pose.position.x);
      ego_y.push_back(step.odometry.pose.pose.position.y);
      t.push_back(static_cast<double>(i) * 0.1);
      v.push_back(step.odometry.twist.twist.linear.x);
      if (i % footprint_interval_steps == 0) {
        plot_footprint(ax_xy, step.odometry.pose.pose, scenario.vehicle_info, "tab:gray");
      }
    }
    for (const auto & object : scenario.predicted_objects.objects) {
      const auto polygon = autoware_utils_geometry::to_polygon2d(
        object.kinematics.initial_pose_with_covariance.pose, object.shape);
      std::vector<double> ox, oy;
      for (const auto & p : polygon.outer()) {
        ox.push_back(p.x());
        oy.push_back(p.y());
      }
      ax_xy.plot(Args(ox, oy), Kwargs("color"_a = "tab:orange", "linewidth"_a = 1.5));
    }
    const auto & final_pose = result.final_state.odometry.pose.pose;
    ego_x.push_back(final_pose.position.x);
    ego_y.push_back(final_pose.position.y);
    t.push_back(static_cast<double>(result.steps.size()) * 0.1);
    v.push_back(result.final_state.odometry.twist.twist.linear.x);
    plot_footprint(ax_xy, final_pose, scenario.vehicle_info, "tab:red");
    plot_footprint(ax_xy, scenario.route.goal_pose, scenario.vehicle_info, "tab:green");
    ax_xy.plot(
      Args(ego_x, ego_y),
      Kwargs("color"_a = "tab:red", "linewidth"_a = 1.5, "label"_a = "ego trace"));
    ax_xy.scatter(
      Args(scenario.route.start_pose.position.x, scenario.route.start_pose.position.y),
      Kwargs("color"_a = "green", "marker"_a = "o", "label"_a = "start"));
    ax_xy.scatter(
      Args(scenario.route.goal_pose.position.x, scenario.route.goal_pose.position.y),
      Kwargs("color"_a = "red", "marker"_a = "*", "s"_a = 120, "label"_a = "goal"));
    ax_xy.set_aspect(Args("equal"));
    ax_xy.set_title(Args(result.termination_reason));
    ax_xy.legend();
    ax_xy.grid();

    ax_v.plot(Args(t, v), Kwargs("color"_a = "tab:red"));
    ax_v.set_xlabel(Args("t [s]"));
    ax_v.set_ylabel(Args("ego velocity [m/s]"));
    ax_v.grid();

    fig.tight_layout();
    save_figure(plt, "closed_loop");
  }
#endif
};

TEST_P(ClosedLoopTest, ReachesGoalWithValidTrajectories)
{
  const auto scenario = load_scenario(GetParam());
  ClosedLoopSimulator simulator(
    scenario.params, scenario.vehicle_info, scenario.map_bin, scenario.route,
    scenario.predicted_objects, ClosedLoopConfig{});
  const auto result = simulator.run();
  write_result_csv(result, make_test_results_dir("closed_loop") + current_test_file_stem());
  SP_PLOT_RESULT({ plot(scenario, simulator, result); });

  if (scenario.expectation == "stop") {
    EXPECT_FALSE(result.goal_reached) << result.termination_reason;
    EXPECT_EQ(result.termination_reason.rfind("stalled", 0), 0u) << result.termination_reason;
    EXPECT_LT(std::abs(result.final_state.odometry.twist.twist.linear.x), 0.1);
  } else {
    EXPECT_TRUE(result.goal_reached) << result.termination_reason;
  }
  EXPECT_TRUE(result.violations.empty()) << [&] {
    std::string s;
    for (const auto & v : result.violations) s += v + "\n";
    return s;
  }();
}

// The list of scenarios to run is test_data/scenarios.yaml (read during static initialization)
std::vector<std::string> load_scenario_list()
{
  const auto path = ament_index_cpp::get_package_share_directory("autoware_safety_planner") +
                    "/test_data/scenarios.yaml";
  return YAML::LoadFile(path)["scenarios"].as<std::vector<std::string>>();
}

INSTANTIATE_TEST_SUITE_P(
  Scenarios, ClosedLoopTest, ::testing::ValuesIn(load_scenario_list()),
  [](const ::testing::TestParamInfo<std::string> & info) {
    // gtest case names allow only alphanumerics and '_'
    auto name = info.param.substr(0, info.param.find_last_of('.'));
    std::replace(name.begin(), name.end(), '.', '_');
    return name;
  });

}  // namespace autoware::safety_planner::testing
