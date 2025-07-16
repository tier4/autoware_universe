// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__CLOTHOID_PULL_OUT_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__CLOTHOID_PULL_OUT_HPP_

#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_planner_base.hpp"
#include "autoware_utils/system/time_keeper.hpp"

#include <autoware/boundary_departure_checker/boundary_departure_checker.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::boundary_departure_checker::BoundaryDepartureChecker;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

// Forward declarations for clothoid-related structures
struct ArcSegment;
struct ClothoidSegment;

/**
 * @brief 指定されたポーズに対してlane_idsを取得する汎用関数
 * @param pose 対象のポーズ
 * @param road_lanes 検索対象のレーン群
 * @param previous_lane_ids 前の点のlane_ids（継承用、オプション）
 * @return 取得されたlane_ids
 */
std::vector<int64_t> getLaneIdsFromPose(
  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & road_lanes,
  const std::vector<int64_t> & previous_lane_ids = {});

/**
 * @brief PathPointWithLaneIdにlane_idsを設定する関数
 * @param point 設定対象のPathPointWithLaneId
 * @param road_lanes 検索対象のレーン群
 * @param previous_lane_ids 前の点のlane_ids（継承用、オプション）
 */
void setLaneIdsToPathPoint(
  PathPointWithLaneId & point, const lanelet::ConstLanelets & road_lanes,
  const std::vector<int64_t> & previous_lane_ids = {});

// Function declarations for clothoid processing functions
std::vector<geometry_msgs::msg::Point> correctClothoidByRigidTransform(
  const std::vector<geometry_msgs::msg::Point> & clothoid_points,
  const ArcSegment & original_segment, const geometry_msgs::msg::Pose & start_pose);

std::pair<std::vector<geometry_msgs::msg::Point>, geometry_msgs::msg::Pose> generateClothoidEntry(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points);

std::pair<std::vector<geometry_msgs::msg::Point>, geometry_msgs::msg::Pose> generateCircularSegment(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points);

std::pair<std::vector<geometry_msgs::msg::Point>, geometry_msgs::msg::Pose> generateClothoidExit(
  const ClothoidSegment & segment, const geometry_msgs::msg::Pose & start_pose, int num_points);

std::vector<geometry_msgs::msg::Point> generateClothoidPath(
  const std::vector<ClothoidSegment> & segments, int num_points_per_segment,
  const geometry_msgs::msg::Pose & start_pose);

std::vector<geometry_msgs::msg::Point> convertArcToClothoid(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose, double A_min,
  double L_min, int num_points_per_segment = 50);

std::vector<geometry_msgs::msg::Point> convertArcToClothoidWithCorrection(
  const ArcSegment & arc_segment, const geometry_msgs::msg::Pose & start_pose, double A_min,
  double L_min, int num_points_per_segment = 50);

/**
 * @brief クロソイドパスからPathWithLaneIdを生成する関数
 * @param clothoid_paths クロソイドパスの配列
 * @param target_pose 目標姿勢
 * @param velocity 初期速度
 * @param target_velocity 目標速度
 * @param acceleration 加速度
 * @param road_lanes 道路レーン情報
 * @param route_handler ルートハンドラー
 * @return PathWithLaneId
 */
PathWithLaneId createPathWithLaneIdFromClothoidPaths(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & clothoid_paths,
  const geometry_msgs::msg::Pose & target_pose, double velocity, double target_velocity,
  double acceleration, const lanelet::ConstLanelets & road_lanes,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

/**
 * @brief センターラインパスとクロソイドパスを結合する関数
 * @param clothoid_path クロソイドパス
 * @param centerline_path センターラインパス
 * @param target_pose 目標姿勢
 * @return 結合されたPathWithLaneId
 */
PathWithLaneId combinePathWithCenterline(
  const PathWithLaneId & clothoid_path, const PathWithLaneId & centerline_path,
  const geometry_msgs::msg::Pose & target_pose);

// Utility function to print PathWithLaneId details
void printPathWithLaneIdDetails(
  const PathWithLaneId & path, const std::string & path_name = "Path");

class ClothoidPullOut : public PullOutPlannerBase
{
public:
  explicit ClothoidPullOut(
    rclcpp::Node & node, const StartPlannerParameters & parameters,
    std::shared_ptr<autoware_utils::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils::TimeKeeper>());

  PlannerType getPlannerType() const override { return PlannerType::CLOTHOID; };
  std::optional<PullOutPath> plan(
    const Pose & start_pose, const Pose & goal_pose,
    const std::shared_ptr<const PlannerData> & planner_data,
    PlannerDebugData & planner_debug_data) override;

  std::shared_ptr<BoundaryDepartureChecker> boundary_departure_checker_;

  friend class TestClothoidPullOut;

  // private:
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__CLOTHOID_PULL_OUT_HPP_
