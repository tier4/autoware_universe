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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__DEBUG_MARKER_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__DEBUG_MARKER_HPP_

#include "types.hpp"

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{

// 1 候補分の debug 中間データ。すべて map 系。
struct DebugData
{
  // 候補依存
  std::vector<Polygon2d> detection_polygons{};
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pointcloud_ptr{};
  std::optional<geometry_msgs::msg::Point> nearest_collision_point{};
  std::optional<double> dist_to_collide{};
  double required_distance{};
  bool is_feasible{true};
  geometry_msgs::msg::Point ego_position{};

  // サイクル依存（時系列 deque スナップショット）
  struct Track
  {
    geometry_msgs::msg::Point point{};
    Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
    bool settled{false};
  };
  std::vector<Track> tracks{};
  std::string status_text{};
  // 0=OK(緑) / 1=注意(黄) / 2=DANGER(赤)。バナー色に使う。
  int status_level{0};
  bool has_stop_obstacle{false};
  // 検出ポリゴンの色を generator で分ける: -1=不明(候補色) / 0=Diffusion(青) / 1=minimum_rule_based(橙)。
  int generator_kind{-1};
};

// 候補依存 marker（検出ポリゴン・絞り込み点群・衝突点・dist/feasibility テキスト）を候補ごとに append する。
void add_candidate_debug_markers(
  visualization_msgs::msg::MarkerArray & markers, const DebugData & debug_data, int k,
  const rclcpp::Time & stamp);

// サイクル依存 marker（時系列 deque の world point・速度ベクトル・status テキスト）を先頭候補で 1 回だけ append する。
void add_cycle_debug_markers(
  visualization_msgs::msg::MarkerArray & markers, const DebugData & debug_data,
  const rclcpp::Time & stamp);

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__DEBUG_MARKER_HPP_
