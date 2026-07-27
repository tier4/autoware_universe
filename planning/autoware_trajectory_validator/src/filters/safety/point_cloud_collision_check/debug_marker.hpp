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
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{

// 1 候補分の debug 中間データ。すべて map 系。
// ObstacleStop の内部状態は覗かず、filter が参照できる情報（前処理済み点群・
// calc_obstacle_stop の戻り値・feasibility 判定結果）だけで構成する。
struct DebugData
{
  // 候補依存
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pointcloud_ptr{};
  std::optional<geometry_msgs::msg::Point> nearest_collision_point{};
  std::optional<double> dist_to_collide{};
  double required_distance{};
  bool is_feasible{true};
  geometry_msgs::msg::Point ego_position{};

  // 追跡中の障害物。速度が確定した候補のみ StopObstacle になるため、未収束の候補は現れない。
  struct Track
  {
    geometry_msgs::msg::Point point{};
    // candidate 軌道に沿った縦速度スカラー（core 1D）。方向は持たないため描画は鉛直バーで代替する。
    double velocity{0.0};
    bool settled{false};
  };
  std::vector<Track> tracks{};
  std::string status_text{};
  // 0=OK(緑) / 1=注意(黄) / 2=DANGER(赤)。バナー色に使う。
  int status_level{0};
  // 検出ポリゴンの色を generator_id(UUID) から決定的に割り当てる（generator ごとに安定色）。
  bool has_generator_color{false};
  std::array<float, 3> generator_color{};
};

// 前処理済み点群と自車位置を収集する。debug が null なら何もしない。
void fill_detection_debug(
  DebugData * debug, const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_pointcloud_ptr,
  const geometry_msgs::msg::Point & ego_position);

// 停止対象（最近傍衝突点とその距離・追跡中の障害物）を収集する。debug が null なら何もしない。
void fill_stop_obstacle_debug(DebugData * debug, const std::vector<StopObstacle> & stop_obstacles);

// feasibility 判定の結果を収集する。debug が null なら何もしない。
void fill_feasibility_debug(DebugData * debug, double required_distance, bool is_feasible);

// 収集済みの DebugData から marker を組み立てて markers へ append する。debug が null
// なら何もしない。 候補ごとの marker と、そのサイクルの先頭候補ならサマリバナーを積む。
// 「先頭候補か」「候補通番」は markers の中身から判定するため、サイクルを跨ぐ状態を持たない。
void emit_debug_markers(
  visualization_msgs::msg::MarkerArray & markers, DebugData * debug,
  const std::array<std::uint8_t, 16> & generator_uuid, const rclcpp::Time & stamp);

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__DEBUG_MARKER_HPP_
