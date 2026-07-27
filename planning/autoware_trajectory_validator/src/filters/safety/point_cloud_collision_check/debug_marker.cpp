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

#include "debug_marker.hpp"

#include <rclcpp/duration.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
namespace
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

std::int32_t next_marker_id(const MarkerArray & markers)
{
  return markers.markers.empty() ? 0 : markers.markers.back().id + 1;
}

std_msgs::msg::ColorRGBA make_color(const double r, const double g, const double b, const double a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

// 候補ごとの色（per-cycle カウンタ k のカラーテーブル・決定10/可視化）。
std_msgs::msg::ColorRGBA candidate_color(const int k, const double a)
{
  static const std::vector<std::array<double, 3>> table = {{0.1, 0.6, 1.0}, {1.0, 0.6, 0.1},
                                                           {0.4, 1.0, 0.4}, {1.0, 0.4, 0.8},
                                                           {0.8, 0.8, 0.2}, {0.6, 0.4, 1.0}};
  const auto & rgb = table[static_cast<size_t>(k) % table.size()];
  return make_color(rgb[0], rgb[1], rgb[2], a);
}

Marker base_marker(
  const std::string & ns, const std::int32_t id, const std::int32_t type,
  const rclcpp::Time & stamp)
{
  Marker m;
  m.header.frame_id = "map";
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 = 無限（RViz は消さない）
  return m;
}

// 候補 marker は候補ごとに必ず 1 つ積まれる（add_candidate_debug_markers の項目4）。
// その数がそのまま、このサイクルで既に評価した候補の数＝今回の候補通番になる。
constexpr const char * candidate_marker_ns_suffix = "/feasibility";

int count_candidate_markers(const MarkerArray & markers)
{
  const std::string suffix{candidate_marker_ns_suffix};
  return static_cast<int>(
    std::count_if(markers.markers.begin(), markers.markers.end(), [&suffix](const Marker & marker) {
      return marker.ns.size() >= suffix.size() &&
             marker.ns.compare(marker.ns.size() - suffix.size(), suffix.size(), suffix) == 0;
    }));
}

geometry_msgs::msg::Point make_point(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// generator_id(UUID の 16 byte) を FNV-1a でハッシュし、パレットから決定的に色を割り当てる。
// generator ごとに安定した色になる（generator_name には依存しない）。
std::array<float, 3> generator_color_from_uuid(const std::array<std::uint8_t, 16> & uuid)
{
  std::uint32_t h = 2166136261u;
  for (const auto byte : uuid) {
    h = (h ^ byte) * 16777619u;
  }
  static const std::array<std::array<float, 3>, 6> palette = {
    {{0.1f, 0.6f, 1.0f},
     {1.0f, 0.5f, 0.1f},
     {0.4f, 1.0f, 0.4f},
     {1.0f, 0.4f, 0.8f},
     {0.9f, 0.9f, 0.2f},
     {0.6f, 0.4f, 1.0f}}};
  return palette.at(h % palette.size());
}

// 候補依存 marker（検出ポリゴン・絞り込み点群・衝突点・dist/feasibility テキスト）を append する。
void add_candidate_debug_markers(
  MarkerArray & markers, const DebugData & debug_data, const int k, const rclcpp::Time & stamp)
{
  const std::string ns_prefix = std::to_string(k);

  // 2. 前処理後の絞り込み点群（回廊内クラスタのみ）
  if (debug_data.filtered_pointcloud_ptr && !debug_data.filtered_pointcloud_ptr->empty()) {
    auto m = base_marker(ns_prefix + "/clusters", next_marker_id(markers), Marker::POINTS, stamp);
    m.scale.x = 0.2;
    m.scale.y = 0.2;
    // generator_id(UUID) 由来の色で描く（無ければ候補色）。
    m.color = debug_data.has_generator_color
                ? make_color(
                    debug_data.generator_color[0], debug_data.generator_color[1],
                    debug_data.generator_color[2], 0.9)
                : candidate_color(k, 0.9);
    for (const auto & p : debug_data.filtered_pointcloud_ptr->points) {
      m.points.push_back(make_point(p.x, p.y, p.z));
    }
    markers.markers.push_back(m);
  }

  // 3. 最近傍衝突点
  if (debug_data.nearest_collision_point) {
    auto m =
      base_marker(ns_prefix + "/collision_point", next_marker_id(markers), Marker::SPHERE, stamp);
    m.pose.position = *debug_data.nearest_collision_point;
    m.scale.x = m.scale.y = m.scale.z = 0.6;
    m.color = make_color(1.0, 0.1, 0.1, 0.9);
    markers.markers.push_back(m);
  }

  // 4. dist と feasibility 判定結果（安全なときも「SAFE」を必ず描く）
  {
    auto m = base_marker(
      ns_prefix + candidate_marker_ns_suffix, next_marker_id(markers), Marker::TEXT_VIEW_FACING,
      stamp);
    // 障害物が在れば衝突点上、無ければ ego 上に候補ごとに段積みで表示する。
    if (debug_data.nearest_collision_point) {
      m.pose.position = *debug_data.nearest_collision_point;
      m.pose.position.z += 1.0;
    } else {
      m.pose.position = debug_data.ego_position;
      m.pose.position.z += 1.5 + 0.7 * static_cast<double>(k);
    }
    m.scale.z = 0.6;
    m.color =
      debug_data.is_feasible ? make_color(0.2, 1.0, 0.3, 1.0) : make_color(1.0, 0.1, 0.1, 1.0);
    char buf[160];
    if (debug_data.dist_to_collide) {
      std::snprintf(
        buf, sizeof(buf), "cand%d: %s  dist=%.2f req=%.2f", k,
        debug_data.is_feasible ? "SAFE" : "STOP REQUIRED", *debug_data.dist_to_collide,
        debug_data.required_distance);
    } else if (debug_data.nearest_collision_point) {
      // 衝突点は検出したが時系列追跡が未収束（確認中）。
      std::snprintf(buf, sizeof(buf), "cand%d: SAFE (obstacle detected, confirming...)", k);
    } else {
      std::snprintf(buf, sizeof(buf), "cand%d: SAFE (clear)", k);
    }
    m.text = buf;
    markers.markers.push_back(m);
  }
}

// サイクル依存 marker（時系列 deque の追跡点・推定速度・status バナー）を append する。
void add_cycle_debug_markers(
  MarkerArray & markers, const DebugData & debug_data, const rclcpp::Time & stamp)
{
  // 5. 時系列追跡 deque の world point
  if (!debug_data.tracks.empty()) {
    auto m = base_marker("tracking/points", next_marker_id(markers), Marker::SPHERE_LIST, stamp);
    m.scale.x = m.scale.y = m.scale.z = 0.4;
    m.color = make_color(1.0, 1.0, 1.0, 0.8);
    for (const auto & track : debug_data.tracks) {
      m.points.push_back(track.point);
    }
    markers.markers.push_back(m);

    // 6. 推定縦速度（scalar）を鉛直バー長で表現（未収束は淡色）
    for (const auto & track : debug_data.tracks) {
      auto arrow = base_marker("tracking/velocity", next_marker_id(markers), Marker::ARROW, stamp);
      arrow.scale.x = 0.1;
      arrow.scale.y = 0.2;
      arrow.scale.z = 0.2;
      arrow.color = track.settled ? make_color(1.0, 1.0, 0.2, 0.9) : make_color(0.6, 0.6, 0.6, 0.5);
      arrow.points.push_back(track.point);
      arrow.points.push_back(
        make_point(track.point.x, track.point.y, track.point.z + track.velocity));
      markers.markers.push_back(arrow);
    }
  }

  // 8. ステータスバナー（常時描画：OK=緑 / 注意=黄 / DANGER=赤）
  {
    auto m = base_marker("status", next_marker_id(markers), Marker::TEXT_VIEW_FACING, stamp);
    m.pose.position = debug_data.ego_position;
    m.pose.position.z += 3.0;
    m.scale.z = 0.9;
    switch (debug_data.status_level) {
      case 2:
        m.color = make_color(1.0, 0.1, 0.1, 1.0);
        break;
      case 1:
        m.color = make_color(1.0, 0.9, 0.2, 1.0);
        break;
      default:
        m.color = make_color(0.2, 1.0, 0.3, 1.0);
        break;
    }
    m.text =
      debug_data.status_text.empty() ? std::string{"PCC: monitoring"} : debug_data.status_text;
    markers.markers.push_back(m);
  }
}

}  // namespace

void fill_detection_debug(
  DebugData * debug, const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_pointcloud_ptr,
  const geometry_msgs::msg::Point & ego_position)
{
  if (!debug) {
    return;
  }
  debug->filtered_pointcloud_ptr = filtered_pointcloud_ptr;
  debug->ego_position = ego_position;
}

void fill_stop_obstacle_debug(DebugData * debug, const std::vector<StopObstacle> & stop_obstacles)
{
  if (!debug) {
    return;
  }
  for (const auto & stop_obstacle : stop_obstacles) {
    DebugData::Track track;
    track.point = stop_obstacle.collision_point;
    track.velocity = stop_obstacle.velocity;
    track.settled = true;
    debug->tracks.push_back(track);
  }
  const auto nearest = std::min_element(
    stop_obstacles.begin(), stop_obstacles.end(),
    [](const StopObstacle & a, const StopObstacle & b) {
      return a.dist_to_collide_on_decimated_traj < b.dist_to_collide_on_decimated_traj;
    });
  if (nearest != stop_obstacles.end()) {
    debug->nearest_collision_point = nearest->collision_point;
    debug->dist_to_collide = nearest->dist_to_collide_on_decimated_traj;
  }
}

void fill_feasibility_debug(
  DebugData * debug, const double required_distance, const bool is_feasible)
{
  if (!debug) {
    return;
  }
  debug->required_distance = required_distance;
  debug->is_feasible = is_feasible;
}

void emit_debug_markers(
  MarkerArray & markers, DebugData * debug, const std::array<std::uint8_t, 16> & generator_uuid,
  const rclcpp::Time & stamp)
{
  if (!debug) {
    return;
  }
  // take_debug_markers() が毎サイクル clear するので、入場時に空なら先頭候補。
  const bool first_candidate = markers.markers.empty();
  const int candidate_index = count_candidate_markers(markers);

  debug->generator_color = generator_color_from_uuid(generator_uuid);
  debug->has_generator_color = true;
  add_candidate_debug_markers(markers, *debug, candidate_index, stamp);

  if (!first_candidate) {
    return;
  }
  // 常時バナー：安全なら緑 SAFE、危険なら赤 STOP。点群 OK と追跡数も表示。
  debug->status_level = debug->is_feasible ? 0 : 2;
  debug->status_text =
    std::string{"PCC: "} + (debug->is_feasible ? "SAFE" : "STOP REQUIRED") +
    " | pointcloud:OK | tracked obstacles:" + std::to_string(debug->tracks.size());
  add_cycle_debug_markers(markers, *debug, stamp);
}

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
