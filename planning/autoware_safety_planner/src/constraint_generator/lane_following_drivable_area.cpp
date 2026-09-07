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

#include "lane_following_drivable_area.hpp"

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

//! [m] bound 上のサンプル間隔
constexpr double SAMPLE_STEP_M = 5.0;
//! [m] bound の外側にこの分出た点で並走車線を探す (車線幅の半分より小さいこと)
constexpr double ADJACENT_OFFSET_M = 1.5;
//! [rad] 並走とみなす方位差 (対向は反転して評価)。交差点で直交する道路を除外する
constexpr double PARALLEL_YAW_THRESHOLD = M_PI / 4.0;
//! サンプルのこの割合以上で並走車線ヒットなら「車線あり」とみなす (継ぎ目の取りこぼし吸収)
constexpr double ADJACENT_FRACTION = 0.5;
//! [m] road_border を探す半径
constexpr double BORDER_SEARCH_RADIUS_M = 15.0;

//! 折れ線を弧長 step ごとにサンプルした (点, 進行方位) 列
std::vector<std::pair<lanelet::BasicPoint2d, double>> sample_polyline_with_yaw(
  const lanelet::ConstLineString3d & line, const double step)
{
  std::vector<std::pair<lanelet::BasicPoint2d, double>> samples;
  for (std::size_t i = 0; i + 1 < line.size(); ++i) {
    const lanelet::BasicPoint2d p0{line[i].x(), line[i].y()};
    const lanelet::BasicPoint2d p1{line[i + 1].x(), line[i + 1].y()};
    const double yaw = std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
    const double length = (p1 - p0).norm();
    for (double s = 0.0; s < length; s += step) {
      samples.emplace_back(p0 + (p1 - p0) * (s / length), yaw);
    }
  }
  return samples;
}

double point_segment_distance(
  const lanelet::BasicPoint2d & p, const lanelet::BasicPoint2d & a, const lanelet::BasicPoint2d & b)
{
  const auto ab = b - a;
  const double len2 = ab.squaredNorm();
  const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
  return (p - (a + ab * t)).norm();
}

double point_polyline_distance(
  const lanelet::BasicPoint2d & p, const lanelet::ConstLineString3d & line)
{
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1 < line.size(); ++i) {
    best = std::min(
      best, point_segment_distance(
              p, lanelet::BasicPoint2d{line[i].x(), line[i].y()},
              lanelet::BasicPoint2d{line[i + 1].x(), line[i + 1].y()}));
  }
  return best;
}

//! line のうち point に最も近いセグメントの方位
double nearest_segment_yaw(
  const lanelet::ConstLineString3d & line, const lanelet::BasicPoint2d & point)
{
  double best_dist = std::numeric_limits<double>::max();
  double best_yaw = 0.0;
  for (std::size_t i = 0; i + 1 < line.size(); ++i) {
    const lanelet::BasicPoint2d p0{line[i].x(), line[i].y()};
    const lanelet::BasicPoint2d p1{line[i + 1].x(), line[i + 1].y()};
    const double dist = point_segment_distance(point, p0, p1);
    if (dist < best_dist) {
      best_dist = dist;
      best_yaw = std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
    }
  }
  return best_yaw;
}

//! p_out に並走車線 (対向含む) が存在するか
bool has_parallel_road_lanelet_at(
  const lanelet::LaneletMapConstPtr & lanelet_map, const lanelet::BasicPoint2d & p_out,
  const double yaw, const lanelet::Id self_id)
{
  for (const auto & candidate : autoware::experimental::lanelet2_utils::get_road_lanelets_at(
         lanelet_map, p_out.x(), p_out.y())) {
    if (candidate.id() == self_id) {
      continue;
    }
    const double dyaw = std::abs(
      std::remainder(nearest_segment_yaw(candidate.centerline(), p_out) - yaw, 2.0 * M_PI));
    if (dyaw < PARALLEL_YAW_THRESHOLD || dyaw > M_PI - PARALLEL_YAW_THRESHOLD) {
      return true;
    }
  }
  return false;
}

//! Boundary 制約を 1 本組み立てる。polyline は進行方向順であること (forbidden_side の前提)
Constraint make_boundary_constraint(
  const std::vector<Point2d> & polyline, const Side side, const double margin_m,
  const Hardness hardness, const double slack_weight, const std::string & plugin_name,
  const std::string & target_id, const std::string & detail)
{
  Constraint constraint;
  constraint.certainty = Certainty::DEFINITE;  // 地図は前提が確定している
  constraint.hardness = hardness;
  constraint.slack_weight = hardness == Hardness::SOFT ? slack_weight : 0.0;
  Boundary boundary;
  boundary.polyline.assign(polyline.begin(), polyline.end());
  boundary.forbidden_side = side;
  boundary.margin = margin_m;
  constraint.payload = std::move(boundary);
  constraint.source = Source{plugin_name, Category::SAFETY, target_id, detail};
  return constraint;
}

}  // namespace

ConstraintGeneratorOutput LaneFollowingDrivableAreaConstraintGenerator::generate_constraints(
  const PlannerContext & context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  ConstraintGeneratorOutput output;

  // route 未確立の周期は制約を出せない (S7 §2: 空を返してパイプラインを継続させる)
  if (!context.route_manager) {
    return output;
  }

  // reference_path と同じ窓のレーン列を対象にする (S2 の reference_path パラメータを共有)
  const auto & route_manager = *context.route_manager;
  const auto lanelets =
    route_manager
      .get_lanelet_sequence_on_route(
        params_.reference_path.forward_length_m, params_.reference_path.backward_length_m)
      .as_lanelets();
  const auto & lanelet_map = route_manager.lanelet_map_ptr();
  const double margin_m = params_.lane_following_drivable_area.margin_m;
  const double bound_slack_weight = params_.lane_following_drivable_area.bound_slack_weight;

  //! 同じ road_border が複数 lanelet から採用されたときの重複発行を防ぐ
  std::set<std::pair<bool, lanelet::Id>> adopted_borders;

  for (const auto & lanelet : lanelets) {
    for (const auto side_left : {true, false}) {
      const auto & bound = side_left ? lanelet.leftBound() : lanelet.rightBound();
      // 地図 (外部リソース) の境界。頂点 2 点未満の不正形状は折れ線にならないので出さない
      if (bound.size() < 2) {
        continue;
      }
      const Side side = side_left ? Side::LEFT : Side::RIGHT;
      const double side_sign = side_left ? 1.0 : -1.0;
      const auto samples = sample_polyline_with_yaw(bound, SAMPLE_STEP_M);
      if (samples.empty()) {
        continue;
      }

      // 並走車線の有無の判定と、無い区間の最寄り road_border の収集を 1 パスで行う
      std::size_t adjacent_hits = 0;
      std::set<lanelet::Id> border_ids;
      for (const auto & [point, yaw] : samples) {
        // 左 bound の外側 = 進行方向の左 (+90°)、右 bound の外側 = 右 (-90°)
        const lanelet::BasicPoint2d p_out{
          point.x() - std::sin(yaw) * ADJACENT_OFFSET_M * side_sign,
          point.y() + std::cos(yaw) * ADJACENT_OFFSET_M * side_sign};

        if (has_parallel_road_lanelet_at(lanelet_map, p_out, yaw, lanelet.id())) {
          ++adjacent_hits;
          continue;
        }

        const lanelet::BoundingBox2d search_box{
          lanelet::BasicPoint2d{
            p_out.x() - BORDER_SEARCH_RADIUS_M, p_out.y() - BORDER_SEARCH_RADIUS_M},
          lanelet::BasicPoint2d{
            p_out.x() + BORDER_SEARCH_RADIUS_M, p_out.y() + BORDER_SEARCH_RADIUS_M}};
        double best_dist = std::numeric_limits<double>::max();
        std::optional<lanelet::Id> best_id;
        for (const auto & linestring : lanelet_map->lineStringLayer.search(search_box)) {
          const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, "");
          if (type != "road_border" || linestring.size() < 2) {
            continue;
          }
          const double dist = point_polyline_distance(p_out, linestring);
          if (dist < best_dist) {
            best_dist = dist;
            best_id = linestring.id();
          }
        }
        if (best_id) {
          border_ids.insert(*best_id);
        }
      }

      const bool adjacent =
        static_cast<double>(adjacent_hits) / static_cast<double>(samples.size()) >=
        ADJACENT_FRACTION;

      if (adjacent) {
        // 並走車線あり: 自レーンの bound を soft にする (この側に road_border は出さない)
        std::vector<Point2d> polyline;
        polyline.reserve(bound.size());
        for (const auto & point : bound) {
          polyline.emplace_back(point.x(), point.y());
        }
        output.constraints.push_back(make_boundary_constraint(
          polyline, side, margin_m, Hardness::SOFT, bound_slack_weight, get_name(),
          std::to_string(lanelet.id()), side_left ? "left_bound" : "right_bound"));
        continue;
      }

      // 並走車線なし: road_border を hard にする (路肩等の上は走行可能領域に含まれる)
      for (const auto border_id : border_ids) {
        if (!adopted_borders.insert({side_left, border_id}).second) {
          continue;  // 別の lanelet が同じ border を既に発行している
        }
        const auto border = lanelet_map->lineStringLayer.get(border_id);
        // forbidden_side は折れ線の進行向き基準。地図の road_border は向きが不定なので、
        // 逆向き (進行方向と反平行) なら反転してから発行する
        const auto & [mid_point, mid_yaw] = samples[samples.size() / 2];
        const double dyaw =
          std::abs(std::remainder(nearest_segment_yaw(border, mid_point) - mid_yaw, 2.0 * M_PI));
        const bool reversed = dyaw > M_PI / 2.0;

        std::vector<Point2d> polyline;
        polyline.reserve(border.size());
        for (const auto & point : border) {
          polyline.emplace_back(point.x(), point.y());
        }
        if (reversed) {
          std::reverse(polyline.begin(), polyline.end());
        }
        output.constraints.push_back(make_boundary_constraint(
          polyline, side, margin_m, Hardness::HARD, 0.0, get_name(), std::to_string(border_id),
          side_left ? "left_road_border" : "right_road_border"));
      }
    }
  }

  // --- debug marker: 採用した境界の折れ線 ---
  {
    using autoware_utils_visualization::create_default_marker;
    using autoware_utils_visualization::create_marker_color;
    using autoware_utils_visualization::create_marker_scale;

    auto bound_marker = create_default_marker(
      "map", rclcpp::Time(0, 0, RCL_ROS_TIME), "lane_following_drivable_area_bounds", 0,
      Marker::LINE_LIST, create_marker_scale(0.15, 0.0, 0.0),
      create_marker_color(1.0, 0.6, 0.0, 0.9));
    for (const auto & constraint : output.constraints) {
      const auto & boundary = std::get<Boundary>(constraint.payload);
      for (std::size_t i = 0; i + 1 < boundary.polyline.size(); ++i) {
        for (const auto & p : {boundary.polyline[i], boundary.polyline[i + 1]}) {
          geometry_msgs::msg::Point q;
          q.x = p.x();
          q.y = p.y();
          q.z = context.odometry.pose.pose.position.z;
          bound_marker.points.push_back(q);
        }
      }
    }
    if (!bound_marker.points.empty()) {
      MarkerArray marker_array;
      marker_array.markers.push_back(std::move(bound_marker));
      output.debug_markers = std::move(marker_array);
    }
  }

  return output;
}

}  // namespace autoware::safety_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::LaneFollowingDrivableAreaConstraintGenerator,
  autoware::safety_planner::ConstraintGeneratorInterface)
