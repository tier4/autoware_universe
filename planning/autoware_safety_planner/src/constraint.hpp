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

#ifndef CONSTRAINT_HPP_
#define CONSTRAINT_HPP_

#include "type_alias.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace autoware::safety_planner
{

inline constexpr double INF = std::numeric_limits<double>::infinity();

//! 制約の**前提がどれだけ確かか**。どの制約セット (normal / cautious) にその制約が
//! 入るかを決める: normal = DEFINITE のみ / cautious = DEFINITE + POSSIBLE。
//! プラグインは自分が出す制約の確度だけを宣言し、何本の軌道が作られるかを知らない
enum class Certainty : std::uint8_t {
  DEFINITE,
  POSSIBLE,
};

//! 制約の出どころの分類。使い道はレポートのグルーピングだけで、パイプラインは区別しない
enum class Category : std::uint8_t {
  SAFETY,   //!< 安全に関する制約 (Safety Constraints Generator)
  TRAFFIC,  //!< 道交法に関する制約 (Traffic Constraints Generator)
};

//! 制約の硬さ。HARD は必ず満たす (スラック無し)。SOFT はスラック付きで破ることを許し、
//! 破り量は Constraint::slack_weight で罰する
enum class Hardness : std::uint8_t {
  HARD,
  SOFT,
};

struct TimeWindow
{
  double t0{0.0};
  double t1{INF};
};

struct ArcRange
{
  double s0{-INF};  //!< [m] ego 投影点基準
  double s1{INF};
};

struct Domain
{
  TimeWindow time{};
  ArcRange arc{};
};

//! 発行元識別子(レポート・診断・マーカー・PlanningFactor 用)。
//! 表示形式は "<plugin_name>/<detail>"
struct Source
{
  std::string plugin_name;              //!< 制約を出したプラグインの識別子(レポート・デバッグ用)
  Category category{Category::SAFETY};  //!< Safety / Traffic の分類(パイプラインは区別しない)
  //! 制約が対象にしている実体の識別子。**周期間で安定な ID** であること
  //! (perception の UUID・lanelet id 等)。s や配列添字に紐づけない。空 = 特定の対象を持たない
  //! (車両運動制約など)。次の 2 つがこの ID を使う:
  //! - 離散決定 (Decisions) のキー。周期をまたいで同じ対象の決定を照合する
  //! - SafetyFactor の object_id。検証層が「どの物体で止まったか」を埋める
  std::string target_id;

  // TODO(odashima): PlanningFactorを参考にする

  std::string detail;  //!< 制約の種別を端的に表す文字列、例: "stop_line",
                       //!< "dynamic_obstacle", "lateral_accel_limit"
};

struct Pose2d
{
  Point2d position{0.0, 0.0};
  double yaw{0.0};
};

//! 時刻付き pose。t は計画基準時刻からの相対秒
struct TimedPose
{
  double t{0.0};
  Pose2d pose{};
};

//! 時刻付き多角形。t は計画基準時刻からの相対秒
struct TimedPolygon
{
  double t{0.0};
  Polygon2d polygon{};
};

// ---------------------------------------------------------------------------------------------
// payload(constraint_desing.md §2.9 の 4 型に 1:1 対応)
// ---------------------------------------------------------------------------------------------

enum class BoundedQuantity : std::uint8_t {
  VELOCITY,     //!< v      [m/s]   縦速度
  LON_ACCEL,    //!< a      [m/s²]  縦加速度(両側 bound が意味を持つ唯一の量)
  LON_JERK,     //!< j      [m/s³]  縦躍度(|j| に対する上限)
  LAT_ACCEL,    //!< v²|κ|  [m/s²]  横加速度
  CURVATURE,    //!< |κ|    [1/m]   曲率
  STEER_ANGLE,  //!< |δ|    [rad]   ステア角
  STEER_RATE,   //!< |δ̇|    [rad/s] ステアレート
};

//! (i) スカラー box。region が無ければ全域(車両運動)、有れば「base_link が region 内にいる間」
//! (§3.5 決定 6: 判定は base_link 包含。保守的にしたい発行側は region を膨張して出す)
struct ScalarBound
{
  BoundedQuantity quantity;
  double min{-INF};  //!< 絶対値量(|j| 等)では使わない(-INF のまま)
  double max{+INF};
  std::optional<Polygon2d> region{};  //!< nullopt = 全域。上限速度は lanelet 形状
};

//! Boundary の禁止側(折れ線の進行向き基準)
enum class Side : std::uint8_t { LEFT, RIGHT };

//! (ii) 越境禁止。走行可能領域の境界折れ線 + 禁止側。周期内静的・無期限
//! 分割・面選択・弧長割当は consumer の仕事(折れ線は生の地図頂点のままでよい)
struct Boundary
{
  LineString2d polyline{};          //!< 折れ線(頂点 2 点以上)
  Side forbidden_side{Side::LEFT};  //!< 越えてはいけない側
  double margin{0.0};               //!< [m] ≥ 0。禁止側をこの分だけ膨張して評価
};

//! 剛体占有。物体ローカル形状が予測 pose 列に沿って動く。waypoint 1 点 = 静的物体。
//! waypoints 間は線形補間(yaw は最短角)、時刻範囲外は無効
struct RigidBody
{
  Polygon2d shape{};                 //!< 物体ローカル(pose 基準)
  std::vector<TimedPose> waypoints;  //!< t 昇順(1 点以上)
};

//! 時変多角形列の占有。時刻とともに変形・成長する領域(飛び出しの到達可能領域等)。
//! polygons 間の補間は保守側(隣接 2 多角形の和で評価してよい)、時刻範囲外は無効
struct TimedPolygonSequence
{
  std::vector<TimedPolygon> polygons;  //!< t 昇順(1 点以上)
};

//! (iii) 占有禁止。footprint(margin 膨張込み)が占有と交わってはいけない
struct KeepOut
{
  std::variant<RigidBody, TimedPolygonSequence> occupancy{};
  double margin_m{0.0};  //!< [m] ≥ 0。定数マージン(消費側が footprint をこの分膨張して評価)
};

//! (iv) 通過禁止ゲート。有効時間帯(Constraint::time)の間、footprint が線分を禁止側へ
//! 越えてはいけない。禁止側 = first → second の向きに対して左側(手前 = 右側が可)。
//! 線分の横は素通りできるので、効かせたい幅の分だけ伸ばして発行する
struct Gate
{
  Segment2d line{};    //!< 有向線分
  double margin{0.0};  //!< [m] ≥ 0。手前に置く追加余裕
};

using ConstraintPayload = std::variant<ScalarBound, Boundary, KeepOut, Gate>;

// ---------------------------------------------------------------------------------------------
// Constraint
// ---------------------------------------------------------------------------------------------

struct Constraint
{
  Certainty certainty{Certainty::DEFINITE};  //!< 前提の確度。normal / cautious の振り分けに使う
  Hardness hardness{Hardness::HARD};         //!< HARD = 必ず満たす / SOFT = スラック付きで破れる
  //! [-] スラック (制約の破り量) に対するペナルティ重み。SOFT のときのみ有効
  double slack_weight{0.0};
  Domain domain{};  //!< 有効時間帯 + 弧長範囲
  ConstraintPayload payload{};
  Source source{};
};

}  // namespace autoware::safety_planner

#endif  // CONSTRAINT_HPP_
