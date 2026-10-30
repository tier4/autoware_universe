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

#include "autoware/diffusion_planner/preprocessing/bev_image.hpp"

#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{
namespace
{
// Line thicknesses and marker sizes, all from render_bev.py.
constexpr int LANE_BOUNDARY_THICKNESS = 1;
constexpr int LANE_CENTERLINE_THICKNESS = 1;
constexpr int TRAFFIC_LIGHT_THICKNESS = 2;
constexpr int ROUTE_THICKNESS = 3;
constexpr int LINE_STRING_THICKNESS = 1;
constexpr int HISTORY_THICKNESS = 2;
constexpr int GOAL_MARKER_THICKNESS = 2;
constexpr double GOAL_MARKER_LENGTH_M = 5.0;
constexpr int GOAL_MARKER_RADIUS_PX = 3;

// Binary masks: hard edges, so cv::LINE_8 rather than an anti-aliased line type that would emit
// intermediate values.
//
// Rendered against render_bev.py on real dataset frames, this file reproduces it exactly for
// every polyline channel. The filled channels (polygon, agent boxes) can differ by a handful of
// pixels on a shape that runs off the canvas, because OpenCV changed how fillPoly clips between
// the version the training environment uses and the one Autoware builds against.
constexpr int LINE_TYPE = cv::LINE_8;
constexpr int FOREGROUND = 255;

constexpr double VALID_EPS = 1e-6;
constexpr double MIN_BOX_SIZE_M = 0.1;

// Neighbor state layout inside neighbor_agents_past (see AgentState::as_array).
constexpr int64_t NEIGHBOR_STATE_DIM = static_cast<int64_t>(AGENT_STATE_DIM);
constexpr int64_t NEIGHBOR_IDX_COS = 2;
constexpr int64_t NEIGHBOR_IDX_SIN = 3;
constexpr int64_t NEIGHBOR_IDX_WIDTH = 6;
constexpr int64_t NEIGHBOR_IDX_LENGTH = 7;
constexpr int64_t NEIGHBOR_IDX_TYPE_START = 8;
constexpr int64_t NEIGHBOR_TYPE_NUM = 3;

// Static object state layout inside static_objects.
constexpr int64_t STATIC_IDX_COS = 2;
constexpr int64_t STATIC_IDX_SIN = 3;
constexpr int64_t STATIC_IDX_WIDTH = 4;
constexpr int64_t STATIC_IDX_LENGTH = 5;

// Guard against undefined behavior when casting a pixel coordinate to int. Anything this far
// off-canvas is clipped away by the drawing calls regardless of its exact value.
constexpr double MAX_PIXEL_COORDINATE = 1.0e7;

// Later entries win where channels overlap, so agents stay visible on top of the map. Lanes
// cross at an intersection, so the light planes are ordered by how much they demand
// attention: an unknown light loses to a green one, which loses to yellow, which loses to red.
constexpr std::array<BevChannel, BEV_NUM_CHANNELS> DRAW_ORDER = {
  BEV_CH_POLYGON,
  BEV_CH_LANE_CENTERLINE,
  BEV_CH_LANE_BOUNDARY,
  BEV_CH_LINE_STRING,
  BEV_CH_ROUTE,
  BEV_CH_TRAFFIC_LIGHT_UNKNOWN,
  BEV_CH_TRAFFIC_LIGHT_GO,
  BEV_CH_TRAFFIC_LIGHT_CAUTION,
  BEV_CH_TRAFFIC_LIGHT_STOP,
  BEV_CH_GOAL_POSE,
  BEV_CH_NEIGHBOR_HISTORY,
  BEV_CH_EGO_HISTORY,
  BEV_CH_STATIC_OBJECT,
  BEV_CH_VEHICLE,
  BEV_CH_PEDESTRIAN,
  BEV_CH_BICYCLE,
  BEV_CH_EGO};
constexpr std::array<std::array<uint8_t, 3>, BEV_NUM_CHANNELS> COLOR_BGR_OF_CHANNEL = {{
  {160, 160, 160},  // BEV_CH_LANE_BOUNDARY
  {90, 90, 90},     // BEV_CH_LANE_CENTERLINE
  {255, 0, 255},    // BEV_CH_ROUTE
  {0, 200, 0},      // BEV_CH_TRAFFIC_LIGHT_GO
  {0, 255, 255},    // BEV_CH_TRAFFIC_LIGHT_CAUTION
  {0, 0, 255},      // BEV_CH_TRAFFIC_LIGHT_STOP
  {128, 0, 128},    // BEV_CH_TRAFFIC_LIGHT_UNKNOWN
  {60, 110, 60},    // BEV_CH_POLYGON
  {0, 140, 255},    // BEV_CH_LINE_STRING
  {0, 200, 200},    // BEV_CH_STATIC_OBJECT
  {255, 128, 0},    // BEV_CH_GOAL_POSE
  {255, 60, 60},    // BEV_CH_VEHICLE
  {60, 255, 60},    // BEV_CH_PEDESTRIAN
  {255, 0, 128},    // BEV_CH_BICYCLE
  {255, 255, 255},  // BEV_CH_EGO
  {128, 64, 0},     // BEV_CH_NEIGHBOR_HISTORY
  {0, 165, 255},    // BEV_CH_EGO_HISTORY
}};
// A channel missing from the draw order would silently vanish from every preview, which is
// exactly how a folded-together traffic light state went unnoticed before.
static_assert(
  [] {
    std::array<bool, BEV_NUM_CHANNELS> seen{};
    for (const BevChannel channel : DRAW_ORDER) {
      seen[static_cast<size_t>(channel)] = true;
    }
    for (const bool value : seen) {
      if (!value) {
        return false;
      }
    }
    return true;
  }(),
  "every BevChannel must appear in the preview draw order");

using Canvas = std::array<cv::Mat, BEV_NUM_CHANNELS>;

/// Pixel mapping of one scale: ego at the image center, ego heading pointing up.
struct BevView
{
  double pixels_per_meter;
  double half;
};

BevView make_view(const double extent_m)
{
  return BevView{
    static_cast<double>(BEV_IMAGE_SIZE) / extent_m, static_cast<double>(BEV_IMAGE_SIZE) / 2.0};
}

/// Map ego-relative meters to integer (col, row) pixels.
cv::Point to_pixel(const BevView & view, const Eigen::Vector2d & point_rel)
{
  const double col = view.half - point_rel.y() * view.pixels_per_meter;
  const double row = view.half - point_rel.x() * view.pixels_per_meter;
  // std::nearbyint rounds half to even under the default rounding mode, matching numpy's round.
  const auto to_int = [](const double value) {
    return static_cast<int>(
      std::nearbyint(std::clamp(value, -MAX_PIXEL_COORDINATE, MAX_PIXEL_COORDINATE)));
  };
  return cv::Point{to_int(col), to_int(row)};
}

double yaw_from_cos_sin(const double cos_value, const double sin_value)
{
  return std::atan2(sin_value, cos_value);
}

Eigen::Matrix2d rotation_matrix(const double yaw)
{
  Eigen::Matrix2d rotation;
  rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  return rotation;
}

/// Maps points into the ego-centered, ego-aligned frame. The tensors handed to the node are
/// already ego-relative, which makes this the identity; it is kept so the renderer stays a
/// faithful port and also works on frames whose ego pose is not the origin.
class BevTransform
{
public:
  BevTransform(const double ego_x, const double ego_y, const double ego_yaw)
  : rotation_(rotation_matrix(-ego_yaw)), origin_(ego_x, ego_y)
  {
  }

  Eigen::Vector2d operator()(const double x, const double y) const
  {
    return rotation_ * (Eigen::Vector2d(x, y) - origin_);
  }

private:
  Eigen::Matrix2d rotation_;
  Eigen::Vector2d origin_;
};

/// Row-major view over the first batch entry of one input tensor.
class TensorView
{
public:
  TensorView(const InputDataMap & input_data_map, const std::string & key, const int64_t row_stride)
  : data_(input_data_map.at(key).data()), row_stride_(row_stride)
  {
    assert(input_data_map.at(key).size() % static_cast<size_t>(row_stride) == 0);
  }

  const float * row(const int64_t index) const { return data_ + index * row_stride_; }

private:
  const float * data_;
  int64_t row_stride_;
};

/// Number of leading points of a polyline before its zero padding.
int64_t valid_prefix_length(const float * points, const int64_t num_points, const int64_t point_dim)
{
  for (int64_t i = 0; i < num_points; ++i) {
    const float * point = points + i * point_dim;
    if (std::abs(point[X]) + std::abs(point[Y]) < VALID_EPS) {
      return i;
    }
  }
  return num_points;
}

// The single-contour overloads are taken explicitly: the InputArrayOfArrays forms would have to
// guess whether a vector<cv::Point> is one contour or a list of them.
void draw_pixel_polyline(
  cv::Mat & canvas, const std::vector<cv::Point> & pixels, const int thickness)
{
  const cv::Point * contour = pixels.data();
  const int num_points = static_cast<int>(pixels.size());
  cv::polylines(
    canvas, &contour, &num_points, 1, false, cv::Scalar(FOREGROUND), thickness, LINE_TYPE);
}

void fill_pixel_polygon(cv::Mat & canvas, const std::vector<cv::Point> & pixels)
{
  const cv::Point * contour = pixels.data();
  const int num_points = static_cast<int>(pixels.size());
  cv::fillPoly(canvas, &contour, &num_points, 1, cv::Scalar(FOREGROUND), LINE_TYPE);
}

void draw_polyline(
  cv::Mat & canvas, const BevView & view, const std::vector<Eigen::Vector2d> & points_rel,
  const int thickness)
{
  if (points_rel.size() < 2) {
    return;
  }
  std::vector<cv::Point> pixels;
  pixels.reserve(points_rel.size());
  for (const auto & point_rel : points_rel) {
    pixels.push_back(to_pixel(view, point_rel));
  }
  draw_pixel_polyline(canvas, pixels, thickness);
}

/// Fill an oriented bounding box; `length` runs along `yaw_rel`.
void draw_box(
  cv::Mat & canvas, const BevView & view, const Eigen::Vector2d & center_rel, const double length,
  const double width, const double yaw_rel)
{
  const double half_length = std::max(length, MIN_BOX_SIZE_M) / 2.0;
  const double half_width = std::max(width, MIN_BOX_SIZE_M) / 2.0;
  const std::array<Eigen::Vector2d, 4> local = {
    Eigen::Vector2d{half_length, half_width}, Eigen::Vector2d{half_length, -half_width},
    Eigen::Vector2d{-half_length, -half_width}, Eigen::Vector2d{-half_length, half_width}};

  const Eigen::Matrix2d rotation = rotation_matrix(yaw_rel);
  std::vector<cv::Point> corners;
  corners.reserve(local.size());
  for (const auto & corner : local) {
    corners.push_back(to_pixel(view, rotation * corner + center_rel));
  }
  fill_pixel_polygon(canvas, corners);
}

/// Mark a pose as a dot with a short segment pointing along the yaw.
void draw_pose_marker(
  cv::Mat & canvas, const BevView & view, const Eigen::Vector2d & center_rel, const double yaw_rel)
{
  const Eigen::Vector2d tip =
    center_rel + Eigen::Vector2d(std::cos(yaw_rel), std::sin(yaw_rel)) * GOAL_MARKER_LENGTH_M;
  draw_polyline(canvas, view, {center_rel, tip}, GOAL_MARKER_THICKNESS);
  cv::circle(
    canvas, to_pixel(view, center_rel), GOAL_MARKER_RADIUS_PX, cv::Scalar(FOREGROUND), cv::FILLED,
    LINE_TYPE);
}

// Plane each traffic light state is drawn onto; mirrors
// render_bev.py::TRAFFIC_LIGHT_CHANNEL_OF_STATE. Every state gets its own plane instead of being
// folded into one "stop" plane, so the encoder can tell a yellow light from a red one, and a
// light whose state never arrived from a lane that has no light at all. TRAFFIC_LIGHT_WHITE is
// the slot the observation writes when the lane has a light but its colour is unknown or was
// never received.
//
// TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT deliberately has no plane: such a lane is already drawn on the
// centerline channel, and its silence across all four light planes is what identifies it.
constexpr std::array<std::pair<int64_t, BevChannel>, 4> TRAFFIC_LIGHT_CHANNEL_OF_STATE = {{
  {TRAFFIC_LIGHT_GREEN, BEV_CH_TRAFFIC_LIGHT_GO},
  {TRAFFIC_LIGHT_YELLOW, BEV_CH_TRAFFIC_LIGHT_CAUTION},
  {TRAFFIC_LIGHT_RED, BEV_CH_TRAFFIC_LIGHT_STOP},
  {TRAFFIC_LIGHT_WHITE, BEV_CH_TRAFFIC_LIGHT_UNKNOWN},
}};

/// Plane the lane's traffic light belongs on; empty when the lane has no light at all.
std::optional<BevChannel> traffic_light_channel(const float * state)
{
  for (const auto & [index, channel] : TRAFFIC_LIGHT_CHANNEL_OF_STATE) {
    if (state[index] == 1.0f) {
      return channel;
    }
  }
  return std::nullopt;
}

BevChannel neighbor_channel(const float * state)
{
  constexpr std::array<BevChannel, NEIGHBOR_TYPE_NUM> CHANNEL_OF_TYPE = {
    BEV_CH_VEHICLE, BEV_CH_PEDESTRIAN, BEV_CH_BICYCLE};
  int64_t best = 0;
  for (int64_t i = 1; i < NEIGHBOR_TYPE_NUM; ++i) {
    if (state[NEIGHBOR_IDX_TYPE_START + i] > state[NEIGHBOR_IDX_TYPE_START + best]) {
      best = i;
    }
  }
  return CHANNEL_OF_TYPE[best];
}

void draw_lanes(
  Canvas & canvas, const BevView & view, const BevTransform & transform,
  const InputDataMap & input_data_map)
{
  const TensorView lanes(input_data_map, "lanes", POINTS_PER_SEGMENT * SEGMENT_POINT_DIM);

  for (int64_t i = 0; i < NUM_SEGMENTS_IN_LANE; ++i) {
    const float * segment = lanes.row(i);
    const int64_t num_points = valid_prefix_length(segment, POINTS_PER_SEGMENT, SEGMENT_POINT_DIM);
    if (num_points < 2) {
      continue;
    }

    std::vector<Eigen::Vector2d> center;
    std::vector<Eigen::Vector2d> left;
    std::vector<Eigen::Vector2d> right;
    center.reserve(num_points);
    left.reserve(num_points);
    right.reserve(num_points);
    for (int64_t p = 0; p < num_points; ++p) {
      const float * point = segment + p * SEGMENT_POINT_DIM;
      center.push_back(transform(point[X], point[Y]));
      // The boundaries are stored as offsets from the centerline point.
      left.push_back(transform(point[X] + point[LB_X], point[Y] + point[LB_Y]));
      right.push_back(transform(point[X] + point[RB_X], point[Y] + point[RB_Y]));
    }

    draw_polyline(canvas[BEV_CH_LANE_CENTERLINE], view, center, LANE_CENTERLINE_THICKNESS);
    draw_polyline(canvas[BEV_CH_LANE_BOUNDARY], view, left, LANE_BOUNDARY_THICKNESS);
    draw_polyline(canvas[BEV_CH_LANE_BOUNDARY], view, right, LANE_BOUNDARY_THICKNESS);

    // The state one-hot is replicated across the segment, so the first point carries it.
    const std::optional<BevChannel> light_channel = traffic_light_channel(segment);
    if (light_channel.has_value()) {
      draw_polyline(canvas[light_channel.value()], view, center, TRAFFIC_LIGHT_THICKNESS);
    }
  }
}

void draw_route(
  Canvas & canvas, const BevView & view, const BevTransform & transform,
  const InputDataMap & input_data_map)
{
  const TensorView route_lanes(
    input_data_map, "route_lanes", POINTS_PER_SEGMENT * SEGMENT_POINT_DIM);

  for (int64_t i = 0; i < NUM_SEGMENTS_IN_ROUTE; ++i) {
    const float * segment = route_lanes.row(i);
    const int64_t num_points = valid_prefix_length(segment, POINTS_PER_SEGMENT, SEGMENT_POINT_DIM);
    if (num_points < 2) {
      continue;
    }

    std::vector<Eigen::Vector2d> center;
    center.reserve(num_points);
    for (int64_t p = 0; p < num_points; ++p) {
      const float * point = segment + p * SEGMENT_POINT_DIM;
      center.push_back(transform(point[X], point[Y]));
    }
    draw_polyline(canvas[BEV_CH_ROUTE], view, center, ROUTE_THICKNESS);
  }
}

void draw_polygons(
  Canvas & canvas, const BevView & view, const BevTransform & transform,
  const InputDataMap & input_data_map)
{
  constexpr int64_t POLYGON_POINT_DIM = 2 + POLYGON_TYPE_NUM;
  const TensorView polygons(input_data_map, "polygons", POINTS_PER_POLYGON * POLYGON_POINT_DIM);

  for (int64_t i = 0; i < NUM_POLYGONS; ++i) {
    const float * polygon = polygons.row(i);
    const int64_t num_points = valid_prefix_length(polygon, POINTS_PER_POLYGON, POLYGON_POINT_DIM);
    if (num_points < 3) {
      continue;
    }

    std::vector<cv::Point> contour;
    contour.reserve(num_points);
    for (int64_t p = 0; p < num_points; ++p) {
      const float * point = polygon + p * POLYGON_POINT_DIM;
      contour.push_back(to_pixel(view, transform(point[X], point[Y])));
    }
    fill_pixel_polygon(canvas[BEV_CH_POLYGON], contour);
  }
}

void draw_line_strings(
  Canvas & canvas, const BevView & view, const BevTransform & transform,
  const InputDataMap & input_data_map)
{
  constexpr int64_t LINE_STRING_POINT_DIM = 2 + LINE_STRING_TYPE_NUM;
  const TensorView line_strings(
    input_data_map, "line_strings", POINTS_PER_LINE_STRING * LINE_STRING_POINT_DIM);

  for (int64_t i = 0; i < NUM_LINE_STRINGS; ++i) {
    const float * line_string = line_strings.row(i);
    const int64_t num_points =
      valid_prefix_length(line_string, POINTS_PER_LINE_STRING, LINE_STRING_POINT_DIM);
    if (num_points < 2) {
      continue;
    }

    std::vector<Eigen::Vector2d> points;
    points.reserve(num_points);
    for (int64_t p = 0; p < num_points; ++p) {
      const float * point = line_string + p * LINE_STRING_POINT_DIM;
      points.push_back(transform(point[X], point[Y]));
    }
    draw_polyline(canvas[BEV_CH_LINE_STRING], view, points, LINE_STRING_THICKNESS);
  }
}

void draw_static_objects(
  Canvas & canvas, const BevView & view, const BevTransform & transform,
  const InputDataMap & input_data_map, const double ego_yaw)
{
  const int64_t static_object_dim = STATIC_OBJECTS_SHAPE[2];
  const TensorView static_objects(input_data_map, "static_objects", static_object_dim);

  for (int64_t i = 0; i < NUM_STATIC_OBJECTS; ++i) {
    const float * object = static_objects.row(i);
    const double pose_magnitude =
      std::abs(object[0]) + std::abs(object[1]) + std::abs(object[2]) + std::abs(object[3]);
    if (pose_magnitude < VALID_EPS) {
      continue;
    }
    draw_box(
      canvas[BEV_CH_STATIC_OBJECT], view, transform(object[X], object[Y]),
      object[STATIC_IDX_LENGTH], object[STATIC_IDX_WIDTH],
      yaw_from_cos_sin(object[STATIC_IDX_COS], object[STATIC_IDX_SIN]) - ego_yaw);
  }
}

/// Draw each neighbor's past track plus its current box.
void draw_neighbors(
  Canvas & canvas, const BevView & view, const BevTransform & transform,
  const InputDataMap & input_data_map, const double ego_yaw)
{
  const TensorView neighbors(
    input_data_map, "neighbor_agents_past", INPUT_T_WITH_CURRENT * NEIGHBOR_STATE_DIM);

  for (int64_t i = 0; i < MAX_NUM_NEIGHBORS; ++i) {
    const float * history = neighbors.row(i);
    const float * state = history + INPUT_T * NEIGHBOR_STATE_DIM;
    const double pose_magnitude =
      std::abs(state[0]) + std::abs(state[1]) + std::abs(state[2]) + std::abs(state[3]);
    if (pose_magnitude < VALID_EPS) {
      continue;
    }

    std::vector<Eigen::Vector2d> track;
    track.reserve(INPUT_T_WITH_CURRENT);
    for (int64_t t = 0; t < INPUT_T_WITH_CURRENT; ++t) {
      const float * step = history + t * NEIGHBOR_STATE_DIM;
      if (std::abs(step[X]) + std::abs(step[Y]) <= VALID_EPS) {
        continue;
      }
      track.push_back(transform(step[X], step[Y]));
    }
    draw_polyline(canvas[BEV_CH_NEIGHBOR_HISTORY], view, track, HISTORY_THICKNESS);

    draw_box(
      canvas[neighbor_channel(state)], view, transform(state[X], state[Y]),
      state[NEIGHBOR_IDX_LENGTH], state[NEIGHBOR_IDX_WIDTH],
      yaw_from_cos_sin(state[NEIGHBOR_IDX_COS], state[NEIGHBOR_IDX_SIN]) - ego_yaw);
  }
}

void draw_ego(
  Canvas & canvas, const BevView & view, const BevTransform & transform,
  const InputDataMap & input_data_map, const double ego_yaw)
{
  const TensorView ego_agent_past(input_data_map, "ego_agent_past", POSE_DIM);
  std::vector<Eigen::Vector2d> track;
  track.reserve(INPUT_T_WITH_CURRENT);
  for (int64_t t = 0; t < INPUT_T_WITH_CURRENT; ++t) {
    const float * step = ego_agent_past.row(t);
    track.push_back(transform(step[EGO_AGENT_PAST_IDX_X], step[EGO_AGENT_PAST_IDX_Y]));
  }
  draw_polyline(canvas[BEV_CH_EGO_HISTORY], view, track, HISTORY_THICKNESS);

  const std::vector<float> & ego_current_state = input_data_map.at("ego_current_state");
  const std::vector<float> & ego_shape = input_data_map.at("ego_shape");
  draw_box(
    canvas[BEV_CH_EGO], view, transform(ego_current_state[X], ego_current_state[Y]),
    ego_shape[1] /* length */, ego_shape[2] /* width */,
    yaw_from_cos_sin(ego_current_state[2], ego_current_state[3]) - ego_yaw);
}

void draw_goal_pose(
  Canvas & canvas, const BevView & view, const BevTransform & transform,
  const InputDataMap & input_data_map, const double ego_yaw)
{
  const std::vector<float> & goal_pose = input_data_map.at("goal_pose");
  if (std::abs(goal_pose[X]) + std::abs(goal_pose[Y]) < VALID_EPS) {
    return;
  }
  const double goal_yaw = yaw_from_cos_sin(goal_pose[2], goal_pose[3]);
  draw_pose_marker(
    canvas[BEV_CH_GOAL_POSE], view, transform(goal_pose[X], goal_pose[Y]), goal_yaw - ego_yaw);
}

/// Rasterize one scale in place, over planes wrapping `image` at `plane_offset`.
void render_view(
  std::vector<uint8_t> & image, const size_t plane_offset, const double extent_m,
  const InputDataMap & input_data_map, const BevTransform & transform, const double ego_yaw)
{
  constexpr size_t PLANE_SIZE = static_cast<size_t>(BEV_IMAGE_SIZE) * BEV_IMAGE_SIZE;
  Canvas canvas;
  for (int64_t channel = 0; channel < BEV_NUM_CHANNELS; ++channel) {
    canvas[channel] = cv::Mat(
      static_cast<int>(BEV_IMAGE_SIZE), static_cast<int>(BEV_IMAGE_SIZE), CV_8UC1,
      image.data() + plane_offset + static_cast<size_t>(channel) * PLANE_SIZE);
  }

  const BevView view = make_view(extent_m);
  draw_lanes(canvas, view, transform, input_data_map);
  draw_route(canvas, view, transform, input_data_map);
  draw_polygons(canvas, view, transform, input_data_map);
  draw_line_strings(canvas, view, transform, input_data_map);
  draw_static_objects(canvas, view, transform, input_data_map, ego_yaw);
  draw_goal_pose(canvas, view, transform, input_data_map, ego_yaw);
  draw_neighbors(canvas, view, transform, input_data_map, ego_yaw);
  draw_ego(canvas, view, transform, input_data_map, ego_yaw);
}

}  // namespace

std::vector<uint8_t> create_bev_image(const InputDataMap & input_data_map, const int64_t batch_size)
{
  assert(batch_size >= 1);

  constexpr size_t SAMPLE_SIZE =
    static_cast<size_t>(BEV_NUM_SCALES) * BEV_NUM_CHANNELS * BEV_IMAGE_SIZE * BEV_IMAGE_SIZE;
  constexpr size_t SCALE_SIZE = SAMPLE_SIZE / BEV_NUM_SCALES;

  std::vector<uint8_t> image(SAMPLE_SIZE * static_cast<size_t>(batch_size), 0U);

  const std::vector<float> & ego_current_state = input_data_map.at("ego_current_state");
  const double ego_yaw = yaw_from_cos_sin(ego_current_state[2], ego_current_state[3]);
  const BevTransform transform(ego_current_state[X], ego_current_state[Y], ego_yaw);

  for (int64_t scale = 0; scale < BEV_NUM_SCALES; ++scale) {
    render_view(
      image, static_cast<size_t>(scale) * SCALE_SIZE, BEV_VIEW_EXTENTS_M[scale], input_data_map,
      transform, ego_yaw);
  }

  // Every scene tensor the renderer reads is batch-replicated from the same frame, so the whole
  // batch shares one raster.
  for (int64_t b = 1; b < batch_size; ++b) {
    std::copy(
      image.begin(), image.begin() + static_cast<std::ptrdiff_t>(SAMPLE_SIZE),
      image.begin() + static_cast<std::ptrdiff_t>(static_cast<size_t>(b) * SAMPLE_SIZE));
  }
  return image;
}

std::vector<uint8_t> colorize_bev_image(const std::vector<uint8_t> & bev_image, const int64_t scale)
{
  constexpr size_t PLANE_SIZE = static_cast<size_t>(BEV_IMAGE_SIZE) * BEV_IMAGE_SIZE;
  assert(scale >= 0 && scale < BEV_NUM_SCALES);
  assert(bev_image.size() >= (static_cast<size_t>(scale) + 1) * BEV_NUM_CHANNELS * PLANE_SIZE);

  std::vector<uint8_t> preview(PLANE_SIZE * 3, 0U);
  for (const BevChannel channel : DRAW_ORDER) {
    const uint8_t * plane =
      bev_image.data() +
      (static_cast<size_t>(scale) * BEV_NUM_CHANNELS + static_cast<size_t>(channel)) * PLANE_SIZE;
    const auto & color = COLOR_BGR_OF_CHANNEL[channel];
    for (size_t pixel = 0; pixel < PLANE_SIZE; ++pixel) {
      if (plane[pixel] == 0U) {
        continue;
      }
      preview[pixel * 3 + 0] = color[0];
      preview[pixel * 3 + 1] = color[1];
      preview[pixel * 3 + 2] = color[2];
    }
  }
  return preview;
}

}  // namespace autoware::diffusion_planner::preprocess
