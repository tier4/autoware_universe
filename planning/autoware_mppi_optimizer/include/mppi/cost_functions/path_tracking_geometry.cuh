/**
 * Shared host/device geometry for analytic path-tracking costs (polyline distance, OBB overlap).
 */
#pragma once

#ifndef MPPI_COST_FUNCTIONS_PATH_TRACKING_GEOMETRY_CUH_
#define MPPI_COST_FUNCTIONS_PATH_TRACKING_GEOMETRY_CUH_

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace mppi
{
namespace cost
{
namespace detail
{
#ifdef __CUDA_ARCH__
__device__ inline float clampUnitInterval(const float t)
{
  return fmaxf(0.0F, fminf(1.0F, t));
}

__device__ inline float vectorLength(const float dx, const float dy)
{
  return sqrtf(dx * dx + dy * dy);
}
#else
inline float clampUnitInterval(const float t)
{
  return std::max(0.0F, std::min(1.0F, t));
}

inline float vectorLength(const float dx, const float dy)
{
  return std::sqrt(dx * dx + dy * dy);
}
#endif

__host__ __device__ inline float distancePointToSegment(
  const float px, const float py, const float x0, const float y0, const float x1, const float y1)
{
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  const float len_sq = dx * dx + dy * dy;
  if (len_sq < 1.0E-8F) {
    return vectorLength(px - x0, py - y0);
  }

  const float t = clampUnitInterval(((px - x0) * dx + (py - y0) * dy) / len_sq);
  return vectorLength(px - (x0 + t * dx), py - (y0 + t * dy));
}

__host__ __device__ inline float cross2(
  const float ax, const float ay, const float bx, const float by)
{
  return ax * by - ay * bx;
}

/** Euclidean distance between two finite line segments (zero when they intersect). */
__host__ __device__ inline float distanceSegmentToSegment(
  const float ax0, const float ay0, const float ax1, const float ay1, const float bx0,
  const float by0, const float bx1, const float by1)
{
  const float adx = ax1 - ax0;
  const float ady = ay1 - ay0;
  const float bdx = bx1 - bx0;
  const float bdy = by1 - by0;
  const float denominator = cross2(adx, ady, bdx, bdy);
  const float qpx = bx0 - ax0;
  const float qpy = by0 - ay0;

  if (fabsf(denominator) > 1.0E-8F) {
    const float t = cross2(qpx, qpy, bdx, bdy) / denominator;
    const float u = cross2(qpx, qpy, adx, ady) / denominator;
    if (t >= 0.0F && t <= 1.0F && u >= 0.0F && u <= 1.0F) {
      return 0.0F;
    }
  }

  const float d0 = distancePointToSegment(ax0, ay0, bx0, by0, bx1, by1);
  const float d1 = distancePointToSegment(ax1, ay1, bx0, by0, bx1, by1);
  const float d2 = distancePointToSegment(bx0, by0, ax0, ay0, ax1, ay1);
  const float d3 = distancePointToSegment(bx1, by1, ax0, ay0, ax1, ay1);
  return fminf(fminf(d0, d1), fminf(d2, d3));
}

/** Perpendicular distance to the infinite line through (x0,y0)-(x1,y1). */
__host__ __device__ inline float perpendicularDistanceToLine(
  const float px, const float py, const float x0, const float y0, const float x1, const float y1)
{
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  const float len_sq = dx * dx + dy * dy;
  if (len_sq < 1.0E-8F) {
    return vectorLength(px - x0, py - y0);
  }
#ifdef __CUDA_ARCH__
  return fabsf((px - x0) * dy - (py - y0) * dx) / sqrtf(len_sq);
#else
  return std::fabs((px - x0) * dy - (py - y0) * dx) / std::sqrt(len_sq);
#endif
}

/**
 * Signed cross-track error to a polyline (+ = left of segment tangent). Interior
 * projections use the closest segment; projections past the first/last vertex use
 * the extended terminal segment (so overshooting the horizon is not treated as
 * lateral departure).
 */
struct PolylineProjection
{
  /** Signed lateral offset [m]; + = left of forward segment tangent. */
  float lateral_distance = 0.0F;
  int best_i = 0;
  /** Unclamped segment parameter from the closest-segment search. */
  float best_t_raw = 0.5F;
};

/** Endpoint differences for a finite segment. These subtractions are independent of the query. */
struct PolylineSegmentDelta
{
  float dx = 0.0F;
  float dy = 0.0F;
};

inline constexpr int kPolylineGroupSize = 8;
// With both vertices and queries in this range, residuals, squared lengths and the
// numerator/divisor in the original projector cannot overflow (len_sq >= 1e-8).
inline constexpr float kPolylinePruningCoordinateLimit = 0x1p30F;

struct PolylineGroupBounds
{
  float min_x = 0.0F;
  float min_y = 0.0F;
  float max_x = 0.0F;
  float max_y = 0.0F;
};

constexpr int polylineGroupCount(const int n_pts, const int group_size = kPolylineGroupSize)
{
  return n_pts < 2 ? 0 : (n_pts - 2) / group_size + 1;
}

/** Host preparation; output has room for polylineGroupCount(n_pts) entries.
 *
 * Bound the COMPUTED footpoint, not just the ideal segment. For t in [0,1], both
 * RN(x0 + RN(t*dx)) and RN(x0 + t*dx) lie between x0 and RN(x0 + dx), by monotonicity.
 * One outward nextafter also encloses the unrounded endpoint sum. This covers
 * cached subtraction roundoff and either FMA setting without changing the candidate
 * calculation. Degenerate segments return the distance to x0, also inside the box.
 * A false return invalidates the entire array; callers must use exhaustive search.
 */
template <int GroupSize = kPolylineGroupSize>
inline bool buildPolylineGroupBounds(
  const float * x, const float * y, const PolylineSegmentDelta * deltas, const int n_pts,
  PolylineGroupBounds * bounds)
{
  static_assert(GroupSize > 0, "A group must contain at least one segment");
  if (n_pts < 2) return false;
  for (int i = 0; i < n_pts; ++i) {
    if (
      !(std::fabs(x[i]) <= kPolylinePruningCoordinateLimit) ||
      !(std::fabs(y[i]) <= kPolylinePruningCoordinateLimit)) {
      return false;
    }
  }
  for (int group = 0; group < polylineGroupCount(n_pts, GroupSize); ++group) {
    auto & box = bounds[group];
    box = {FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};
    const int end = std::min(n_pts - 1, (group + 1) * GroupSize);
    for (int i = group * GroupSize; i < end; ++i) {
      const float end_x = x[i] + deltas[i].dx;
      const float end_y = y[i] + deltas[i].dy;
      if (!std::isfinite(end_x) || !std::isfinite(end_y)) return false;
      box.min_x = std::min(box.min_x, std::min(x[i], std::nextafter(end_x, -FLT_MAX)));
      box.min_y = std::min(box.min_y, std::min(y[i], std::nextafter(end_y, -FLT_MAX)));
      box.max_x = std::max(box.max_x, std::max(x[i], std::nextafter(end_x, FLT_MAX)));
      box.max_y = std::max(box.max_y, std::max(y[i], std::nextafter(end_y, FLT_MAX)));
    }
  }
  return true;
}

/** Per-query diagnostic counters; production instantiations compile out all updates. */
struct PolylineSearchStats
{
  unsigned int groups_examined = 0;
  unsigned int groups_pruned = 0;
  unsigned int segments_evaluated = 0;
  unsigned int exhaustive_fallbacks = 0;
};

__host__ __device__ inline PolylineSegmentDelta makePolylineSegmentDelta(
  const float x0, const float y0, const float x1, const float y1)
{
  return {x1 - x0, y1 - y0};
}

/** Distance and unclamped parameter of point (px,py) onto segment i. */
__host__ __device__ inline void distanceToPolylineSegment(
  const float px, const float py, const float * poly_x, const float * poly_y, const int i,
  float & dist, float & t_raw)
{
  const float x0 = poly_x[i];
  const float y0 = poly_y[i];
  const float x1 = poly_x[i + 1];
  const float y1 = poly_y[i + 1];
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  const float len_sq = dx * dx + dy * dy;
  if (len_sq < 1.0E-8F) {
    dist = vectorLength(px - x0, py - y0);
    t_raw = 0.0F;
    return;
  }
  t_raw = ((px - x0) * dx + (py - y0) * dy) / len_sq;
  const float t = clampUnitInterval(t_raw);
  dist = vectorLength(px - (x0 + t * dx), py - (y0 + t * dy));
}

/** Same arithmetic and finite-segment conventions as distanceToPolylineSegment. */
__host__ __device__ inline void distanceToPreparedPolylineSegment(
  const float px, const float py, const float * poly_x, const float * poly_y,
  const PolylineSegmentDelta * deltas, const int i, float & dist, float & t_raw)
{
  const float x0 = poly_x[i];
  const float y0 = poly_y[i];
  const float dx = deltas[i].dx;
  const float dy = deltas[i].dy;
  // Keep the original multiply/add, division, square root and distance comparison on
  // the querying processor. A host-precomputed squared length can round differently
  // from the device expression and change a near-tie or a safety boundary.
  const float len_sq = dx * dx + dy * dy;
  if (len_sq < 1.0E-8F) {
    dist = vectorLength(px - x0, py - y0);
    t_raw = 0.0F;
    return;
  }
  t_raw = ((px - x0) * dx + (py - y0) * dy) / len_sq;
  const float t = clampUnitInterval(t_raw);
  dist = vectorLength(px - (x0 + t * dx), py - (y0 + t * dy));
}

/** Signed lateral offset from segment; positive = left of forward tangent. */
__host__ __device__ inline float signedLateralOffsetPointToSegment(
  const float px, const float py, const float x0, const float y0, const float x1, const float y1)
{
  const float dx = x1 - x0;
  const float dy = y1 - y0;
  const float len_sq = dx * dx + dy * dy;
  if (len_sq < 1.0E-8F) {
    return px - x0;
  }

  const float t = clampUnitInterval(((px - x0) * dx + (py - y0) * dy) / len_sq);
  const float cx = x0 + t * dx;
  const float cy = y0 + t * dy;
  const float len = vectorLength(dx, dy);
  return ((px - cx) * (-dy) + (py - cy) * dx) / len;
}

__host__ __device__ inline void finalizePolylineProjection(
  PolylineProjection & result, const float px, const float py, const float * poly_x,
  const float * poly_y, const int n_pts, const int best_i, const float best_t_raw,
  const float /*best_dist*/)
{
  result.best_i = best_i;
  result.best_t_raw = best_t_raw;
  if (n_pts < 2) {
    result.lateral_distance = (n_pts == 1) ? (px - poly_x[0]) : 0.0F;
    return;
  }

  const float x0 = poly_x[best_i];
  const float y0 = poly_y[best_i];
  const float x1 = poly_x[best_i + 1];
  const float y1 = poly_y[best_i + 1];
  // Past the start/end of the finite polyline: signed cross-track to the extended tip.
  if ((best_i == 0 && best_t_raw < 0.0F) || (best_i == n_pts - 2 && best_t_raw > 1.0F)) {
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float len = vectorLength(dx, dy);
    if (len < 1.0E-8F) {
      result.lateral_distance = px - x0;
    } else {
      result.lateral_distance = ((px - x0) * (-dy) + (py - y0) * dx) / len;
    }
    return;
  }
  result.lateral_distance = signedLateralOffsetPointToSegment(px, py, x0, y0, x1, y1);
}

/**
 * Globally closest finite-segment projection onto a polyline, with O(n) query work.
 * Equal computed distances select the lowest segment index, independent of hints.
 * Finalization retains the signed lateral offset and extended-tip conventions above.
 * @param hint_i optional first candidate; invalid indices are ignored. A texture seed or
 *               previous projection only changes evaluation order, never search coverage.
 * @param max_local_search_steps retained for source compatibility, ignored. A local step
 *               budget cannot establish the global nearest segment on arbitrary polylines.
 */
__host__ __device__ inline PolylineProjection projectPointToPolyline(
  const float px, const float py, const float * poly_x, const float * poly_y, const int n_pts,
  const int hint_i = -1, const int /*max_local_search_steps*/ = -1)
{
  PolylineProjection result;
  if (n_pts <= 0) {
    return result;
  }
  if (n_pts == 1) {
    result.lateral_distance = px - poly_x[0];
    result.best_i = 0;
    result.best_t_raw = 0.0F;
    return result;
  }

  const int n_seg = n_pts - 1;
  float best_dist = FLT_MAX;
  int best_i = 0;
  float best_t_raw = 0.5F;

  const bool valid_hint = hint_i >= 0 && hint_i < n_seg;
  if (valid_hint) {
    best_i = hint_i;
    distanceToPolylineSegment(px, py, poly_x, poly_y, best_i, best_dist, best_t_raw);
  }
  // A local minimum (including a texture-cell seed) does not exclude a closer,
  // nonadjacent branch. Evaluate every remaining segment, including equal-distance ties.
  for (int i = 0; i < n_seg; ++i) {
    if (valid_hint && i == hint_i) {
      continue;
    }
    float dist = 0.0F;
    float t_raw = 0.0F;
    distanceToPolylineSegment(px, py, poly_x, poly_y, i, dist, t_raw);
    if (dist < best_dist || (dist == best_dist && i < best_i)) {
      best_dist = dist;
      best_i = i;
      best_t_raw = t_raw;
    }
  }

  finalizePolylineProjection(result, px, py, poly_x, poly_y, n_pts, best_i, best_t_raw, best_dist);
  return result;
}

/** Exhaustive projection using geometry prepared from the same local-frame vertices. */
__host__ __device__ inline PolylineProjection projectPointToPreparedPolyline(
  const float px, const float py, const float * poly_x, const float * poly_y,
  const PolylineSegmentDelta * deltas, const int n_pts, const int hint_i = -1)
{
  PolylineProjection result;
  if (n_pts <= 0) {
    return result;
  }
  if (n_pts == 1) {
    result.lateral_distance = px - poly_x[0];
    result.best_t_raw = 0.0F;
    return result;
  }

  const int n_seg = n_pts - 1;
  float best_dist = FLT_MAX;
  int best_i = 0;
  float best_t_raw = 0.5F;
  const bool valid_hint = hint_i >= 0 && hint_i < n_seg;
  if (valid_hint) {
    best_i = hint_i;
    distanceToPreparedPolylineSegment(
      px, py, poly_x, poly_y, deltas, best_i, best_dist, best_t_raw);
  }
  for (int i = 0; i < n_seg; ++i) {
    if (valid_hint && i == hint_i) {
      continue;
    }
    float dist = 0.0F;
    float t_raw = 0.0F;
    distanceToPreparedPolylineSegment(px, py, poly_x, poly_y, deltas, i, dist, t_raw);
    if (dist < best_dist || (dist == best_dist && i < best_i)) {
      best_dist = dist;
      best_i = i;
      best_t_raw = t_raw;
    }
  }
  finalizePolylineProjection(result, px, py, poly_x, poly_y, n_pts, best_i, best_t_raw, best_dist);
  return result;
}

#ifdef __CUDACC__
/** Round each nonnegative operation down, including sqrt: this bounds the computed
 * distance, even when the candidate's squared norm uses a fused multiply-add.
 * Comparing squared distances to best_dist*best_dist would not preserve sqrt ties.
 */
__device__ inline float distanceToPolylineGroupLowerBound(
  const float px, const float py, const PolylineGroupBounds & box)
{
  const float dx = fmaxf(0.0F, fmaxf(__fsub_rd(box.min_x, px), __fsub_rd(px, box.max_x)));
  const float dy = fmaxf(0.0F, fmaxf(__fsub_rd(box.min_y, py), __fsub_rd(py, box.max_y)));
  return __fsqrt_rd(__fadd_rd(__fmul_rd(dx, dx), __fmul_rd(dy, dy)));
}

/** CMake certifies the math flags for this translation unit; external consumers
 * without that contract use exhaustive search. Fast/FTZ/approximate sqrt builds
 * cannot use the directed-rounding proof above.
 */
__host__ __device__ constexpr bool polylinePruningMathSupported()
{
#if defined(MPPI_PROJECTION_PRECISE_MATH) && MPPI_PROJECTION_PRECISE_MATH && \
  !defined(__FAST_MATH__) && !defined(__USE_FAST_MATH__)
  return true;
#else
  return false;
#endif
}

template <bool CollectStats = false, int GroupSize = kPolylineGroupSize>
__device__ inline PolylineProjection projectPointToPrunedPolyline(
  const float px, const float py, const float * poly_x, const float * poly_y,
  const PolylineSegmentDelta * deltas, const PolylineGroupBounds * bounds, const bool bounds_valid,
  const int n_pts, const int hint_i = -1, PolylineSearchStats * stats = nullptr)
{
  static_assert(GroupSize > 0, "A group must contain at least one segment");
  if constexpr (CollectStats) *stats = {};
  if (
    !polylinePruningMathSupported() || !bounds_valid || bounds == nullptr ||
    n_pts <= GroupSize + 1 || !(fabsf(px) <= kPolylinePruningCoordinateLimit) ||
    !(fabsf(py) <= kPolylinePruningCoordinateLimit)) {
    if constexpr (CollectStats) {
      stats->exhaustive_fallbacks = 1;
      stats->segments_evaluated = n_pts > 1 ? n_pts - 1 : 0;
    }
    return projectPointToPreparedPolyline(px, py, poly_x, poly_y, deltas, n_pts, hint_i);
  }

  const int n_seg = n_pts - 1;
  float best_dist = FLT_MAX;
  int best_i = 0;
  float best_t_raw = 0.5F;
  const bool valid_hint = hint_i >= 0 && hint_i < n_seg;
  if (valid_hint) {
    best_i = hint_i;
    distanceToPreparedPolylineSegment(
      px, py, poly_x, poly_y, deltas, best_i, best_dist, best_t_raw);
    if constexpr (CollectStats) ++stats->segments_evaluated;
  }
  for (int group = 0; group * GroupSize < n_seg; ++group) {
    if constexpr (CollectStats) ++stats->groups_examined;
    // Strict comparison preserves equal computed distances and lowest-index ties,
    // even when the hint belongs to a later group. NaN best_dist never prunes.
    if (distanceToPolylineGroupLowerBound(px, py, bounds[group]) > best_dist) {
      if constexpr (CollectStats) ++stats->groups_pruned;
      continue;
    }
    const int end = min(n_seg, (group + 1) * GroupSize);
    for (int i = group * GroupSize; i < end; ++i) {
      if (valid_hint && i == hint_i) continue;
      float dist = 0.0F;
      float t_raw = 0.0F;
      distanceToPreparedPolylineSegment(px, py, poly_x, poly_y, deltas, i, dist, t_raw);
      if constexpr (CollectStats) ++stats->segments_evaluated;
      if (dist < best_dist || (dist == best_dist && i < best_i)) {
        best_dist = dist;
        best_i = i;
        best_t_raw = t_raw;
      }
    }
  }
  PolylineProjection result;
  finalizePolylineProjection(result, px, py, poly_x, poly_y, n_pts, best_i, best_t_raw, best_dist);
  return result;
}
#endif  // __CUDACC__

__host__ __device__ inline float crossTrackDistanceToPolyline(
  const float px, const float py, const float * poly_x, const float * poly_y, const int n_pts)
{
  return projectPointToPolyline(px, py, poly_x, poly_y, n_pts).lateral_distance;
}

/**
 * Arc length along a polyline at the projected footpoint.
 * @param total_s full polyline length (final cumulative chord length).
 * Tip overhang clamps path length to [0, total_s]; remaining = total_s - path_length.
 * When past the tip (last segment, t_raw > 1), overshoot_distance_s is the along-track
 * extension beyond the terminal vertex; otherwise 0.
 */
__host__ __device__ inline void pathLengthAtProjection(
  const PolylineProjection & proj, const float * poly_x, const float * poly_y,
  const float * cumulative_s, const int n_pts, const float total_s, float & path_length_s,
  float & remaining_distance_s, float & overshoot_distance_s)
{
  path_length_s = 0.0F;
  remaining_distance_s = total_s;
  overshoot_distance_s = 0.0F;
  if (n_pts <= 0) {
    return;
  }
  if (n_pts == 1) {
    path_length_s = total_s;
    remaining_distance_s = 0.0F;
    return;
  }

  if (proj.best_i == 0 && proj.best_t_raw < 0.0F) {
    path_length_s = 0.0F;
    remaining_distance_s = total_s;
    return;
  }
  if (proj.best_i == n_pts - 2 && proj.best_t_raw > 1.0F) {
    path_length_s = total_s;
    remaining_distance_s = 0.0F;
    overshoot_distance_s =
      (proj.best_t_raw - 1.0F) *
      vectorLength(poly_x[n_pts - 1] - poly_x[n_pts - 2], poly_y[n_pts - 1] - poly_y[n_pts - 2]);
    return;
  }

  const float t = clampUnitInterval(proj.best_t_raw);
  if (cumulative_s != nullptr) {
    const float s0 = cumulative_s[proj.best_i];
    const float s1 = cumulative_s[proj.best_i + 1];
    path_length_s = s0 + t * (s1 - s0);
  } else {
    float s = 0.0F;
    for (int i = 0; i < proj.best_i; ++i) {
      s += vectorLength(poly_x[i + 1] - poly_x[i], poly_y[i + 1] - poly_y[i]);
    }
    path_length_s = s + t * vectorLength(
                              poly_x[proj.best_i + 1] - poly_x[proj.best_i],
                              poly_y[proj.best_i + 1] - poly_y[proj.best_i]);
  }
  remaining_distance_s = total_s - path_length_s;
}

__host__ __device__ inline float dot2(
  const float ax, const float ay, const float bx, const float by)
{
  return ax * bx + ay * by;
}

/** Separating-axis test for two oriented boxes (body x = forward, y = left). */
__host__ __device__ inline bool orientedBoxesOverlap(
  const float acx, const float acy, const float a_cos, const float a_sin, const float a_hl,
  const float a_hw, const float bcx, const float bcy, const float b_cos, const float b_sin,
  const float b_hl, const float b_hw)
{
  const float tx = bcx - acx;
  const float ty = bcy - acy;

  const float a0x = a_cos;
  const float a0y = a_sin;
  const float a1x = -a_sin;
  const float a1y = a_cos;
  const float b0x = b_cos;
  const float b0y = b_sin;
  const float b1x = -b_sin;
  const float b1y = b_cos;

#ifdef __CUDA_ARCH__
#define MPPI_COST_ABS(x) fabsf(x)
#else
#define MPPI_COST_ABS(x) std::fabs(x)
#endif

  float t = MPPI_COST_ABS(dot2(tx, ty, a0x, a0y));
  float r = a_hl + MPPI_COST_ABS(dot2(b0x, b0y, a0x, a0y)) * b_hl +
            MPPI_COST_ABS(dot2(b1x, b1y, a0x, a0y)) * b_hw;
  if (t > r) {
    return false;
  }

  t = MPPI_COST_ABS(dot2(tx, ty, a1x, a1y));
  r = a_hw + MPPI_COST_ABS(dot2(b0x, b0y, a1x, a1y)) * b_hl +
      MPPI_COST_ABS(dot2(b1x, b1y, a1x, a1y)) * b_hw;
  if (t > r) {
    return false;
  }

  t = MPPI_COST_ABS(dot2(tx, ty, b0x, b0y));
  r = b_hl + MPPI_COST_ABS(dot2(a0x, a0y, b0x, b0y)) * a_hl +
      MPPI_COST_ABS(dot2(a1x, a1y, b0x, b0y)) * a_hw;
  if (t > r) {
    return false;
  }

  t = MPPI_COST_ABS(dot2(tx, ty, b1x, b1y));
  r = b_hw + MPPI_COST_ABS(dot2(a0x, a0y, b1x, b1y)) * a_hl +
      MPPI_COST_ABS(dot2(a1x, a1y, b1x, b1y)) * a_hw;
  if (t > r) {
    return false;
  }

#undef MPPI_COST_ABS
  return true;
}

/** Four corners in body frame order: front-left, front-right, rear-right, rear-left. */
__host__ __device__ inline void orientedBoxCorners(
  const float cx, const float cy, const float cos_yaw, const float sin_yaw, const float half_length,
  const float half_width, float corners_x[4], float corners_y[4])
{
  const float fx = cos_yaw;
  const float fy = sin_yaw;
  const float lx = -sin_yaw;
  const float ly = cos_yaw;

  corners_x[0] = cx + half_length * fx + half_width * lx;
  corners_y[0] = cy + half_length * fy + half_width * ly;
  corners_x[1] = cx + half_length * fx - half_width * lx;
  corners_y[1] = cy + half_length * fy - half_width * ly;
  corners_x[2] = cx - half_length * fx - half_width * lx;
  corners_y[2] = cy - half_length * fy - half_width * ly;
  corners_x[3] = cx - half_length * fx + half_width * lx;
  corners_y[3] = cy - half_length * fy + half_width * ly;
}

/** Signed OBB separation: positive clearance, zero contact, negative SAT penetration depth. */
__host__ __device__ inline float signedDistanceBetweenOrientedBoxes(
  const float acx, const float acy, const float a_cos, const float a_sin, const float a_hl,
  const float a_hw, const float bcx, const float bcy, const float b_cos, const float b_sin,
  const float b_hl, const float b_hw)
{
  if (!orientedBoxesOverlap(
        acx, acy, a_cos, a_sin, a_hl, a_hw, bcx, bcy, b_cos, b_sin, b_hl, b_hw)) {
    float ax[4];
    float ay[4];
    float bx[4];
    float by[4];
    orientedBoxCorners(acx, acy, a_cos, a_sin, a_hl, a_hw, ax, ay);
    orientedBoxCorners(bcx, bcy, b_cos, b_sin, b_hl, b_hw, bx, by);
    float min_distance = 1.0E8F;
#pragma unroll
    for (int ai = 0; ai < 4; ++ai) {
      const int an = (ai + 1) & 3;
#pragma unroll
      for (int bi = 0; bi < 4; ++bi) {
        const int bn = (bi + 1) & 3;
        min_distance = fminf(
          min_distance,
          distanceSegmentToSegment(ax[ai], ay[ai], ax[an], ay[an], bx[bi], by[bi], bx[bn], by[bn]));
      }
    }
    return min_distance;
  }

  const float tx = bcx - acx;
  const float ty = bcy - acy;
  const float axes_x[4] = {a_cos, -a_sin, b_cos, -b_sin};
  const float axes_y[4] = {a_sin, a_cos, b_sin, b_cos};
  float min_overlap = 1.0E8F;
#pragma unroll
  for (int i = 0; i < 4; ++i) {
    const float axis_x = axes_x[i];
    const float axis_y = axes_y[i];
    const float center_distance = fabsf(dot2(tx, ty, axis_x, axis_y));
    const float a_radius = a_hl * fabsf(dot2(a_cos, a_sin, axis_x, axis_y)) +
                           a_hw * fabsf(dot2(-a_sin, a_cos, axis_x, axis_y));
    const float b_radius = b_hl * fabsf(dot2(b_cos, b_sin, axis_x, axis_y)) +
                           b_hw * fabsf(dot2(-b_sin, b_cos, axis_x, axis_y));
    min_overlap = fminf(min_overlap, a_radius + b_radius - center_distance);
  }
  return -min_overlap;
}

/** Ray-casting point-in-polygon (closed boundary; vertices in order). */
__host__ __device__ inline bool pointInPolygon(
  const float px, const float py, const float * vertices_x, const float * vertices_y,
  const int vertex_count)
{
  if (vertex_count < 3) {
    return false;
  }

  bool inside = false;
  int j = vertex_count - 1;
  for (int i = 0; i < vertex_count; ++i) {
    const float xi = vertices_x[i];
    const float yi = vertices_y[i];
    const float xj = vertices_x[j];
    const float yj = vertices_y[j];
    const bool intersects =
      ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi + 1.0E-12F) + xi);
    if (intersects) {
      inside = !inside;
    }
    j = i;
  }
  return inside;
}
}  // namespace detail
}  // namespace cost
}  // namespace mppi

#endif  // MPPI_COST_FUNCTIONS_PATH_TRACKING_GEOMETRY_CUH_
