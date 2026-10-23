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

#include "autoware/tensorrt_e2e/bev_warp_target.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>

namespace
{
std::array<double, 4> pose(const double x, const double y, const double yaw)
{
  return {x, y, std::cos(yaw), std::sin(yaw)};
}

double heading(const std::array<double, 4> & p)
{
  return std::atan2(p[3], p[2]);
}

bool near(const double a, const double b, const double tol)
{
  return std::abs(a - b) <= tol;
}
}  // namespace

int main()
{
  using autoware::tensorrt_e2e::BevWarpTarget;
  constexpr double latency = 0.11;

  // Off: the measured planning pose, exactly.
  {
    BevWarpTarget off(0.0);
    const auto planning = pose(10.0, 2.0, 0.3);
    const auto out = off.update(pose(9.5, 1.9, 0.28), planning, 0.2, latency, 1.0);
    assert(out == planning);
  }

  // A steady turn: the filter settles on the yaw rate, and the target is the planning pose exactly.
  {
    BevWarpTarget smooth(0.3);
    constexpr double rate = 0.2;
    double t = 0.0;
    std::array<double, 4> out{};
    for (int i = 0; i < 50; ++i, t += 0.1) {
      const double yaw_cloud = rate * t;
      const auto planning = pose(t, 0.0, yaw_cloud + rate * latency);
      out = smooth.update(pose(t, 0.0, yaw_cloud), planning, rate, latency, t);
      assert(out[0] == planning[0] && out[1] == planning[1]);
      assert(near(heading(out), heading(planning), 1e-12));
    }
  }

  // A 1.6 Hz wobble on a steady turn: the target follows the turn, not the wobble.
  {
    BevWarpTarget smooth(0.3);
    constexpr double rate = 0.2, wobble = 0.05, omega = 2.0 * M_PI * 1.6;
    double worst_measured = 0.0, worst_target = 0.0, t = 0.0;
    for (int i = 0; i < 100; ++i, t += 0.1) {
      // yaw(t) = rate t + wobble / omega sin(omega t): the yaw rate swings by +-wobble
      const auto yaw = [&](const double s) {
        return rate * s + wobble / omega * std::sin(omega * s);
      };
      const double yaw_rate_now = rate + wobble * std::cos(omega * (t + latency));
      const auto out = smooth.update(
        pose(0.0, 0.0, yaw(t)), pose(1.0, 0.0, yaw(t + latency)), yaw_rate_now, latency,
        t + latency);
      if (i < 20) {
        continue;  // settled after 2 s
      }
      const double smooth_turn = rate * latency;
      worst_measured = std::max(worst_measured, std::abs(yaw(t + latency) - yaw(t) - smooth_turn));
      worst_target = std::max(worst_target, std::abs(heading(out) - yaw(t) - smooth_turn));
    }
    assert(worst_measured > 4e-3);  // the measured turn during L carries ~5 mrad of wobble
    assert(worst_target < 0.5 * worst_measured);  // at 1.6 Hz a 0.3 s filter keeps ~1/3 of it
  }

  // Non-finite input leaves the planning pose; a gap restarts the filter at the new yaw rate.
  {
    BevWarpTarget smooth(0.3);
    const auto planning = pose(1.0, 1.0, 0.1);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    assert(smooth.update(pose(0.0, 0.0, 0.0), planning, nan, 0.1, 0.0) == planning);
    smooth.update(pose(0.0, 0.0, 0.0), planning, 0.5, 0.1, 1.0);
    smooth.update(pose(0.0, 0.0, 0.0), planning, -0.5, 0.1, 5.0);
    assert(near(*smooth.filtered_yaw_rate(), -0.5, 1e-12));
    // A second pass at the same stamp does not filter twice.
    smooth.update(pose(0.0, 0.0, 0.0), planning, 0.5, 0.1, 5.1);
    const double once = *smooth.filtered_yaw_rate();
    smooth.update(pose(0.0, 0.0, 0.0), planning, 0.5, 0.1, 5.1);
    assert(*smooth.filtered_yaw_rate() == once);
    smooth.reset();
    assert(!smooth.filtered_yaw_rate());
  }
  return 0;
}
