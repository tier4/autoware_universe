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

#include "boundary_simplifier.hpp"

#include <boost/geometry.hpp>

#include <cstring>

namespace autoware::safety_planner
{

BoundarySimplifier::BoundarySimplifier(const double tolerance_m, const std::size_t cache_size)
: tolerance_m_(tolerance_m), cache_(cache_size)
{
}

LineString2d BoundarySimplifier::simplify(const LineString2d & polyline)
{
  if (tolerance_m_ <= 0.0 || polyline.size() < 3) {
    return polyline;
  }
  const auto key = hash(polyline);
  if (const auto cached = cache_.get(key)) {
    return *cached;
  }
  LineString2d simplified;
  boost::geometry::simplify(polyline, simplified, tolerance_m_);
  cache_.put(key, simplified);
  return simplified;
}

std::uint64_t BoundarySimplifier::hash(const LineString2d & polyline)
{
  // FNV-1a over the coordinate bit patterns; equal geometry hashes equal, no float rounding
  constexpr std::uint64_t OFFSET_BASIS = 14695981039346656037ULL;
  constexpr std::uint64_t PRIME = 1099511628211ULL;
  std::uint64_t h = OFFSET_BASIS;
  const auto mix = [&h](const double value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    for (int shift = 0; shift < 64; shift += 8) {
      h ^= (bits >> shift) & 0xFFU;
      h *= PRIME;
    }
  };
  mix(static_cast<double>(polyline.size()));
  for (const auto & vertex : polyline) {
    mix(vertex.x());
    mix(vertex.y());
  }
  return h;
}

}  // namespace autoware::safety_planner
