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
#ifndef UTILS__BOUNDARY_SIMPLIFIER_HPP_
#define UTILS__BOUNDARY_SIMPLIFIER_HPP_

#include "../type_alias.hpp"

#include <autoware_utils_system/lru_cache.hpp>

#include <cstddef>
#include <cstdint>

namespace autoware::safety_planner
{

class BoundarySimplifier
{
public:
  BoundarySimplifier(double tolerance_m, std::size_t cache_size);

  //! tolerance_m <= 0 returns the input unchanged
  LineString2d simplify(const LineString2d & polyline);

private:
  static std::uint64_t hash(const LineString2d & polyline);

  double tolerance_m_;
  autoware_utils_system::LRUCache<std::uint64_t, LineString2d> cache_;
};

}  // namespace autoware::safety_planner

#endif  // UTILS__BOUNDARY_SIMPLIFIER_HPP_
