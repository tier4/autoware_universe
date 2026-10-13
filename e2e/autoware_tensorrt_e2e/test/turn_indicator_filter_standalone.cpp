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

#include "autoware/tensorrt_e2e/turn_indicator_filter.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <limits>
#include <vector>

int main()
{
  using autoware::tensorrt_e2e::TurnIndicatorFilter;
  const std::vector<float> off{4.f, 0.f, 0.f}, left{0.f, 4.f, 0.f}, right{0.f, 0.f, 4.f};
  const std::vector<float> unsure{0.f, 0.5f, 0.f};  // LEFT leads, but under 0.6

  // Starts DISABLE; a confident class must persist for confirm_cycles before it takes over.
  TurnIndicatorFilter filter({0.6, 3, 1.0});
  assert(filter.command() == 1);
  assert(filter.update(left, 10.0) == 1);
  assert(filter.update(left, 10.1) == 1);
  assert(filter.update(left, 10.2) == 2);

  // One dissenting cycle resets the count, so a flicker never switches.
  assert(filter.update(off, 10.3) == 2);
  assert(filter.update(left, 10.4) == 2);
  assert(filter.update(off, 10.5) == 2);
  assert(filter.update(off, 10.6) == 2);

  // Three confirming cycles are not enough until hold_s has passed since the last change.
  assert(filter.update(off, 10.7) == 2);
  assert(filter.update(off, 11.0) == 2);
  assert(filter.update(off, 11.3) == 1);

  // A leading class below min_probability counts for nothing.
  for (int i = 0; i < 10; ++i) assert(filter.update(unsure, 20.0 + i) == 1);

  // LEFT to RIGHT directly, after the hold.
  for (int i = 0; i < 3; ++i) filter.update(left, 30.0 + 0.1 * i);
  assert(filter.command() == 2);
  for (int i = 0; i < 3; ++i) filter.update(right, 40.0 + 0.1 * i);
  assert(filter.command() == 3);

  // Malformed or non-finite output leaves the command alone.
  assert(filter.update({0.f, 1.f}, 50.0) == 3);
  assert(filter.update({std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f}, 50.1) == 3);
  return 0;
}
