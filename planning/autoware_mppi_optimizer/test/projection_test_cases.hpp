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
#pragma once

#include <vector>

namespace autoware::mppi_optimizer::projection_test
{
struct Case
{
  const char * name;
  std::vector<float> x;
  std::vector<float> y;
  float query_x;
  float query_y;
  int segment;
  float t_raw;
  float lateral;
};

// Hand-computed answers, independent of the production search and its hint behavior.
inline std::vector<Case> cases()
{
  std::vector<Case> result{
    {"U bend", {0, 10, 10, 0}, {0, 0, 10, 10}, 1, 9, 2, 0.9F, 1},
    {"parallel hairpin", {0, 10, 10, 0}, {0, 0, 0.25F, 0.25F}, 1, 0.2F, 2, 0.9F, 0.05F},
    {"crossing tie", {-2, 2, -2, 2}, {-2, 2, 2, -2}, 0, 0, 0, 0.5F, 0},
    {"shared vertex tie", {0, 10, 10}, {0, 0, 10}, 10, 0, 0, 1, 0},
    {"parallel tie", {0, 10, 10, 0}, {0, 0, 2, 2}, 1, 1, 0, 0.1F, 1},
    {"duplicate vertex", {0, 0, 10, 10, 0}, {0, 0, 0, 10, 10}, 1, 9, 3, 0.9F, 1},
    {"before start", {0, 10}, {0, 0}, -2, 3, 0, -0.2F, 3},
    {"after end", {0, 10}, {0, 0}, 12, -3, 0, 1.2F, -3},
    {"nonuniform hairpin",
     {0, 0.01F, 0.02F, 10, 10, 9.99F, 1, 0},
     {0, 0, 0, 0, 1, 1, 1, 1},
     0.5F,
     0.9F,
     6,
     0.5F,
     0.1F},
    {"far outside grid", {0, 10, 10, 0}, {0, 0, 10, 10}, -1000, 9, 2, 101, 1}};
  Case dense{"256 point path", {}, {}, 200.5F, 0.25F, 200, 0.5F, 0.25F};
  for (int i = 0; i < 256; ++i) {
    dense.x.push_back(static_cast<float>(i));
    dense.y.push_back(0.0F);
  }
  result.push_back(dense);
  return result;
}
}  // namespace autoware::mppi_optimizer::projection_test
