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

#ifndef AUTOWARE__SAFETY_PLANNER__TEST_OUTPUT_UTILS_HPP_
#define AUTOWARE__SAFETY_PLANNER__TEST_OUTPUT_UTILS_HPP_

// Output location for test artifacts (CSV dumps and figures).
// Everything goes under TEST_RESULTS_DIR (build tree), never into the source tree.

#include <string>

namespace autoware::safety_planner
{

//! Creates and returns "<TEST_RESULTS_DIR>/<sub_dir>/" (trailing slash included)
std::string make_test_results_dir(const std::string & sub_dir);

//! File-name stem derived from the running gtest name (snake_case, '/' replaced by '_')
std::string current_test_file_stem();

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__TEST_OUTPUT_UTILS_HPP_
