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

#include "test_output_utils.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>

namespace autoware::safety_planner
{

namespace
{
std::string to_snake_case(const std::string & str)
{
  std::string result;
  for (size_t i = 0; i < str.size(); ++i) {
    if (std::isupper(static_cast<unsigned char>(str[i]))) {
      if (i > 0 && std::islower(static_cast<unsigned char>(str[i - 1]))) {
        result += '_';
      }
      result += static_cast<char>(std::tolower(static_cast<unsigned char>(str[i])));
    } else {
      result += str[i];
    }
  }
  return result;
}
}  // namespace

std::string make_test_results_dir(const std::string & sub_dir)
{
  std::string path = std::string(TEST_RESULTS_DIR) + "/";
  if (!sub_dir.empty()) {
    path += sub_dir + "/";
  }
  std::filesystem::create_directories(path);
  return path;
}

std::string current_test_file_stem()
{
  const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info();
  if (!test_info) return "unknown_test";
  // Parameterized test names contain '/', which would become a directory separator
  std::string name = test_info->name();
  std::replace(name.begin(), name.end(), '/', '_');
  return to_snake_case(name);
}

}  // namespace autoware::safety_planner
