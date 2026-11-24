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

#include "autoware/tensorrt_e2e/types.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

TEST(TypesTest, ShapeNumElements)
{
  EXPECT_EQ(shape_num_elements({}), 0);
  EXPECT_EQ(shape_num_elements({1}), 1);
  EXPECT_EQ(shape_num_elements({1, 40, 4}), 160);
  EXPECT_EQ(shape_num_elements({2, 5, 3, 8, 16}), 3840);
}

TEST(TypesTest, ShapeToString)
{
  EXPECT_EQ(shape_to_string({}), "[]");
  EXPECT_EQ(shape_to_string({1, 40, 4}), "[1, 40, 4]");
}

TEST(TypesTest, TensorFactories)
{
  const auto host = Tensor::from_host({1, 2, 3}, std::vector<float>(6, 1.0f));
  EXPECT_FALSE(host.is_device());
  EXPECT_EQ(host.num_elements(), 6);
  EXPECT_EQ(host.host_data.size(), 6U);

  const float dummy = 0.0f;
  const auto device = Tensor::from_device({1, 4}, &dummy);
  EXPECT_TRUE(device.is_device());
  EXPECT_EQ(device.num_elements(), 4);
  EXPECT_TRUE(device.host_data.empty());
}

TEST(TypesTest, FindSpec)
{
  const std::vector<TensorSpec> specs = {
    {"prediction", {1, 1, 40, 4}, TensorDataType::kFLOAT32},
    {"lanes_has_speed_limit", {1, 140, 1}, TensorDataType::kBOOL},
  };
  ASSERT_NE(find_spec(specs, "prediction"), nullptr);
  EXPECT_EQ(find_spec(specs, "prediction")->shape, (std::vector<int64_t>{1, 1, 40, 4}));
  EXPECT_EQ(find_spec(specs, "lanes_has_speed_limit")->dtype, TensorDataType::kBOOL);
  EXPECT_EQ(find_spec(specs, "missing"), nullptr);
}

}  // namespace autoware::tensorrt_e2e
