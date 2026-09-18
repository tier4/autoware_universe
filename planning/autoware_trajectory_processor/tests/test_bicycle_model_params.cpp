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

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <gtest/gtest.h>
#include <temporal_mpt/bicycle_model_params.hpp>

namespace
{
autoware::vehicle_info_utils::VehicleInfo makeJ6Gen2LikeVehicleInfo()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.3725, 0.215, 4.76012, 1.754, 0.95099, 1.52579, 0.32358, 0.34983, 3.080, 0.640);
}
}  // namespace

TEST(BicycleModelParamsTest, J6Gen2LfLrMatchesWheelBase)
{
  const auto vehicle = makeJ6Gen2LikeVehicleInfo();
  const auto params = temporal_mpt::bicycleLfLrFromVehicleInfo(vehicle);

  EXPECT_NEAR(params.lr, 2.092659950, 1.0e-6);
  EXPECT_NEAR(params.lf, 2.667460050, 1.0e-6);
  EXPECT_NEAR(params.lf + params.lr, vehicle.wheel_base_m, 1.0e-9);
}
