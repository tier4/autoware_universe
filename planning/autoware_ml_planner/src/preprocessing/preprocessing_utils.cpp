// Copyright 2025 TIER IV, Inc.
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

#include "autoware/ml_planner/preprocessing/preprocessing_utils.hpp"

#include "autoware/ml_planner/dimensions.hpp"

namespace autoware::ml_planner::preprocess
{
namespace
{
void scale_pose_xy(xt::xarray<float> & values)
{
  const size_t feature_dim = values.shape().back();
  const size_t rows = values.size() / feature_dim;
  for (size_t row = 0; row < rows; ++row) {
    values[row * feature_dim] /= POSITION_SCALE;
    values[row * feature_dim + 1] /= POSITION_SCALE;
  }
}

void divide_all(xt::xarray<float> & values, const float scale)
{
  for (float & value : values) {
    value /= scale;
  }
}
}  // namespace

void normalize_input_data(TensorMap & input_data_map)
{
  scale_pose_xy(input_data_map.at("ego_agent_past"));
  scale_pose_xy(input_data_map.at("neighbor_agents_past"));
  scale_pose_xy(input_data_map.at("goal_pose"));

  for (const char * key :
       {"lanes", "route_lanes", "intersection_area", "stop_lines", "road_borders"}) {
    divide_all(input_data_map.at(key), POSITION_SCALE);
  }
  for (const char * key : {"lanes_speed_limit", "route_lanes_speed_limit"}) {
    divide_all(input_data_map.at(key), SPEED_SCALE);
  }
  for (const char * key : {"agent_shape", "ego_shape"}) {
    divide_all(input_data_map.at(key), VEHICLE_SHAPE_SCALE);
  }
}

}  // namespace autoware::ml_planner::preprocess
