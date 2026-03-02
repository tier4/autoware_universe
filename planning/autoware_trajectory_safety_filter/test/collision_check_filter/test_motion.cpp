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

// #include "autoware/trajectory_safety_filter/filters/collision_check_filter.hpp"
#include "../../src/filters/collision_check_filter.cpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

// todo: テストケースの中身はLLMそのままで未確認

namespace autoware::trajectory_safety_filter::plugin::motion
{
class ComputeMotionProfile1dTest : public ::testing::Test
{
protected:
  // Twistメッセージを手軽に生成するためのヘルパー関数
  geometry_msgs::msg::Twist create_twist(double vx, double vy)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx;
    twist.linear.y = vy;
    return twist;
  }
};

// 1. 初期速度が0以下のケース
TEST_F(ComputeMotionProfile1dTest, ZeroInitialVelocity)
{
  auto twist = create_twist(0.0, 0.0);
  auto profile = compute_motion_profile_1d(twist, 1.0, 1.0, 5.0);

  ASSERT_EQ(profile.times.size(), 1u);
  ASSERT_EQ(profile.distances.size(), 1u);
  EXPECT_DOUBLE_EQ(profile.times[0], 0.0);
  EXPECT_DOUBLE_EQ(profile.distances[0], 0.0);
}

// 2. 等速運動のケース (加速度 = 0)
TEST_F(ComputeMotionProfile1dTest, ConstantVelocity)
{
  auto twist = create_twist(10.0, 0.0);  // 合成初速: 10.0
  double lag = 1.0;
  double accel = 0.0;
  // 浮動小数点のループ判定(t < max_time)を安定させるため、端数を持たせています
  double max_time = 3.05;

  auto profile = compute_motion_profile_1d(twist, lag, accel, max_time);

  // max_timeは含まれないため t = 0.0, 0.1, ..., 3.0
  size_t expected_size = static_cast<size_t>(max_time / TIME_RESOLUTION) + 1;
  ASSERT_EQ(profile.times.size(), expected_size);
  ASSERT_EQ(profile.distances.size(), expected_size);

  for (size_t i = 0; i < expected_size; ++i) {
    double t = profile.times[i];
    EXPECT_NEAR(t, i * TIME_RESOLUTION, 1e-6);
    // 加速度0なので、遅延終了後も常に d = v * t となる
    EXPECT_NEAR(profile.distances[i], 10.0 * t, 1e-6);
  }
}

// 3. 加速運動のケース (遅延時間あり)
TEST_F(ComputeMotionProfile1dTest, Acceleration)
{
  auto twist = create_twist(3.0, 4.0);  // 合成初速: hypot(3,4) = 5.0
  double lag = 1.0;
  double accel = 2.0;
  double max_time = 2.05;

  auto profile = compute_motion_profile_1d(twist, lag, accel, max_time);

  ASSERT_FALSE(profile.times.empty());

  for (size_t i = 0; i < profile.times.size(); ++i) {
    double t = profile.times[i];
    if (t < lag) {
      EXPECT_NEAR(profile.distances[i], 5.0 * t, 1e-6);
    } else {
      double time_after_lag = t - lag;
      double expected_d =
        (5.0 * lag) + (5.0 * time_after_lag) + (0.5 * accel * time_after_lag * time_after_lag);
      EXPECT_NEAR(profile.distances[i], expected_d, 1e-6);
    }
  }
}

// 4. 減速して停止するケース
TEST_F(ComputeMotionProfile1dTest, DecelerationAndStop)
{
  auto twist = create_twist(10.0, 0.0);  // 合成初速: 10.0
  double lag = 1.0;
  double accel = -5.0;
  double max_time = 5.0;  // t=5.0まで計算しようとするが、途中で停止するはず

  auto profile = compute_motion_profile_1d(twist, lag, accel, max_time);

  // 初速10、加速度-5なので、遅延(1秒)の後、2秒間で停止する。つまり t = 3.0 で停止判定に入る。
  // プロファイルは t = 0.0 から 3.0 までの要素を持つ。
  double expected_stop_time = 3.0;
  size_t expected_size = static_cast<size_t>(expected_stop_time / TIME_RESOLUTION) + 1;

  ASSERT_EQ(profile.times.size(), expected_size);

  // 計算上の停止距離の検証
  double lag_distance = 10.0 * 1.0;  // 10.0
  double time_to_stop = 10.0 / 5.0;  // 2.0
  double expected_stop_distance =
    lag_distance + (10.0 * time_to_stop) + (0.5 * -5.0 * time_to_stop * time_to_stop);
  // 10.0 + 20.0 - 10.0 = 20.0

  EXPECT_NEAR(profile.times.back(), expected_stop_time, 1e-6);
  EXPECT_NEAR(profile.distances.back(), expected_stop_distance, 1e-6);
}

// 5. max_timeより遅延(lag)が長いケース
TEST_F(ComputeMotionProfile1dTest, LagLongerThanMaxTime)
{
  auto twist = create_twist(5.0, 0.0);  // 合成初速: 5.0
  double lag = 5.0;
  double accel = -10.0;
  double max_time = 2.05;

  auto profile = compute_motion_profile_1d(twist, lag, accel, max_time);

  // max_timeに到達するまでずっと遅延状態（等速）として計算されるか
  for (size_t i = 0; i < profile.times.size(); ++i) {
    double t = profile.times[i];
    EXPECT_NEAR(profile.distances[i], 5.0 * t, 1e-6);
  }
}

}  // namespace autoware::trajectory_safety_filter::plugin