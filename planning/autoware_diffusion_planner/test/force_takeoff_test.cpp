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

#include "autoware/diffusion_planner/diffusion_planner_core.hpp"

#include <rclcpp/time.hpp>

#include <autoware_system_msgs/msg/autoware_state.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace autoware::diffusion_planner::test
{

class ForceTakeoffTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    DiffusionPlannerParams params{};
    params.batch_size = 1;
    params.turn_indicator_hold_duration = 0.0;
    params.turn_indicator_keep_offset = 0.0f;
    params.force_takeoff_enable = true;
    params.force_takeoff_synthetic_speed_mps = 1.5;
    params.force_takeoff_stationary_duration_s = 3.0;
    params.force_takeoff_release_speed_mps = 0.5;
    params.force_takeoff_min_agent_distance_m = 20.0;
    params_ = params;

    VehicleInfo vehicle_info{};
    core_ = std::make_unique<DiffusionPlannerCore>(params_, vehicle_info);
  }

  static std::shared_ptr<const AutowareState> make_state(const uint8_t state)
  {
    auto msg = std::make_shared<AutowareState>();
    msg->state = state;
    return msg;
  }

  static Odometry make_odometry(const double speed_mps)
  {
    Odometry odometry;
    odometry.twist.twist.linear.x = speed_mps;
    return odometry;
  }

  static TrackedObjects make_objects(const std::vector<double> & distances_m)
  {
    TrackedObjects objects;
    for (const double distance : distances_m) {
      autoware_perception_msgs::msg::TrackedObject object;
      object.kinematics.pose_with_covariance.pose.position.x = distance;
      objects.objects.push_back(object);
    }
    return objects;
  }

  rclcpp::Time at(const double seconds) const
  {
    return rclcpp::Time(0, 0, RCL_ROS_TIME) + rclcpp::Duration::from_seconds(seconds);
  }

  void tick(
    const double t, const double speed_mps, const uint8_t state,
    const std::vector<double> & agent_distances_m = {})
  {
    core_->update_force_takeoff_state(
      make_odometry(speed_mps), make_objects(agent_distances_m), make_state(state), at(t));
  }

  // Engage at t=0 (WAITING_FOR_ENGAGE -> DRIVING) with the ego stopped.
  void engage()
  {
    tick(0.0, 0.0, AutowareState::WAITING_FOR_ENGAGE);
    tick(0.1, 0.0, AutowareState::DRIVING);
  }

  DiffusionPlannerParams params_;
  std::unique_ptr<DiffusionPlannerCore> core_;
};

TEST_F(ForceTakeoffTest, ActivatesAfterStallAndReleasesOnSpeed)
{
  engage();
  tick(2.0, 0.0, AutowareState::DRIVING);
  EXPECT_FALSE(core_->is_force_takeoff_active()) << "must not activate before the stall duration";

  tick(3.5, 0.0, AutowareState::DRIVING);
  EXPECT_TRUE(core_->is_force_takeoff_active()) << "must activate after stalling > 3 s post engage";

  tick(4.0, 0.3, AutowareState::DRIVING);
  EXPECT_TRUE(core_->is_force_takeoff_active()) << "must stay active below the release speed";

  tick(4.5, 0.6, AutowareState::DRIVING);
  EXPECT_FALSE(core_->is_force_takeoff_active()) << "must release once the release speed is reached";
}

TEST_F(ForceTakeoffTest, DoesNotActivateWhenEgoMovesInTime)
{
  engage();
  tick(1.0, 0.5, AutowareState::DRIVING);  // ego starts moving within the stall window
  tick(5.0, 0.0, AutowareState::DRIVING);  // later stop without a new engage
  tick(20.0, 0.0, AutowareState::DRIVING);
  EXPECT_FALSE(core_->is_force_takeoff_active())
    << "a stop without a fresh engage transition must not trigger the override";
}

TEST_F(ForceTakeoffTest, DoesNotActivateWithAgentNearby)
{
  engage();
  tick(3.5, 0.0, AutowareState::DRIVING, {10.0});
  EXPECT_FALSE(core_->is_force_takeoff_active()) << "agent within 20 m must gate activation";

  tick(4.0, 0.0, AutowareState::DRIVING, {30.0});
  EXPECT_TRUE(core_->is_force_takeoff_active()) << "activation may proceed once the agent leaves";
}

TEST_F(ForceTakeoffTest, AbortsWhenAgentEntersWhileActive)
{
  engage();
  tick(3.5, 0.0, AutowareState::DRIVING);
  ASSERT_TRUE(core_->is_force_takeoff_active());

  tick(4.0, 0.0, AutowareState::DRIVING, {5.0});
  EXPECT_FALSE(core_->is_force_takeoff_active()) << "agent entering the radius must abort";
}

TEST_F(ForceTakeoffTest, AbortsWhenLeavingDriving)
{
  engage();
  tick(3.5, 0.0, AutowareState::DRIVING);
  ASSERT_TRUE(core_->is_force_takeoff_active());

  tick(4.0, 0.0, AutowareState::WAITING_FOR_ENGAGE);
  EXPECT_FALSE(core_->is_force_takeoff_active()) << "disengage must abort the active override";

  // Re-engaging arms a fresh timer instead of resuming the old override.
  tick(5.0, 0.0, AutowareState::DRIVING);
  EXPECT_FALSE(core_->is_force_takeoff_active());
  tick(8.5, 0.0, AutowareState::DRIVING);
  EXPECT_TRUE(core_->is_force_takeoff_active());
}

TEST_F(ForceTakeoffTest, DisableParameterIsImmediateKillSwitch)
{
  engage();
  tick(3.5, 0.0, AutowareState::DRIVING);
  ASSERT_TRUE(core_->is_force_takeoff_active());

  auto disabled_params = params_;
  disabled_params.force_takeoff_enable = false;
  core_->update_params(disabled_params);
  tick(4.0, 0.0, AutowareState::DRIVING);
  EXPECT_FALSE(core_->is_force_takeoff_active()) << "disabling must release the active override";

  // Re-enabling must not resurrect the old engage timer.
  core_->update_params(params_);
  tick(10.0, 0.0, AutowareState::DRIVING);
  EXPECT_FALSE(core_->is_force_takeoff_active())
    << "re-enabling without a fresh engage must stay inactive";
}

TEST_F(ForceTakeoffTest, DisableWhileArmedClearsTimer)
{
  engage();

  auto disabled_params = params_;
  disabled_params.force_takeoff_enable = false;
  core_->update_params(disabled_params);
  tick(1.0, 0.0, AutowareState::DRIVING);

  core_->update_params(params_);
  tick(5.0, 0.0, AutowareState::DRIVING);
  EXPECT_FALSE(core_->is_force_takeoff_active())
    << "an armed timer must be cleared by the kill switch, not resumed on re-enable";
}

TEST_F(ForceTakeoffTest, NoActivationWithoutEngageTransition)
{
  // First observed state is already DRIVING: no transition, so never activates.
  tick(0.0, 0.0, AutowareState::DRIVING);
  tick(10.0, 0.0, AutowareState::DRIVING);
  EXPECT_FALSE(core_->is_force_takeoff_active());
}

}  // namespace autoware::diffusion_planner::test
