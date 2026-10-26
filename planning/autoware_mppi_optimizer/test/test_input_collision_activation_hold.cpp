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

#include "autoware/mppi_optimizer/input_collision_activation_hold.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace autoware::mppi_optimizer
{
namespace
{

TEST(InputCollisionActivationHold, ZeroDurationUsesOnlyTheCurrentCondition)
{
  InputCollisionActivationHold hold;
  EXPECT_EQ(hold.update(true, 10.0, 0.0), InputCollisionActivationSource::condition);
  EXPECT_EQ(hold.update(false, 10.0, 0.0), InputCollisionActivationSource::none);
}

TEST(InputCollisionActivationHold, EachMatchRefreshesTheHold)
{
  InputCollisionActivationHold hold;
  EXPECT_EQ(hold.update(true, 10.0, 2.0), InputCollisionActivationSource::condition);
  EXPECT_EQ(hold.update(false, 11.0, 2.0), InputCollisionActivationSource::hold);
  EXPECT_EQ(hold.update(true, 11.5, 2.0), InputCollisionActivationSource::condition);
  EXPECT_EQ(hold.update(false, 13.49, 2.0), InputCollisionActivationSource::hold);
  EXPECT_EQ(hold.update(false, 13.5, 2.0), InputCollisionActivationSource::none);
}

TEST(InputCollisionActivationHold, ResetAndBackwardClockJumpClearTheHold)
{
  InputCollisionActivationHold hold;
  EXPECT_EQ(hold.update(true, 10.0, 2.0), InputCollisionActivationSource::condition);
  hold.reset();
  EXPECT_EQ(hold.update(false, 11.0, 2.0), InputCollisionActivationSource::none);

  EXPECT_EQ(hold.update(true, 20.0, 5.0), InputCollisionActivationSource::condition);
  EXPECT_EQ(hold.update(false, 22.0, 5.0), InputCollisionActivationSource::hold);
  EXPECT_EQ(hold.update(false, 21.0, 5.0), InputCollisionActivationSource::none);
  EXPECT_EQ(hold.update(false, 22.0, 5.0), InputCollisionActivationSource::none);
}

TEST(InputCollisionActivationHold, RejectsInvalidDuration)
{
  InputCollisionActivationHold hold;
  EXPECT_THROW(static_cast<void>(hold.update(true, 10.0, -1.0)), std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(hold.update(true, 10.0, std::numeric_limits<double>::quiet_NaN())),
    std::invalid_argument);
  EXPECT_EQ(hold.update(false, 11.0, 2.0), InputCollisionActivationSource::none);
}

}  // namespace
}  // namespace autoware::mppi_optimizer
