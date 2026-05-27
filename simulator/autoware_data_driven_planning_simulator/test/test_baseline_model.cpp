// Copyright 2026 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autoware/data_driven_planning_simulator/models/baseline_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::simulator::data_driven_planning_simulator
{

TEST(BaselineModel, IntegratesStraightDifferentialMotion)
{
  BaselineModelParams params;
  params.type = BaselineModelType::Differential;
  BaselineModel model(params);
  VehicleState state;
  VehicleCommand command;
  command.vx = 2.0;
  command.wz = 0.0;
  model.reset(state);
  model.set_command(command);
  model.update(1.0);
  const auto result = model.state();
  EXPECT_NEAR(result.x, 2.0, 1.0e-6);
  EXPECT_NEAR(result.y, 0.0, 1.0e-6);
  EXPECT_NEAR(result.yaw, 0.0, 1.0e-6);
}

TEST(BaselineModel, IntegratesConstantTwistArc)
{
  BaselineModelParams params;
  params.type = BaselineModelType::Differential;
  BaselineModel model(params);
  VehicleState state;
  VehicleCommand command;
  command.vx = 1.0;
  command.wz = 1.0;
  model.reset(state);
  model.set_command(command);
  model.update(1.0);
  const auto result = model.state();
  EXPECT_NEAR(result.x, std::sin(1.0), 1.0e-6);
  EXPECT_NEAR(result.y, 1.0 - std::cos(1.0), 1.0e-6);
  EXPECT_NEAR(result.yaw, 1.0, 1.0e-6);
}

TEST(BaselineModel, ParsesAliases)
{
  EXPECT_EQ(parse_baseline_model_type("ackermann"), BaselineModelType::Ackermann);
  EXPECT_EQ(parse_baseline_model_type("skid_steer"), BaselineModelType::Differential);
  EXPECT_EQ(parse_baseline_model_type("mecanum"), BaselineModelType::Holonomic);
}

}  // namespace autoware::simulator::data_driven_planning_simulator
