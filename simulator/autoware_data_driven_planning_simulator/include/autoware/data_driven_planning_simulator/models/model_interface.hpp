// Copyright 2026 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTOWARE__DATA_DRIVEN_PLANNING_SIMULATOR__MODELS__MODEL_INTERFACE_HPP_
#define AUTOWARE__DATA_DRIVEN_PLANNING_SIMULATOR__MODELS__MODEL_INTERFACE_HPP_

#include <string>

namespace autoware::simulator::data_driven_planning_simulator
{

struct VehicleState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  double steer{0.0};
  double ax{0.0};
};

struct VehicleCommand
{
  double velocity{0.0};
  double acceleration{0.0};
  double steer{0.0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  double left_wheel_velocity{0.0};
  double right_wheel_velocity{0.0};
};

class ModelInterface
{
public:
  virtual ~ModelInterface() = default;
  virtual void reset(const VehicleState & state) = 0;
  virtual void set_command(const VehicleCommand & command) = 0;
  virtual void update(double dt) = 0;
  virtual VehicleState state() const = 0;
  virtual std::string name() const = 0;
};

}  // namespace autoware::simulator::data_driven_planning_simulator

#endif  // AUTOWARE__DATA_DRIVEN_PLANNING_SIMULATOR__MODELS__MODEL_INTERFACE_HPP_
