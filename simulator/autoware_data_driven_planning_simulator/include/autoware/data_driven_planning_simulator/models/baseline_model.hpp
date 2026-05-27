// Copyright 2026 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AUTOWARE__DATA_DRIVEN_PLANNING_SIMULATOR__MODELS__BASELINE_MODEL_HPP_
#define AUTOWARE__DATA_DRIVEN_PLANNING_SIMULATOR__MODELS__BASELINE_MODEL_HPP_

#include "autoware/data_driven_planning_simulator/models/model_interface.hpp"

#include <string>

namespace autoware::simulator::data_driven_planning_simulator
{

enum class BaselineModelType { Ackermann, Differential, Holonomic };

struct BaselineModelParams
{
  BaselineModelType type{BaselineModelType::Ackermann};
  double wheelbase{2.7};
  double track_width{1.5};
  double wheel_radius{0.3};
  double steer_time_constant{0.0};
  bool use_wheel_command{false};
};

BaselineModelType parse_baseline_model_type(const std::string & value);

class BaselineModel : public ModelInterface
{
public:
  explicit BaselineModel(const BaselineModelParams & params);

  void reset(const VehicleState & state) override;
  void set_command(const VehicleCommand & command) override;
  void update(double dt) override;
  VehicleState state() const override;
  std::string name() const override;

private:
  void update_ackermann(double dt);
  void update_differential(double dt);
  void update_holonomic(double dt);
  void integrate_constant_body_twist(double vx, double vy, double wz, double dt);

  BaselineModelParams params_{};
  VehicleState state_{};
  VehicleCommand command_{};
};

}  // namespace autoware::simulator::data_driven_planning_simulator

#endif  // AUTOWARE__DATA_DRIVEN_PLANNING_SIMULATOR__MODELS__BASELINE_MODEL_HPP_
