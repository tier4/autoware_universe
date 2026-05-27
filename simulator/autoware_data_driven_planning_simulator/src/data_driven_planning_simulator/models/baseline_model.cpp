// Copyright 2026 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "autoware/data_driven_planning_simulator/models/baseline_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace autoware::simulator::data_driven_planning_simulator
{
namespace
{
double normalize_yaw(double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}
}  // namespace

BaselineModelType parse_baseline_model_type(const std::string & value)
{
  if (value == "ackermann") {
    return BaselineModelType::Ackermann;
  }
  if (value == "differential" || value == "skid_steer") {
    return BaselineModelType::Differential;
  }
  if (value == "holonomic" || value == "omni" || value == "mecanum") {
    return BaselineModelType::Holonomic;
  }
  throw std::invalid_argument("unknown baseline_model_type: " + value);
}

BaselineModel::BaselineModel(const BaselineModelParams & params) : params_(params) {}

void BaselineModel::reset(const VehicleState & state)
{
  state_ = state;
}

void BaselineModel::set_command(const VehicleCommand & command)
{
  command_ = command;
}

void BaselineModel::update(const double dt)
{
  if (dt <= 0.0) {
    return;
  }
  switch (params_.type) {
    case BaselineModelType::Ackermann:
      update_ackermann(dt);
      break;
    case BaselineModelType::Differential:
      update_differential(dt);
      break;
    case BaselineModelType::Holonomic:
      update_holonomic(dt);
      break;
  }
}

VehicleState BaselineModel::state() const
{
  return state_;
}

std::string BaselineModel::name() const
{
  switch (params_.type) {
    case BaselineModelType::Ackermann:
      return "ackermann";
    case BaselineModelType::Differential:
      return "differential";
    case BaselineModelType::Holonomic:
      return "holonomic";
  }
  return "unknown";
}

void BaselineModel::update_ackermann(const double dt)
{
  const double prev_vx = state_.vx;
  state_.vx += command_.acceleration * dt;
  if (std::abs(command_.velocity) > 1.0e-6) {
    state_.vx = command_.velocity;
  }

  if (params_.steer_time_constant > 1.0e-6) {
    state_.steer += (command_.steer - state_.steer) / params_.steer_time_constant * dt;
  } else {
    state_.steer = command_.steer;
  }

  const double wz = state_.vx * std::tan(state_.steer) / std::max(params_.wheelbase, 1.0e-6);
  state_.wz = wz;
  state_.ax = (state_.vx - prev_vx) / dt;
  integrate_constant_body_twist(state_.vx, 0.0, wz, dt);
}

void BaselineModel::update_differential(const double dt)
{
  if (params_.use_wheel_command) {
    state_.vx =
      0.5 * params_.wheel_radius * (command_.right_wheel_velocity + command_.left_wheel_velocity);
    state_.wz = params_.wheel_radius *
                (command_.right_wheel_velocity - command_.left_wheel_velocity) /
                std::max(params_.track_width, 1.0e-6);
  } else {
    state_.vx = command_.vx;
    state_.wz = command_.wz;
  }
  state_.vy = 0.0;
  state_.ax = 0.0;
  integrate_constant_body_twist(state_.vx, 0.0, state_.wz, dt);
}

void BaselineModel::update_holonomic(const double dt)
{
  state_.vx = command_.vx;
  state_.vy = command_.vy;
  state_.wz = command_.wz;
  state_.ax = 0.0;
  integrate_constant_body_twist(state_.vx, state_.vy, state_.wz, dt);
}

void BaselineModel::integrate_constant_body_twist(
  const double vx, const double vy, const double wz, const double dt)
{
  const double yaw0 = state_.yaw;
  if (std::abs(wz) < 1.0e-8) {
    const double c = std::cos(yaw0);
    const double s = std::sin(yaw0);
    state_.x += (c * vx - s * vy) * dt;
    state_.y += (s * vx + c * vy) * dt;
  } else {
    const double yaw1 = yaw0 + wz * dt;
    state_.x += vx / wz * (std::sin(yaw1) - std::sin(yaw0)) +
                vy / wz * (std::cos(yaw1) - std::cos(yaw0));
    state_.y += vx / wz * (-std::cos(yaw1) + std::cos(yaw0)) +
                vy / wz * (std::sin(yaw1) - std::sin(yaw0));
  }
  state_.yaw = normalize_yaw(yaw0 + wz * dt);
}

}  // namespace autoware::simulator::data_driven_planning_simulator
