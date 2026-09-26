// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
namespace autoware::tensorrt_e2e {
// e2e-data-producer make_derived VEHICLE_DIMENSIONS for the vehicles the
// derived-v10 training data was recorded on: jpntaxi (prd_jt) and j6 (x2_dev,
// 71 % of the training scenes). The model learned ego_shape from this table,
// not from vehicle_info, so it is fed the entry of the vehicle it runs on.
struct TrainingEgoShape {
  const char *vehicle;
  double wheel_base;
  double length;
  double width;
};
inline constexpr std::array<TrainingEgoShape, 2> TRAINING_EGO_SHAPES{{
    {"jpntaxi", 2.75, 4.34, 1.84},
    {"j6", 4.76012, 7.2369, 2.42741},
}};
// The two wheel bases are 2 m apart; anything farther than this from both is a
// vehicle the model never trained on.
inline constexpr double TRAINING_WHEEL_BASE_TOLERANCE_M = 0.5;
inline const TrainingEgoShape &training_ego_shape_for(double wheel_base_m) {
  const TrainingEgoShape *best = &TRAINING_EGO_SHAPES[0];
  for (const auto &shape : TRAINING_EGO_SHAPES) {
    if (std::abs(shape.wheel_base - wheel_base_m) <
        std::abs(best->wheel_base - wheel_base_m)) {
      best = &shape;
    }
  }
  if (!(std::abs(best->wheel_base - wheel_base_m) <=
        TRAINING_WHEEL_BASE_TOLERANCE_M)) {
    throw std::runtime_error(
        "vehicle_info wheel base " + std::to_string(wheel_base_m) +
        " m matches no vehicle the model was trained on (jpntaxi 2.75 m, j6 "
        "4.76 m)");
  }
  return *best;
}
} // namespace autoware::tensorrt_e2e
