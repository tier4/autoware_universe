#include "autoware/tensorrt_vad/input_converter/bev_shift_converter.hpp"

namespace autoware::tensorrt_vad::vad_interface {

InputBEVShiftConverter::InputBEVShiftConverter(const CoordinateTransformer& transformer, const VadInterfaceConfig& config)
  : Converter(transformer, config),
    real_w_(config.detection_range[3] - config.detection_range[0]),
    real_h_(config.detection_range[4] - config.detection_range[1])
{
}

ShiftData InputBEVShiftConverter::process_shift(
  const CanBusData& can_bus,
  const CanBusData& prev_can_bus) const
{
  if (prev_can_bus.empty()) {
    return config_.default_shift;
  }

  float delta_x = can_bus[0] - prev_can_bus[0];  // translation difference
  float delta_y = can_bus[1] - prev_can_bus[1];  // translation difference
  float patch_angle_rad = can_bus[16];  // current patch_angle[rad]

  float ego_angle = patch_angle_rad / M_PI * 180.0;

  float translation_length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
  float translation_angle = std::atan2(delta_y, delta_x) / M_PI * 180.0;
  float bev_angle = ego_angle - translation_angle;

  float shift_y = translation_length * std::cos(bev_angle / 180.0 * M_PI) / real_h_;
  float shift_x = translation_length * std::sin(bev_angle / 180.0 * M_PI) / real_w_;

  return {shift_x, shift_y};
}

} // namespace autoware::tensorrt_vad::vad_interface
