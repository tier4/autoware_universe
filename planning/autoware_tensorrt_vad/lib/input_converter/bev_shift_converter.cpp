#include "autoware/tensorrt_vad/input_converter/bev_shift_converter.hpp"

namespace autoware::tensorrt_vad::vad_interface {

InputBEVShiftConverter::InputBEVShiftConverter(const CoordinateTransformer& transformer, const VadInterfaceConfig& config)
  : Converter(transformer, config)
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

  float real_w = config_.detection_range[3] - config_.detection_range[0];
  float real_h = config_.detection_range[4] - config_.detection_range[1];
  float grid_length[] = {real_h / config_.bev_h, real_w / config_.bev_w};

  float ego_angle = patch_angle_rad / M_PI * 180.0;
  float grid_length_y = grid_length[0];
  float grid_length_x = grid_length[1];

  float translation_length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
  float translation_angle = std::atan2(delta_y, delta_x) / M_PI * 180.0;
  float bev_angle = ego_angle - translation_angle;

  float shift_y = translation_length * std::cos(bev_angle / 180.0 * M_PI) /
                  grid_length_y / config_.bev_h;
  float shift_x = translation_length * std::sin(bev_angle / 180.0 * M_PI) /
                  grid_length_x / config_.bev_w;

  return {shift_x, shift_y};
}

} // namespace autoware::tensorrt_vad::vad_interface
