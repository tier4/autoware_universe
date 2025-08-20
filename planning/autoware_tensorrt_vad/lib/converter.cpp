#include "autoware/tensorrt_vad/converter.hpp"

namespace autoware::tensorrt_vad::vad_interface {

Converter::Converter(const CoordinateTransformer& transformer, const VadInterfaceConfig& config)
  : coordinate_transformer_(transformer), config_(config)
{
}

} // namespace autoware::tensorrt_vad::vad_interface
