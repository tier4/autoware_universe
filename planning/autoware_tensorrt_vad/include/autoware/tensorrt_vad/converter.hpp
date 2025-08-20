#ifndef AUTOWARE_TENSORRT_VAD_CONVERTER_HPP_
#define AUTOWARE_TENSORRT_VAD_CONVERTER_HPP_

#include "coordinate_transformer.hpp"
#include "vad_interface_config.hpp"

namespace autoware::tensorrt_vad::vad_interface {

/**
 * @brief Base class for all data converters
 * 
 * This abstract class provides common functionality for all converters:
 * - Access to coordinate transformation capabilities
 * - Access to configuration parameters
 * - Common interface for derived converter classes
 */
class Converter {
public:
  /**
   * @brief Constructor
   * @param transformer Reference to coordinate transformer for coordinate conversions
   * @param config Reference to configuration containing all necessary parameters
   */
  Converter(const CoordinateTransformer& transformer, const VadInterfaceConfig& config);
  
  /**
   * @brief Virtual destructor to enable proper cleanup of derived classes
   */
  virtual ~Converter() = default;

protected:
  const CoordinateTransformer& transformer_;  ///< Reference to coordinate transformer
  const VadInterfaceConfig& config_;          ///< Reference to configuration parameters
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE_TENSORRT_VAD_CONVERTER_HPP_
