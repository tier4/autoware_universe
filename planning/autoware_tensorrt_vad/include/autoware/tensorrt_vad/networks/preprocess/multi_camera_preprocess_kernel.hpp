#ifndef AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_KERNEL_HPP_
#define AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_KERNEL_HPP_

#include <cuda_runtime.h>
#include <cstdint>

namespace autoware::tensorrt_vad {

/**
 * @struct MultiCameraPreprocessConfig
 * @brief Configuration parameters required for CUDA preprocessing
 */
struct MultiCameraPreprocessConfig {
    int32_t input_width;
    int32_t input_height;
    int32_t output_width;
    int32_t output_height;
    int32_t num_cameras;
    float scale_x;             // Scaling factor for x-axis (input_width / output_width)
    float scale_y;             // Scaling factor for y-axis (input_height / output_height)
    float mean[3];             // Mean values for normalization [R, G, B]
    float inverse_std[3];      // Inverse standard deviation values [R, G, B]
};

/**
 * @brief Launch resize kernel to resize input images to target dimensions
 * @param d_input_images Array of pointers to input images on device (BGR uint8_t format)
 * @param d_resized_images Array of pointers to output resized images on device (BGR uint8_t format)
 * @param config Preprocessing configuration parameters
 * @param stream CUDA stream for asynchronous execution
 * @return cudaError_t CUDA error code
 */
cudaError_t launch_multi_camera_resize_kernel(
    uint8_t** d_input_images,
    uint8_t** d_resized_images,
    const MultiCameraPreprocessConfig& config,
    cudaStream_t stream);

/**
 * @brief Launch normalization kernel to convert BGR->RGB, normalize, and format as CHW
 * @param d_resized_images Array of pointers to resized images on device (BGR uint8_t format)
 * @param d_output Final output buffer (RGB float CHW format)
 * @param config Preprocessing configuration parameters
 * @param stream CUDA stream for asynchronous execution
 * @return cudaError_t CUDA error code
 */
cudaError_t launch_multi_camera_normalize_kernel(
    uint8_t** d_resized_images,
    float* d_output,
    const MultiCameraPreprocessConfig& config,
    cudaStream_t stream);

}  // namespace autoware::tensorrt_vad

#endif // AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_KERNEL_HPP_
