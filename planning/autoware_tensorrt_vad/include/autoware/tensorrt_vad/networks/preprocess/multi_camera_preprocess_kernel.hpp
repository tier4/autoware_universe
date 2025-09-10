#ifndef AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_KERNEL_HPP_
#define AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_KERNEL_HPP_

#include <cuda_runtime.h>
#include <cstdint>

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
    float scale_x;
    float scale_y;
    float mean[3];
    float inverse_std[3];
};

// Declaration of host-side wrapper function to launch CUDA kernel
cudaError_t launch_multi_camera_preprocess_kernel(
    uint8_t** d_input_images,
    float* d_output,
    const MultiCameraPreprocessConfig& config,
    cudaStream_t stream
);

#endif // AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_KERNEL_HPP_
