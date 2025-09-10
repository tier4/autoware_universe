#ifndef AUTOWARE_TENSORRT_VAD_CUDA_UTILS_HPP_
#define AUTOWARE_TENSORRT_VAD_CUDA_UTILS_HPP_

#include <cuda_runtime.h>
#include <cmath>
#include <cfloat>

namespace autoware::tensorrt_vad::cuda_utils {
    // Mathematical constants for numerical stability
    namespace limits {
        constexpr float SIGMOID_MAX_ARG = 88.0f;  // Conservative threshold for sigmoid stability
        constexpr float EXP_MAX_ARG = 88.72f;     // logf(FLT_MAX) ≈ 88.72f
    }
    
    /**
     * @brief Safe sigmoid function with overflow/underflow protection
     * 
     * CUDA's expf() handles extreme values by returning +∞ or 0, but doesn't 
     * clamp to finite values like FLT_MAX. This can cause issues in sigmoid calculation.
     * 
     * @param x Input value
     * @return Sigmoid value clamped to [0, 1] range with finite precision
     */
    __device__ inline float sigmoid_cuda(float x) {
        if (x > limits::SIGMOID_MAX_ARG) {
            return 1.0f;  // Avoid expf(-x) = 0 leading to 1.0f/(1.0f+0) = 1.0f
        } else if (x < -limits::SIGMOID_MAX_ARG) {
            return 0.0f;  // Avoid expf(-x) = +∞ leading to 1.0f/(1.0f+∞) = 0
        } else {
            return 1.0f / (1.0f + expf(-x));  // Safe range for expf()
        }
    }
    
    /**
     * @brief Safe exp function with finite value clamping
     * 
     * Unlike CUDA's expf() which returns +∞ for large inputs, this function
     * clamps to finite maximum values for downstream calculation safety.
     * 
     * @param x Input value
     * @return Exponential value clamped to finite range
     */
    __device__ inline float exp_cuda(float x) {
        if (x > limits::EXP_MAX_ARG) {
            return FLT_MAX;  // Return finite maximum instead of +∞
        } else if (x < -limits::EXP_MAX_ARG) {
            return 0.0f;     // Consistent with expf() underflow behavior
        } else {
            return expf(x);  // Safe range for standard expf()
        }
    }
} // namespace autoware::tensorrt_vad::cuda_utils

#endif  // AUTOWARE_TENSORRT_VAD_CUDA_UTILS_HPP_
