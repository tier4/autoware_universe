#ifndef AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_HPP_
#define AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_HPP_

#include <vector>
#include <opencv2/core/mat.hpp>
#include <memory>
#include <string>
#include <cuda_runtime.h>
#include "autoware/tensorrt_vad/ros_vad_logger.hpp"
#include "autoware/tensorrt_vad/networks/preprocess/multi_camera_preprocess_kernel.hpp"

/**
 * @class MultiCameraPreprocessor
 * @brief GPU preprocessing pipeline for multiple camera images
 *
 * This class encapsulates the entire flow from receiving cv::Mat images from host,
 * copying to device, custom CUDA kernel-based resize/BGR2RGB convert/normalize/concatenate operations.
 * Resources are allocated in constructor (RAII) and released in destructor.
 */
class MultiCameraPreprocessor {
public:
    // Template constructor to accept shared_ptr<LoggerType>
    template<typename LoggerType>
    MultiCameraPreprocessor(const MultiCameraPreprocessConfig& config, std::shared_ptr<LoggerType> logger);
    
    ~MultiCameraPreprocessor();

    // Prohibit copy constructor and copy assignment operator to prevent double deallocation
    MultiCameraPreprocessor(const MultiCameraPreprocessor&) = delete;
    MultiCameraPreprocessor& operator=(const MultiCameraPreprocessor&) = delete;

  /**
   * @brief Batch preprocess multiple camera images (cv::Mat) on GPU.
   * @param camera_images Host-side input images (cv::Mat) vector. Should be BGR8 format.
   * @param d_output_buffer Pointer to device output buffer. Results are written here in CHW format.
   * @param stream CUDA stream to use for execution.
   * @return cudaError_t Execution status.
   */
  cudaError_t preprocess_images(
      const std::vector<cv::Mat>& camera_images,
      float* d_output_buffer,
      cudaStream_t stream
  );private:
    /**
     * @brief Validates input image vector (size, format, etc.).
     * @param camera_images Image vector to validate.
     * @return cudaError_t Validation result. cudaSuccess if successful.
     */
    cudaError_t validate_input(const std::vector<cv::Mat>& camera_images) const;

    MultiCameraPreprocessConfig config_;
    std::shared_ptr<autoware::tensorrt_vad::VadLogger> logger_;  // Direct VadLogger pointer

    // --- GPU Buffers ---
    // Input buffers (allocated in constructor)
    uint8_t* d_input_buffer_{nullptr};      // Single continuous buffer storing all raw input images
    uint8_t** d_input_image_ptrs_{nullptr}; // Pointer array pointing to start positions of each image within d_input_buffer_
};

// Template method implementations (must be in header for templates)
template<typename LoggerType>
MultiCameraPreprocessor::MultiCameraPreprocessor(const MultiCameraPreprocessConfig& config, std::shared_ptr<LoggerType> logger) 
    : config_(config), logger_(std::static_pointer_cast<autoware::tensorrt_vad::VadLogger>(logger)) {
    
    // Logger accepts only classes that inherit from VadLogger
    static_assert(std::is_base_of_v<autoware::tensorrt_vad::VadLogger, LoggerType>, 
        "LoggerType must be VadLogger or derive from VadLogger.");
    
    logger_->debug("MultiCameraPreprocessor config: input=" + 
                   std::to_string(config_.input_width) + "x" + std::to_string(config_.input_height) + 
                   ", output=" + std::to_string(config_.output_width) + "x" + std::to_string(config_.output_height) + 
                   ", cameras=" + std::to_string(config_.num_cameras));
    
    // --- Allocate Input Buffers ---
    const size_t single_input_size = static_cast<size_t>(config_.input_width) * config_.input_height * 3;
    const size_t total_input_size = single_input_size * config_.num_cameras;
    
    cudaError_t err = cudaMalloc(&d_input_buffer_, total_input_size);
    if (err != cudaSuccess) {
        logger_->error("Failed to allocate input buffer of size " + std::to_string(total_input_size) + ": " + cudaGetErrorString(err));
        return;
    }
    
    err = cudaMalloc(&d_input_image_ptrs_, config_.num_cameras * sizeof(uint8_t*));
    if (err != cudaSuccess) {
        logger_->error("Failed to allocate input image pointers: " + std::string(cudaGetErrorString(err)));
        cudaFree(d_input_buffer_);
        d_input_buffer_ = nullptr;
        return;
    }

    std::vector<uint8_t*> h_input_ptrs(config_.num_cameras);
    for (int32_t i = 0; i < config_.num_cameras; ++i) {
        h_input_ptrs[i] = d_input_buffer_ + i * single_input_size;
    }
    
    err = cudaMemcpy(d_input_image_ptrs_, h_input_ptrs.data(), config_.num_cameras * sizeof(uint8_t*), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        logger_->error("Failed to copy input image pointers to device: " + std::string(cudaGetErrorString(err)));
        cudaFree(d_input_buffer_);
        cudaFree(d_input_image_ptrs_);
        d_input_buffer_ = nullptr;
        d_input_image_ptrs_ = nullptr;
        return;
    }
    
    logger_->debug("MultiCameraPreprocessor initialized successfully");
}

#endif // AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_HPP_
