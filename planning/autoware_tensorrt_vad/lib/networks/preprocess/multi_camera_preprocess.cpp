#include "autoware/tensorrt_vad/networks/preprocess/multi_camera_preprocess.hpp"
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

MultiCameraPreprocessor::MultiCameraPreprocessor(const MultiCameraPreprocessConfig& config) : config_(config) {
    // --- Allocate Input Buffers ---
    const size_t single_input_size = static_cast<size_t>(config_.input_width) * config_.input_height * 3;
    const size_t total_input_size = single_input_size * config_.num_cameras;
    cudaMalloc(&d_input_buffer_, total_input_size);
    cudaMalloc(&d_input_image_ptrs_, config_.num_cameras * sizeof(uint8_t*));

    std::vector<uint8_t*> h_input_ptrs(config_.num_cameras);
    for (int32_t i = 0; i < config_.num_cameras; ++i) {
        h_input_ptrs[i] = d_input_buffer_ + i * single_input_size;
    }
    cudaMemcpy(d_input_image_ptrs_, h_input_ptrs.data(), config_.num_cameras * sizeof(uint8_t*), cudaMemcpyHostToDevice);
}

MultiCameraPreprocessor::~MultiCameraPreprocessor() {
    cudaFree(d_input_buffer_);
    cudaFree(d_input_image_ptrs_);
}

cudaError_t MultiCameraPreprocessor::validate_input(const std::vector<cv::Mat>& camera_images) const
{
    // Check number of cameras
    if (camera_images.size() != static_cast<size_t>(config_.num_cameras)) {
        // Debug info: expected vs actual number of cameras
        printf("Camera count mismatch: expected %d, got %zu\n", config_.num_cameras, camera_images.size());
        return cudaErrorInvalidValue;
    }

    const size_t expected_input_size = static_cast<size_t>(config_.input_width) * config_.input_height * 3;
    
    for (size_t i = 0; i < camera_images.size(); ++i) {
        const auto& img = camera_images[i];
        
        // Check if image is empty
        if (img.empty()) {
            printf("Camera %zu: image is empty\n", i);
            return cudaErrorInvalidValue;
        }
        
        // Check if image memory is continuous
        if (!img.isContinuous()) {
            printf("Camera %zu: image memory is not continuous\n", i);
            return cudaErrorInvalidValue;
        }
        
        // Check image size and format
        const size_t actual_size = img.total() * img.elemSize();
        if (actual_size != expected_input_size) {
            printf("Camera %zu: size mismatch - expected %zu, got %zu (dims: %dx%dx%d, elemSize: %zu)\n", 
                   i, expected_input_size, actual_size, img.cols, img.rows, img.channels(), img.elemSize());
            return cudaErrorInvalidValue;
        }
        
        // Check image format (should be BGR, 3 channels, 8UC3)
        if (img.channels() != 3 || img.depth() != CV_8U) {
            printf("Camera %zu: invalid format - channels: %d, depth: %d (expected: 3 channels, CV_8U)\n", 
                   i, img.channels(), img.depth());
            return cudaErrorInvalidValue;
        }
    }

    return cudaSuccess;
}

cudaError_t MultiCameraPreprocessor::preprocess_images(
    const std::vector<cv::Mat>& camera_images,
    float* d_output_buffer,
    cudaStream_t stream)
{
    // Debug: Print configuration
    printf("MultiCameraPreprocessor config: input=%dx%d, output=%dx%d, cameras=%d\n",
           config_.input_width, config_.input_height, 
           config_.output_width, config_.output_height, config_.num_cameras);
    
    // Step 0: 入力データを検証
    cudaError_t validation_status = validate_input(camera_images);
    if (validation_status != cudaSuccess) {
        printf("Input validation failed with error: %s\n", cudaGetErrorString(validation_status));
        return validation_status;
    }
    
    // Debug: Print validation success
    printf("Input validation passed for %zu cameras\n", camera_images.size());

    // Step 1: ホスト(cv::Mat)から事前に確保したデバイスバッファへ画像データをコピー
    const size_t single_input_size = static_cast<size_t>(config_.input_width) * config_.input_height * 3;
    for (int32_t i = 0; i < config_.num_cameras; ++i) {
        const auto& img = camera_images.at(i);
        
        // 大きな入力バッファ内のコピー先ポインタを計算
        uint8_t* d_dst = d_input_buffer_ + i * single_input_size;
        
        // データを非同期にコピー
        cudaError_t copy_result = cudaMemcpyAsync(d_dst, img.data, single_input_size, cudaMemcpyHostToDevice, stream);
        if (copy_result != cudaSuccess) {
            printf("cudaMemcpyAsync failed for camera %d: %s\n", i, cudaGetErrorString(copy_result));
            return copy_result;
        }
    }
    
    printf("Image data copied to GPU successfully\n");

    // Step 2: resize + BGR->RGB変換 + 正規化 + 結合を行うカスタムカーネルを起動
    cudaError_t kernel_result = launch_multi_camera_preprocess_kernel(
        d_input_image_ptrs_,
        d_output_buffer,
        config_,
        stream
    );
    
    if (kernel_result != cudaSuccess) {
        printf("CUDA kernel launch failed: %s\n", cudaGetErrorString(kernel_result));
    } else {
        printf("CUDA kernel launched successfully\n");
    }
    
    return kernel_result;
}
