#ifndef AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_HPP_
#define AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_HPP_

#include <vector>
#include <opencv2/core/mat.hpp>  // cv::Matを使用するためにインクルード
#include <memory>
#include <string>
#include <cuda_runtime.h>
#include "autoware/tensorrt_vad/ros_vad_logger.hpp"
#include "autoware/tensorrt_vad/networks/preprocess/multi_camera_preprocess_kernel.hpp"

/**
 * @class MultiCameraPreprocessor
 * @brief 複数カメラ画像のGPU前処理パイプラインを管理するクラス
 *
 * このクラスは、ホストからのcv::Mat画像の受け取り、デバイスへのコピー、
 * カスタムCUDAカーネルによるリサイズ・変換・正規化・結合までの一連の流れをカプセル化します。
 * NPPに依存せず、独自のbilinear interpolationを使用してリサイズを行います。
 * リソースはコンストラクタで確保(RAII)、デストラクタで解放されます。
 */
class MultiCameraPreprocessor {
public:
    // Template constructor to accept shared_ptr<LoggerType>
    template<typename LoggerType>
    MultiCameraPreprocessor(const MultiCameraPreprocessConfig& config, std::shared_ptr<LoggerType> logger);
    
    ~MultiCameraPreprocessor();

    // コピーコンストラクタとコピー代入演算子を禁止し、リソースの二重解放を防ぐ
    MultiCameraPreprocessor(const MultiCameraPreprocessor&) = delete;
    MultiCameraPreprocessor& operator=(const MultiCameraPreprocessor&) = delete;

  /**
   * @brief 複数台のカメラ画像(cv::Mat)をGPUで一括前処理します。
   * @param camera_images ホスト側の入力画像(cv::Mat)のベクトル。BGR8フォーマットであること。
   * @param d_output_buffer デバイス上の出力バッファへのポインタ。結果はここにCHW形式で書き込まれる。
   * @param stream 実行に使用するCUDAストリーム。
   * @return cudaError_t 実行ステータス。
   */
  cudaError_t preprocess_images(
      const std::vector<cv::Mat>& camera_images,
      float* d_output_buffer,
      cudaStream_t stream
  );private:
    /**
     * @brief 入力画像ベクトルが有効か検証します（サイズ、フォーマットなど）。
     * @param camera_images 検証対象の画像ベクトル。
     * @return cudaError_t 検証結果。成功時はcudaSuccess。
     */
    cudaError_t validate_input(const std::vector<cv::Mat>& camera_images) const;

    MultiCameraPreprocessConfig config_;
    std::shared_ptr<autoware::tensorrt_vad::VadLogger> logger_;  // Direct VadLogger pointer

    // --- GPU Buffers ---
    // 入力バッファ (コンストラクタで確保)
    uint8_t* d_input_buffer_{nullptr};      // 全ての生入力画像を格納する単一の連続バッファ
    uint8_t** d_input_image_ptrs_{nullptr}; // d_input_buffer_内の各画像の開始位置を指すポインタ配列
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
