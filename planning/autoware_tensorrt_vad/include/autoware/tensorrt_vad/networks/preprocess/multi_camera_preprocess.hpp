#ifndef AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_HPP_
#define AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_HPP_

#include <cuda_runtime.h>
#include <vector>
#include <cstdint>
#include <opencv2/core/mat.hpp>  // cv::Matを使用するためにインクルード

/**
 * @struct MultiCameraPreprocessConfig
 * @brief CUDAによる前処理に必要な設定パラメータを保持する構造体
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

// CUDAカーネルを起動するホスト側ラッパー関数の前方宣言
cudaError_t launch_multi_camera_preprocess_kernel(
    uint8_t** d_input_images,
    float* d_output,
    const MultiCameraPreprocessConfig& config,
    cudaStream_t stream
);

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
    explicit MultiCameraPreprocessor(const MultiCameraPreprocessConfig& config);
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

    // --- GPU Buffers ---
    // 入力バッファ (コンストラクタで確保)
    uint8_t* d_input_buffer_{nullptr};      // 全ての生入力画像を格納する単一の連続バッファ
    uint8_t** d_input_image_ptrs_{nullptr}; // d_input_buffer_内の各画像の開始位置を指すポインタ配列
};

#endif // AUTOWARE_TENSORRT_VAD_MULTI_CAMERA_PREPROCESS_HPP_
