#include "autoware/tensorrt_vad/networks/preprocess/multi_camera_preprocess.hpp" // 設定構造体の定義などを共有

/**
 * @brief bilinear interpolationを用いて画像リサイズを行うデバイス関数
 *
 * @param src_img 元画像のピクセルデータ
 * @param src_width 元画像の幅
 * @param src_height 元画像の高さ
 * @param dst_x 出力画像のx座標
 * @param dst_y 出力画像のy座標
 * @param scale_x x方向のスケールファクター (src_width / dst_width)
 * @param scale_y y方向のスケールファクター (src_height / dst_height)
 * @return uint3 リサイズされたピクセル値 (BGR)
 */
__device__ uint3 bilinear_interpolation_resize(
    uint8_t* src_img, 
    int32_t src_width, 
    int32_t src_height,
    int32_t dst_x, 
    int32_t dst_y, 
    float scale_x, 
    float scale_y)
{
    // 出力座標を入力座標系にマッピング
    float src_x = (dst_x + 0.5f) * scale_x - 0.5f;
    float src_y = (dst_y + 0.5f) * scale_y - 0.5f;
    
    // 境界をクランプ
    src_x = fmaxf(0.0f, fminf(src_x, src_width - 1.0f));
    src_y = fmaxf(0.0f, fminf(src_y, src_height - 1.0f));
    
    // 4つの近傍ピクセルの座標
    int32_t x0 = static_cast<int32_t>(floorf(src_x));
    int32_t y0 = static_cast<int32_t>(floorf(src_y));
    int32_t x1 = static_cast<int32_t>(fminf(static_cast<float>(x0 + 1), static_cast<float>(src_width - 1)));
    int32_t y1 = static_cast<int32_t>(fminf(static_cast<float>(y0 + 1), static_cast<float>(src_height - 1)));
    
    // 補間の重み
    float dx = src_x - x0;
    float dy = src_y - y0;
    
    // 4つの近傍ピクセルの値を取得 (BGRフォーマット)
    int32_t idx_00 = (y0 * src_width + x0) * 3;
    int32_t idx_10 = (y0 * src_width + x1) * 3;
    int32_t idx_01 = (y1 * src_width + x0) * 3;
    int32_t idx_11 = (y1 * src_width + x1) * 3;
    
    uint3 result;
    
    // Bチャンネル
    float b00 = src_img[idx_00 + 0];
    float b10 = src_img[idx_10 + 0];
    float b01 = src_img[idx_01 + 0];
    float b11 = src_img[idx_11 + 0];
    result.x = static_cast<uint8_t>(
        b00 * (1 - dx) * (1 - dy) + b10 * dx * (1 - dy) + 
        b01 * (1 - dx) * dy + b11 * dx * dy);
    
    // Gチャンネル
    float g00 = src_img[idx_00 + 1];
    float g10 = src_img[idx_10 + 1];
    float g01 = src_img[idx_01 + 1];
    float g11 = src_img[idx_11 + 1];
    result.y = static_cast<uint8_t>(
        g00 * (1 - dx) * (1 - dy) + g10 * dx * (1 - dy) + 
        g01 * (1 - dx) * dy + g11 * dx * dy);
    
    // Rチャンネル
    float r00 = src_img[idx_00 + 2];
    float r10 = src_img[idx_10 + 2];
    float r01 = src_img[idx_01 + 2];
    float r11 = src_img[idx_11 + 2];
    result.z = static_cast<uint8_t>(
        r00 * (1 - dx) * (1 - dy) + r10 * dx * (1 - dy) + 
        r01 * (1 - dx) * dy + r11 * dx * dy);
    
    return result;
}

/**
 * @brief 複数カメラからの画像を一括で前処理するCUDAカーネル
 *
 * このカーネルは、カメラごとに1つのブロックグリッド(blockIdx.z)を使用して並列処理を実行します。
 * 各スレッドは出力画像の1ピクセルを担当し、以下の処理を融合して行います。
 * - bilinear interpolationを使ったリサイズ
 * - BGRフォーマットからRGBフォーマットにチャンネルを並べ替える (bgr2rgb)
 * - 各チャンネルを正規化する ((pixel - mean) * inv_std) (normalization)
 * - 最終的な連続した出力バッファに結果を書き込む (CHW planar format)
 * これにより、複数カメラの画像データが1つのテンソルに結合(concatenate)されます。
 *
 * @param d_input_images デバイス上の入力BGR画像(uint8_t*)を指すポインタの配列
 * @param d_output すべての処理結果を格納する単一のデバイス上の出力バッファ(float*)
 * @param config 前処理に必要なパラメータを含む設定構造体
 */
__global__ void multi_camera_preprocess_kernel(
    uint8_t** d_input_images,
    float* d_output,
    const MultiCameraPreprocessConfig config)
{
    const int32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const int32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    const int32_t camera_idx = blockIdx.z;

    // グリッド内のスレッドが出力画像の範囲外であれば、何もせずに終了 (境界チェック)
    if (x >= config.output_width || y >= config.output_height || camera_idx >= config.num_cameras) {
        return;
    }

    // このスレッドが担当するカメラの入力画像へのポインタを取得
    uint8_t* input_img = d_input_images[camera_idx];
    
    // bilinear interpolationを使ってリサイズ
    uint3 resized_pixel = bilinear_interpolation_resize(
        input_img, 
        config.input_width, 
        config.input_height,
        x, y, 
        config.scale_x, 
        config.scale_y);
    
    // BGRピクセル値をfloat型として取得し、BGR->RGB変換
    const float b_val = static_cast<float>(resized_pixel.x);
    const float g_val = static_cast<float>(resized_pixel.y);
    const float r_val = static_cast<float>(resized_pixel.z);

    // 正規化 (BGR->RGBの順番で実施)
    const float norm_r = (r_val - config.mean[0]) * config.inverse_std[0];
    const float norm_g = (g_val - config.mean[1]) * config.inverse_std[1];
    const float norm_b = (b_val - config.mean[2]) * config.inverse_std[2];

    // CHW (Planar) フォーマットで出力バッファの対応する位置に書き込む
    // この書き込み方により、全カメラのデータが連結された一つの大きなテンソルが形成される
    const int32_t single_camera_plane_size = config.output_width * config.output_height;
    const int32_t single_camera_total_size = 3 * single_camera_plane_size;
    const int32_t out_pixel_offset = y * config.output_width + x;

    const int32_t out_idx_base = camera_idx * single_camera_total_size;

    d_output[out_idx_base + 0 * single_camera_plane_size + out_pixel_offset] = norm_r; // Rチャンネル
    d_output[out_idx_base + 1 * single_camera_plane_size + out_pixel_offset] = norm_g; // Gチャンネル
    d_output[out_idx_base + 2 * single_camera_plane_size + out_pixel_offset] = norm_b; // Bチャンネル
}


/**
 * @brief multi_camera_preprocess_kernelを起動するためのホスト側ラッパー関数
 *
 * CUDAカーネルの実行に必要なグリッドとブロックのディメンションを計算し、
 * カーネルを非同期に起動します。
 */
cudaError_t launch_multi_camera_preprocess_kernel(
    uint8_t** d_input_images,
    float* d_output,
    const MultiCameraPreprocessConfig& config,
    cudaStream_t stream)
{
    // 1ブロックあたりのスレッド数を定義 (例: 16x16 = 256スレッド)
    const dim3 threads_per_block(16, 16, 1);

    // 必要なブロック数を計算 (3Dグリッド: 幅, 高さ, カメラ数)
    const dim3 blocks_per_grid(
        (config.output_width + threads_per_block.x - 1) / threads_per_block.x,
        (config.output_height + threads_per_block.y - 1) / threads_per_block.y,
        config.num_cameras);

    // CUDAカーネルを起動
    // config構造体は値渡しされる。
    // CUDAカーネルにパラメータを値渡しすると、CUDAランタイムがそのデータをホスト（CPU側）からデバイス（GPU側）の非常に高速な特殊メモリ空間（コンスタントメモリなど）に自動的にコピーされる
    // 毎回コピーするほうが良い理由1: データが小さい: MultiCameraPreprocessConfig 構造体は、数個の int32_t と float だけで構成されており、サイズは数十バイト程度と非常に小さいです。
    // 毎回コピーするほうが良い理由2: コピーが高速: CUDAランタイムは、このような小さなデータ（通常は数百バイトまで）のコピーを極めて高速に行うように最適化されています。このコピーにかかる時間は、画像データ全体のコピーやNPPのリサイズ処理、カーネル本体の実行時間と比較すると、無視できるほどわずかです。
    multi_camera_preprocess_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
        d_input_images, d_output, config);

    // カーネル起動に関するエラーをチェックして返す
    return cudaGetLastError();
}
