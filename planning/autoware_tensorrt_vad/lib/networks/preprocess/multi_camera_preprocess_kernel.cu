#include "autoware/tensorrt_vad/networks/preprocess/multi_camera_preprocess_kernel.hpp" // Configuration structures and declarations for CUDA kernel

/**
 * @brief Device function for image resize using bilinear interpolation
 *
 * @param src_img Source image pixel data (BGR)
 * @param src_width Source image width
 * @param src_height Source image height
 * @param dst_x Output image x-coordinate
 * @param dst_y Output image y-coordinate
 * @param scale_x Scale factor in x direction (src_width / dst_width)
 * @param scale_y Scale factor in y direction (src_height / dst_height)
 * @return uint3 Resized pixel value (BGR)
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
    // Convert output coordinates to input coordinate system
    float src_x = (dst_x + 0.5f) * scale_x - 0.5f;
    float src_y = (dst_y + 0.5f) * scale_y - 0.5f;
    
    // Clamp boundaries
    src_x = fmaxf(0.0f, fminf(src_x, src_width - 1.0f));
    src_y = fmaxf(0.0f, fminf(src_y, src_height - 1.0f));
    
    // Coordinates of four neighboring pixels
    int32_t x0 = static_cast<int32_t>(floorf(src_x));
    int32_t y0 = static_cast<int32_t>(floorf(src_y));
    int32_t x1 = static_cast<int32_t>(fminf(static_cast<float>(x0 + 1), static_cast<float>(src_width - 1)));
    int32_t y1 = static_cast<int32_t>(fminf(static_cast<float>(y0 + 1), static_cast<float>(src_height - 1)));
    
    // Interpolation weights
    float dx = src_x - x0;
    float dy = src_y - y0;
    
    // Get values of four neighboring pixels (BGR format)
    int32_t idx_00 = (y0 * src_width + x0) * 3;
    int32_t idx_10 = (y0 * src_width + x1) * 3;
    int32_t idx_01 = (y1 * src_width + x0) * 3;
    int32_t idx_11 = (y1 * src_width + x1) * 3;
    
    uint3 result;
    
    // B channel
    float b00 = src_img[idx_00 + 0];
    float b10 = src_img[idx_10 + 0];
    float b01 = src_img[idx_01 + 0];
    float b11 = src_img[idx_11 + 0];
    result.x = static_cast<uint8_t>(
        b00 * (1 - dx) * (1 - dy) + b10 * dx * (1 - dy) + 
        b01 * (1 - dx) * dy + b11 * dx * dy);
    
    // G channel
    float g00 = src_img[idx_00 + 1];
    float g10 = src_img[idx_10 + 1];
    float g01 = src_img[idx_01 + 1];
    float g11 = src_img[idx_11 + 1];
    result.y = static_cast<uint8_t>(
        g00 * (1 - dx) * (1 - dy) + g10 * dx * (1 - dy) + 
        g01 * (1 - dx) * dy + g11 * dx * dy);
    
    // R channel
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
 * @brief CUDA kernel for batch preprocessing of images from multiple cameras
 *
 * This kernel uses one block grid (blockIdx.z) per camera to perform parallel processing.
 * Each thread handles one pixel of the output image, performing the following fused operations:
 * - Resize using bilinear interpolation
 * - Rearrange channels from BGR format to RGB format (bgr2rgb)
 * - Normalize each channel ((pixel - mean) * inv_std) (normalization)
 * - Finally, write the results to a contiguous output buffer (CHW planar format)
 * This concatenates image data from multiple cameras into a single tensor.
 *
 * @param d_input_images Array of pointers to device input BGR images (uint8_t*)
 * @param d_output Single device output buffer (float*) to store all processed results
 * @param config Configuration structure containing parameters needed for preprocessing
 */
__global__ void multi_camera_preprocess_kernel(
    uint8_t** d_input_images,
    float* d_output,
    const MultiCameraPreprocessConfig config)
{
    const int32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const int32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    const int32_t camera_idx = blockIdx.z;

    // Exit early if thread is outside output image bounds (boundary check)
    if (x >= config.output_width || y >= config.output_height || camera_idx >= config.num_cameras) {
        return;
    }

    // Get pointer to input image for the camera this thread is responsible for
    uint8_t* input_img = d_input_images[camera_idx];
    
    // Resize using bilinear interpolation
    uint3 resized_pixel = bilinear_interpolation_resize(
        input_img, 
        config.input_width, 
        config.input_height,
        x, y, 
        config.scale_x, 
        config.scale_y);
    
    // Get BGR pixel values as float and convert BGR->RGB
    const float b_val = static_cast<float>(resized_pixel.x);
    const float g_val = static_cast<float>(resized_pixel.y);
    const float r_val = static_cast<float>(resized_pixel.z);

    // Normalize (applied in BGR->RGB order)
    const float norm_r = (r_val - config.mean[0]) * config.inverse_std[0];
    const float norm_g = (g_val - config.mean[1]) * config.inverse_std[1];
    const float norm_b = (b_val - config.mean[2]) * config.inverse_std[2];

    // Write to corresponding position in output buffer in CHW (Planar) format
    // This writing approach creates one large tensor with concatenated data from all cameras
    const int32_t single_camera_plane_size = config.output_width * config.output_height;
    const int32_t single_camera_total_size = 3 * single_camera_plane_size;
    const int32_t out_pixel_offset = y * config.output_width + x;

    const int32_t out_idx_base = camera_idx * single_camera_total_size;

    d_output[out_idx_base + 0 * single_camera_plane_size + out_pixel_offset] = norm_r; // R channel
    d_output[out_idx_base + 1 * single_camera_plane_size + out_pixel_offset] = norm_g; // G channel
    d_output[out_idx_base + 2 * single_camera_plane_size + out_pixel_offset] = norm_b; // B channel
}


/**
 * @brief Host-side wrapper function for launching multi_camera_preprocess_kernel
 *
 * Calculates grid and block dimensions required for CUDA kernel execution
 * and launches the kernel asynchronously.
 */
cudaError_t launch_multi_camera_preprocess_kernel(
    uint8_t** d_input_images,
    float* d_output,
    const MultiCameraPreprocessConfig& config,
    cudaStream_t stream)
{
    // Define threads per block (e.g., 16x16 = 256 threads)
    const dim3 threads_per_block(16, 16, 1);

    // Calculate required number of blocks (3D grid: width, height, number of cameras)
    const dim3 blocks_per_grid(
        (config.output_width + threads_per_block.x - 1) / threads_per_block.x,
        (config.output_height + threads_per_block.y - 1) / threads_per_block.y,
        config.num_cameras);

    // Launch CUDA kernel
    // The config structure is passed by value.
    // When parameters are passed by value to CUDA kernels, the CUDA runtime automatically 
    // copies the data from host (CPU) to device (GPU) special high-speed memory space 
    // (such as constant memory)
    // Reason 1 for copying each time: Small data size - MultiCameraPreprocessConfig structure 
    // consists of only a few int32_t and float values, with a total size of just a few dozen bytes.
    // Reason 2 for copying each time: Fast copy - CUDA runtime is optimized to perform extremely 
    // fast copies of such small data (typically up to a few hundred bytes). The time required 
    // for this copy is negligible compared to copying entire image data, NPP resize processing, 
    // or kernel execution time.
    multi_camera_preprocess_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
        d_input_images, d_output, config);

    // Check and return errors related to kernel launch
    return cudaGetLastError();
}
