# Refactoring Plan: CUDA Occlusion Predictor Interface

## Problem

Current implementation tries to pass `sensor_msgs::msg::PointCloud2` to CUDA files, which causes:
1. ROS2 message headers in CUDA compilation
2. Unnecessary host-to-device memory copies
3. Violation of best practices (sensor_msgs should only be in node, not CUDA lib)

## Solution Architecture

### Pattern from `autoware_cuda_pointcloud_preprocessor`

**Node Layer (.cpp):**
- Receives `sensor_msgs::msg::PointCloud2::ConstSharedPtr` from ROS2
- Converts to `cuda_blackboard::CudaPointCloud2::ConstSharedPtr` (zero-copy GPU pointer)
- Calls CUDA filter with GPU pointer

**CUDA Library Layer (.cu):**
- Receives `cuda_blackboard::CudaPointCloud2::ConstSharedPtr`
- Accesses GPU memory directly via `msg->data.get()`
- No sensor_msgs dependencies

## Refactoring Steps

### Step 1: Update Node to Convert Point Cloud

**File: `cuda_traffic_light_occlusion_predictor_node.cpp`**

```cpp
// Add conversion before calling predict
auto cuda_cloud = convertToCudaPointCloud(in_cloud_msg);
cuda_occlusion_predictor_->predict(
  cuda_cloud, camera2cloud_transform, roi_3d_points, cuda_occlusion_ratios);
```

Or simpler: **Create a helper to copy sensor_msgs to GPU**

```cpp
// In node: copy point cloud to GPU
std::unique_ptr<uint8_t[]> d_pointcloud_data = copyPointCloudToDevice(in_cloud_msg);
cuda_occlusion_predictor_->predict(
  d_pointcloud_data.get(), 
  in_cloud_msg->width * in_cloud_msg->height,
  in_cloud_msg->point_step,
  camera2cloud_transform, 
  roi_3d_points, 
  cuda_occlusion_ratios);
```

### Step 2: Update CUDA Predictor Interface

**File: `cuda_occlusion_predictor.hpp`**

```cpp
// Remove sensor_msgs include
// #include <sensor_msgs/msg/point_cloud2.hpp>  // REMOVE THIS

class CudaOcclusionPredictor
{
public:
  /**
   * @brief Predict occlusion ratios using GPU point cloud data
   *
   * @param d_pointcloud_data GPU pointer to point cloud data (already on device)
   * @param num_points Number of points in cloud
   * @param point_step Bytes per point
   * @param camera2cloud_transform 4x4 transformation matrix (row-major, 16 floats)
   * @param roi_3d_points Pre-calculated 3D ROI corners in camera frame
   * @param occlusion_ratios Output vector of occlusion ratios (0-100)
   */
  void predict(
    const uint8_t * d_pointcloud_data,  // GPU pointer
    size_t num_points,
    size_t point_step,
    const float * camera2cloud_transform,
    const std::vector<PointXYZ> & roi_3d_points,
    std::vector<int> & occlusion_ratios);
};
```

### Step 3: Update CUDA Implementation

**File: `cuda_occlusion_predictor.cu`**

```cpp
void CudaOcclusionPredictor::predict(
  const uint8_t * d_pointcloud_data,  // Already on GPU
  size_t num_points,
  size_t point_step,
  const float * camera2cloud_transform,
  const std::vector<PointXYZ> & roi_3d_points,
  std::vector<int> & occlusion_ratios)
{
  // No sensor_msgs dependencies!
  // Data is already on GPU, just extract XYZ values
  
  PointXYZ * d_input_points = allocateBufferFromPool<PointXYZ>(num_points);
  
  // Extract XYZ from raw point cloud bytes (GPU kernel)
  const int blocks = (num_points + threads_per_block_ - 1) / threads_per_block_;
  extractXYZKernel<<<blocks, threads_per_block_, 0, stream_>>>(
    d_pointcloud_data, d_input_points, num_points, point_step);
  
  // Continue with transformation, filtering, etc.
  // ...
}
```

### Step 4: Add XYZ Extraction Kernel

**File: `cuda_occlusion_kernels.cu`**

```cuda
__global__ void extractXYZKernel(
  const uint8_t * __restrict__ raw_points,
  PointXYZ * __restrict__ output_points,
  int num_points,
  int point_step)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) return;
  
  // Extract XYZ from byte array (assuming standard x,y,z at offset 0,4,8)
  const float * pt = reinterpret_cast<const float *>(raw_points + idx * point_step);
  
  if (isfinite(pt[0]) && isfinite(pt[1]) && isfinite(pt[2])) {
    output_points[idx] = {pt[0], pt[1], pt[2]};
  } else {
    output_points[idx] = {0.0f, 0.0f, 0.0f};  // Invalid marker
  }
}
```

### Step 5: Add Device Memory Copy Helper in Node

**File: `cuda_traffic_light_occlusion_predictor_node.cpp`**

```cpp
class CudaTrafficLightOcclusionPredictorNode
{
private:
  // Helper to copy point cloud to device
  uint8_t * copyPointCloudToDevice(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    cudaStream_t stream)
  {
    size_t data_size = cloud_msg->data.size();
    uint8_t * d_data = nullptr;
    
    CHECK_CUDA_ERROR(cudaMallocAsync(&d_data, data_size, stream));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      d_data, cloud_msg->data.data(), data_size,
      cudaMemcpyHostToDevice, stream));
    
    return d_data;
  }
  
  // Corresponding cleanup
  void freeDeviceMemory(uint8_t * d_data, cudaStream_t stream)
  {
    CHECK_CUDA_ERROR(cudaFreeAsync(d_data, stream));
  }
};
```

## Alternative: Use CUDA Blackboard (Recommended for Future)

For better integration, consider using `cuda_blackboard::CudaPointCloud2`:

```cpp
// In node: subscribe to CUDA blackboard directly
cuda_blackboard_sub_ = std::make_shared<
  cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
  *this, "~/input/pointcloud", ...);

// CUDA predictor interface
void predict(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & cloud_msg,
  const float * camera2cloud_transform,
  const std::vector<PointXYZ> & roi_3d_points,
  std::vector<int> & occlusion_ratios);

// Implementation accesses GPU pointer directly
const uint8_t * d_data = cloud_msg->data.get();  // Already on GPU!
```

## Files to Modify

1. ✅ `cuda_occlusion_predictor.hpp` - Update interface signature
2. ✅ `cuda_occlusion_predictor.cu` - Remove sensor_msgs, use raw GPU pointer
3. ✅ `cuda_occlusion_kernels.hpp` - Add extractXYZKernel declaration
4. ✅ `cuda_occlusion_kernels.cu` - Implement extractXYZKernel
5. ✅ `cuda_traffic_light_occlusion_predictor_node.cpp` - Add device copy helper, update predict call
6. ✅ `package.xml` - Keep dependencies (sensor_msgs only in node)
7. ✅ `CMakeLists.txt` - Verify CUDA lib doesn't link sensor_msgs

## Benefits

1. ✅ No ROS2 dependencies in CUDA code
2. ✅ Follows Autoware CUDA best practices
3. ✅ More efficient (direct GPU memory access)
4. ✅ Cleaner separation of concerns
5. ✅ Easier to test CUDA code independently

## Implementation Priority

**Phase 1 (Quick Fix):** Raw pointer approach
- Minimal changes
- Works with existing ROS2 subscriptions
- Node handles conversion

**Phase 2 (Future):** CUDA Blackboard integration
- Requires updating launch files
- Zero-copy from previous CUDA filters
- Best performance

