# TF Operations Moved to C++

## Summary

All TF (Transform) operations have been moved from CUDA files (`.cu`) to C++ files (`.cpp`) to avoid ROS2 dependencies in CUDA compilation.

## Changes Made

### 1. CUDA Library Interface (`cuda_occlusion_predictor.hpp`)

**Before:**
```cpp
void predict(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & rois_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const tf2_ros::Buffer & tf_buffer,  // ❌ TF buffer in CUDA interface
  const std::map<lanelet::Id, tf2::Vector3> & traffic_light_position_map,
  std::vector<int> & occlusion_ratios);
```

**After:**
```cpp
void predict(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const float * camera2cloud_transform,  // ✅ Pre-calculated transformation matrix
  const std::vector<PointXYZ> & roi_3d_points,  // ✅ Pre-calculated ROI 3D points
  std::vector<int> & occlusion_ratios);
```

### 2. CUDA Implementation (`cuda_occlusion_predictor.cu`)

**Removed:**
- ❌ `#include <tf2_eigen/tf2_eigen.hpp>`
- ❌ `#include <tf2_ros/buffer.h>`
- ❌ `#include <tf2/LinearMath/Transform.h>`
- ❌ All TF lookup operations
- ❌ All ROI 3D projection (moved to node)

**Now Contains:**
- ✅ Pure CUDA operations
- ✅ Point cloud transformation (using pre-calculated matrix)
- ✅ Point cloud filtering
- ✅ Spherical coordinate conversion
- ✅ Occlusion detection

### 3. C++ Node Implementation (`cuda_traffic_light_occlusion_predictor_node.cpp`)

**Added:**
- ✅ TF lookup: `camera_info frame → pointcloud frame`
- ✅ TF lookup: `camera_info frame → map frame`
- ✅ Eigen matrix conversion to float array
- ✅ ROI 3D projection using OpenCV (`calcRoiVector3D`)
- ✅ Error handling for TF operations

**Flow:**
1. Lookup TF transformations (CPU-side)
2. Convert Eigen matrix to float array
3. Calculate ROI 3D points using OpenCV (CPU-side)
4. Call CUDA `predict()` with pre-calculated data
5. Map occlusion ratios back to signal indices

### 4. CMakeLists.txt

**CUDA Library:**
- Removed: `tf2_INCLUDE_DIRS`, `tf2_eigen_INCLUDE_DIRS`
- Kept: Only `sensor_msgs` for message types

**Node Library:**
- Added: `EIGEN3_INCLUDE_DIR` for matrix operations
- Kept: All TF and ROS2 dependencies

## Benefits

1. **No ROS2 in CUDA**: CUDA files compile without ROS2 node dependencies
2. **Clear Separation**: CPU operations (TF, OpenCV) in C++, GPU operations in CUDA
3. **Better Performance**: TF lookups done once in node, results reused
4. **Easier Testing**: CUDA library can be tested independently

## Interface Contract

**CUDA Library Receives:**
- Point cloud data
- Pre-calculated 4×4 transformation matrix (camera → LiDAR)
- Pre-calculated ROI 3D points in camera frame

**CUDA Library Returns:**
- Occlusion ratios (0-100) for each ROI

**Node Handles:**
- All TF lookups
- All OpenCV operations
- Error handling and logging
- Message synchronization

## Example Usage (in Node)

```cpp
// 1. Lookup TF (CPU)
Eigen::Matrix4d camera2cloud = tf2::transformToEigen(
  tf_buffer.lookupTransform(...)).matrix();

// 2. Convert to float array
float transform[16];
for (int i = 0; i < 4; i++) {
  for (int j = 0; j < 4; j++) {
    transform[i * 4 + j] = static_cast<float>(camera2cloud(i, j));
  }
}

// 3. Calculate ROI 3D points (CPU, OpenCV)
std::vector<PointXYZ> roi_3d_points;
// ... calculate using calcRoiVector3D ...

// 4. Call CUDA (GPU)
cuda_occlusion_predictor_->predict(
  cloud_msg, transform, roi_3d_points, occlusion_ratios);
```

## Files Modified

1. `include/.../cuda_occlusion_predictor.hpp` - Interface updated
2. `src/cuda_occlusion_predictor/cuda_occlusion_predictor.cu` - TF removed
3. `src/cuda_traffic_light_occlusion_predictor_node.cpp` - TF operations added
4. `CMakeLists.txt` - Dependencies updated

