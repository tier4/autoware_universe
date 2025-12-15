# Code Separation: C++ vs CUDA

## Critical Rule

**CUDA files (`.cu`) must NOT include:**
- ❌ ROS2 node headers (`rclcpp/rclcpp.hpp`)
- ❌ OpenCV headers (`opencv2/*`, `image_geometry/*`)
- ❌ CPU-only libraries that don't compile with `nvcc`

**C++ files (`.cpp`) can include:**
- ✅ All ROS2 headers
- ✅ OpenCV/image_geometry
- ✅ Any CPU libraries

## Current File Organization

### CUDA Library Files (`.cu`)

**Files:**
- `src/cuda_occlusion_predictor/cuda_occlusion_kernels.cu`
- `src/cuda_occlusion_predictor/cuda_occlusion_predictor.cu`

**Allowed Includes:**
- CUDA runtime (`cuda_runtime.h`)
- CUDA utilities (`autoware_cuda_utils/*`)
- Thrust library
- ROS2 message types (forward declarations or includes in `.cu` implementation)
- Eigen (for transformations)
- Standard C++ libraries

**NOT Allowed:**
- `rclcpp/rclcpp.hpp` - ROS2 node headers
- `image_geometry/*` - OpenCV-based camera model
- Any OpenCV headers

**What Goes Here:**
- CUDA kernel implementations (`__global__` functions)
- CUDA kernel launch functions
- CUDA memory management
- Host code that orchestrates CUDA operations
- Functions that use CUDA APIs

### C++ Node Files (`.cpp`)

**Files:**
- `src/cuda_traffic_light_occlusion_predictor_node.cpp`

**Allowed Includes:**
- Everything! All ROS2 headers, OpenCV, etc.

**What Goes Here:**
- ROS2 node class implementation
- Subscriber/publisher setup
- Message synchronization
- CPU-side computations (like ROI 3D projection using OpenCV)
- Logging with RCLCPP macros
- Error handling and user-facing messages

## Specific Example: `calcRoiVector3D`

**Problem:** This function uses OpenCV's `PinholeCameraModel` which is CPU-only.

**Solution:** Moved from `.cu` file to `.cpp` node file.

**Before (WRONG):**
```cpp
// In cuda_occlusion_predictor.cu
#include <image_geometry/pinhole_camera_model.h>  // ❌ OpenCV in CUDA file
void CudaOcclusionPredictor::calcRoiVector3D(...) {
  // Uses cv::Point2d, cv::Point3d, PinholeCameraModel
}
```

**After (CORRECT):**
```cpp
// In cuda_traffic_light_occlusion_predictor_node.cpp
#include <image_geometry/pinhole_camera_model.h>  // ✅ OpenCV in C++ file
namespace {
void calcRoiVector3D(...) {  // Helper function in node file
  // Uses cv::Point2d, cv::Point3d, PinholeCameraModel
}
}
```

## Header File Organization

### `cuda_occlusion_kernels.hpp`
- **Purpose:** Kernel launch function declarations
- **Includes:** Only CUDA runtime, basic types
- **Contains:** `PointXYZ`, `Ray` structs, kernel launch functions
- **Used by:** Both `.cu` and `.cpp` files

### `cuda_occlusion_predictor.hpp`
- **Purpose:** CUDA filter class declaration
- **Includes:** Forward declarations only (no full includes)
- **Contains:** Class interface, forward declarations
- **Used by:** `.cu` implementation file and `.cpp` node file

## Build System Separation

### CUDA Library (`cuda_traffic_light_occlusion_predictor_lib`)
- **CMake:** `cuda_add_library()` - compiled with `nvcc`
- **Files:** All `.cu` files
- **Includes:** NO `rclcpp`, NO `image_geometry`
- **Dependencies:** CUDA, Thrust, message types (forward decls)

### ROS2 Node Library (`cuda_traffic_light_occlusion_predictor`)
- **CMake:** `ament_auto_add_library()` - compiled with `g++`
- **Files:** All `.cpp` files
- **Includes:** Everything (rclcpp, image_geometry, etc.)
- **Dependencies:** All ROS2 packages, OpenCV, etc.

## Common Mistakes

### ❌ Mistake 1: Including ROS2 node headers in CUDA files
```cpp
// WRONG in .cu file
#include <rclcpp/rclcpp.hpp>
RCLCPP_INFO(...);  // Won't compile with nvcc
```

### ❌ Mistake 2: Using OpenCV in CUDA files
```cpp
// WRONG in .cu file
#include <image_geometry/pinhole_camera_model.h>
cv::Point2d pixel(...);  // OpenCV doesn't compile with nvcc
```

### ❌ Mistake 3: Mixing CUDA and CPU code incorrectly
```cpp
// WRONG - CPU-only function in .cu file
void someFunction() {
  // Uses std::map, std::vector - OK
  // Uses cv::Mat - NOT OK in .cu
}
```

## Correct Patterns

### ✅ Pattern 1: Forward declarations in headers
```cpp
// In .hpp (CUDA library header)
namespace sensor_msgs { namespace msg { struct PointCloud2; } }
// Full include only in .cu implementation
```

### ✅ Pattern 2: CPU computations in node file
```cpp
// In .cpp node file
void calcRoiVector3D(...) {
  // Uses OpenCV - perfectly fine here
  cv::Point2d pixel(...);
}
```

### ✅ Pattern 3: CUDA operations in .cu file
```cpp
// In .cu file
__global__ void kernel(...) {
  // Pure CUDA device code
}
void launchKernel(...) {
  kernel<<<...>>>(...);  // Host code calling CUDA
}
```

## Summary

| Code Type | File Extension | Compiler | Can Include |
|-----------|---------------|----------|-------------|
| CUDA kernels | `.cu` | `nvcc` | CUDA, Thrust, message types (forward decls) |
| CUDA host code | `.cu` | `nvcc` | Same as kernels + CUDA APIs |
| ROS2 node | `.cpp` | `g++` | Everything (ROS2, OpenCV, etc.) |
| Headers | `.hpp` | Both | Forward declarations preferred |

**Golden Rule:** If it doesn't compile with `nvcc`, it doesn't belong in a `.cu` file!

