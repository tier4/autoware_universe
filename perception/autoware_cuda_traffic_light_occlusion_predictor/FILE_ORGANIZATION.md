# File Organization Guide

This document explains the organization of files in the CUDA traffic light occlusion predictor package.

## File Type Separation

### `.cu` Files (CUDA Source Files)
**Purpose**: Contains CUDA kernels (device code) and host code that uses CUDA APIs.

**Files**:
- `src/cuda_occlusion_predictor/cuda_occlusion_kernels.cu`
  - Contains CUDA kernel implementations (`__global__` functions)
  - Contains launch functions that call kernels
  - Pure CUDA device code

- `src/cuda_occlusion_predictor/cuda_occlusion_predictor.cu`
  - Contains `CudaOcclusionPredictor` class implementation
  - Host code that uses CUDA APIs (cudaMalloc, cudaStream, etc.)
  - Memory management with CUDA memory pools
  - Template instantiations for CUDA types
  - Can contain CPU-side helper functions that are called from CUDA code

### `.cpp` Files (C++ Source Files)
**Purpose**: Contains pure C++ host code without CUDA dependencies.

**Files**:
- `src/cuda_traffic_light_occlusion_predictor_node.cpp`
  - ROS2 node implementation
  - Subscriber/publisher setup
  - Message synchronization
  - Pure C++ code (no direct CUDA API calls)
  - Links against CUDA library but doesn't compile with nvcc

## Build System Organization

### CUDA Library (`cuda_traffic_light_occlusion_predictor_lib`)
- **Type**: Shared library compiled with `nvcc` (CUDA compiler)
- **Files**: All `.cu` files
- **Purpose**: Contains all CUDA kernels and CUDA API usage
- **CMake**: Uses `cuda_add_library()`

### ROS2 Node Library (`cuda_traffic_light_occlusion_predictor`)
- **Type**: Shared library compiled with `g++` (C++ compiler)
- **Files**: All `.cpp` files
- **Purpose**: ROS2 node implementation
- **CMake**: Uses `ament_auto_add_library()`
- **Links**: Against CUDA library and CUDA runtime

## Why This Separation?

1. **Compilation**: `.cu` files must be compiled with `nvcc` (CUDA compiler), while `.cpp` files are compiled with standard C++ compiler
2. **Dependencies**: `.cu` files can use CUDA-specific headers and APIs, `.cpp` files should avoid direct CUDA includes
3. **Performance**: Separating allows better optimization for each type
4. **Maintainability**: Clear separation makes code easier to understand and maintain

## File Contents Summary

### `cuda_occlusion_kernels.cu`
- `transformPointCloudKernel` - CUDA kernel
- `filterPointCloudKernel` - CUDA kernel
- `convertToSphericalKernel` - CUDA kernel
- `sampleRoiKernel` - CUDA kernel
- `detectOcclusionKernel` - CUDA kernel
- Launch functions for each kernel

### `cuda_occlusion_predictor.cu`
- `CudaOcclusionPredictor::CudaOcclusionPredictor()` - Constructor (uses CUDA APIs)
- `CudaOcclusionPredictor::~CudaOcclusionPredictor()` - Destructor (uses CUDA APIs)
- `CudaOcclusionPredictor::allocateBufferFromPool()` - CUDA memory allocation
- `CudaOcclusionPredictor::returnBufferToPool()` - CUDA memory deallocation
- `CudaOcclusionPredictor::calcRoiVector3D()` - CPU helper function (uses OpenCV)
- `CudaOcclusionPredictor::predict()` - Main prediction pipeline (orchestrates CUDA calls)
- Template instantiations

### `cuda_traffic_light_occlusion_predictor_node.cpp`
- `CudaTrafficLightOcclusionPredictorNode` class definition
- Constructor with ROS2 parameter declaration
- `mapCallback()` - Map message handler
- `syncCallback()` - Synchronized message callback
- Node registration with `RCLCPP_COMPONENTS_REGISTER_NODE`

## Header Files

All header files are in `.hpp` format and can be included by both `.cu` and `.cpp` files:
- `cuda_occlusion_predictor.hpp` - Class declaration
- `cuda_occlusion_kernels.hpp` - Kernel launch function declarations

## Best Practices

1. **Keep CUDA API calls in `.cu` files**: If a function uses `cudaMalloc`, `cudaStream`, etc., it should be in a `.cu` file
2. **Keep ROS2 code in `.cpp` files**: ROS2 node code should be in `.cpp` files
3. **Minimize dependencies**: `.cpp` files should not include CUDA headers directly
4. **Template instantiations**: Must be in `.cu` files for CUDA types
5. **Error handling**: Use `CHECK_CUDA_ERROR` macro in `.cu` files

## Common Mistakes to Avoid

❌ **Don't**: Put ROS2 node code in `.cu` file
✅ **Do**: Put ROS2 node code in `.cpp` file

❌ **Don't**: Include CUDA headers in `.cpp` files unnecessarily
✅ **Do**: Only include CUDA headers in `.cu` files or headers that are included by `.cu` files

❌ **Don't**: Mix CUDA API calls with pure C++ code in `.cpp` files
✅ **Do**: Keep CUDA API calls in `.cu` files, call them from `.cpp` through class methods

