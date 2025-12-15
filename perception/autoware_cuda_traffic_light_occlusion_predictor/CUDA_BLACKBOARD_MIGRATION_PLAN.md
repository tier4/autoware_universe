# CUDA Blackboard Migration Plan

## Overview

Migrate from raw `sensor_msgs::msg::PointCloud2` to `cuda_blackboard::CudaPointCloud2` for:
- ✅ **Zero-copy** when connected to CUDA nodes
- ✅ **Automatic adaptation** from sensor_msgs when needed
- ✅ **Performance boost** in CUDA pipelines
- ✅ **Cleaner interface** - no manual device memory management in node

## Current vs Target Architecture

### Current (Raw Pointer)
```
sensor_msgs::PointCloud2 (host)
    ↓ (manual cudaMemcpy in node)
uint8_t* d_data (device)
    ↓
CudaOcclusionPredictor::predict(d_data, ...)
```

**Problems**:
- Manual memory management in node
- Always copies even if source is already on GPU
- No zero-copy benefit in CUDA pipelines

### Target (CUDA Blackboard)
```
cuda_blackboard::CudaPointCloud2
    ↓ (already on GPU if from CUDA node!)
CudaOcclusionPredictor::predict(cuda_cloud, ...)
```

**Benefits**:
- ✅ Zero-copy from CUDA nodes
- ✅ Automatic host-to-device copy for sensor_msgs
- ✅ No manual memory management in node
- ✅ Consistent with other Autoware CUDA nodes

## Implementation Steps

### Step 1: Update CUDA Predictor Interface ✅

**File**: `cuda_occlusion_predictor.hpp`

```cpp
// Change from:
void predict(
  const uint8_t * d_pointcloud_data,
  size_t num_points,
  size_t point_step,
  ...);

// To:
void predict(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & cloud_msg,
  const float * camera2cloud_transform,
  const std::vector<PointXYZ> & roi_3d_points,
  std::vector<int> & occlusion_ratios);
```

### Step 2: Update CUDA Predictor Implementation ✅

**File**: `cuda_occlusion_predictor.cu`

```cpp
void CudaOcclusionPredictor::predict(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & cloud_msg,
  const float * camera2cloud_transform,
  const std::vector<PointXYZ> & roi_3d_points,
  std::vector<int> & occlusion_ratios)
{
  // Validate
  if (!cloud_msg || !cloud_msg->data) return;
  
  const size_t num_points = cloud_msg->width * cloud_msg->height;
  const size_t point_step = cloud_msg->point_step;
  
  // Access GPU pointer directly - NO COPY if already on GPU!
  const uint8_t * d_pointcloud_data = cloud_msg->data.get();
  
  // Extract XYZ from GPU memory
  PointXYZ * d_input_points = allocateBufferFromPool<PointXYZ>(num_points);
  kernels::extractXYZLaunch(
    d_pointcloud_data, d_input_points, num_points, point_step, ...);
  
  // Continue with processing...
}
```

### Step 3: Update Node to Use CUDA Blackboard ✅

**File**: `cuda_traffic_light_occlusion_predictor_node.cpp`

```cpp
class CudaTrafficLightOcclusionPredictorNode {
private:
  // Change from sensor_msgs subscription to CUDA blackboard
  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<
    cuda_blackboard::CudaPointCloud2>> pointcloud_sub_;
  
  void pointcloudCallback(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr cloud_msg)
  {
    // Store for synchronization
    latest_cloud_msg_ = cloud_msg;
  }
  
  void syncCallback(...)
  {
    // Use CUDA blackboard directly - no manual copy!
    cuda_occlusion_predictor_->predict(
      latest_cloud_msg_,  // Already on GPU if from CUDA source!
      camera2cloud_transform,
      roi_3d_points,
      occlusion_ratios);
  }
};
```

### Step 4: Update Includes and Dependencies ✅

**Files to update**:
- `cuda_occlusion_predictor.hpp` - Add `#include <cuda_blackboard/cuda_pointcloud2.hpp>`
- `package.xml` - Add `<depend>cuda_blackboard</depend>`
- `CMakeLists.txt` - Add `cuda_blackboard` to `ament_auto_find_build_dependencies`

### Step 5: Remove Manual Memory Management ✅

**Remove from node**:
```cpp
// ❌ DELETE THIS - no longer needed!
uint8_t * copyPointCloudToDevice(...);
cudaStream_t stream_;
cudaMallocAsync/cudaFreeAsync in node
```

### Step 6: Update Tests ✅

**File**: `test_cuda_occlusion_predictor.cpp`

```cpp
// Change tests to use cuda_blackboard::CudaPointCloud2
TEST_F(CudaOcclusionPredictorTest, WithCudaBlackboard) {
  // Create CUDA blackboard message
  auto cloud_msg = std::make_shared<cuda_blackboard::CudaPointCloud2>();
  cloud_msg->width = 100;
  cloud_msg->height = 1;
  cloud_msg->point_step = 16;
  
  // Allocate GPU memory through blackboard
  size_t data_size = 100 * 16;
  cloud_msg->data = cuda_blackboard::make_unique<uint8_t[]>(data_size);
  
  // Copy test data to GPU
  std::vector<uint8_t> host_data(data_size);
  // ... fill host_data ...
  CHECK_CUDA_ERROR(cudaMemcpy(
    cloud_msg->data.get(), host_data.data(), data_size,
    cudaMemcpyHostToDevice));
  
  // Test
  std::vector<int> occlusion_ratios;
  EXPECT_NO_THROW({
    predictor_->predict(cloud_msg, transform, roi_points, occlusion_ratios);
  });
}
```

## Benefits Analysis

### Performance

| Scenario | Before | After | Improvement |
|----------|--------|-------|-------------|
| From sensor_msgs | Copy in node + process | Auto copy + process | Same |
| From CUDA node | Copy in node + process | Zero-copy + process | **~1-2ms saved** |

### Code Quality

| Metric | Before | After |
|--------|--------|-------|
| Lines in node | ~470 | ~430 (-40) |
| Manual CUDA in node | ✅ Yes | ❌ No |
| Error-prone | ⚠️ Medium | ✅ Low |
| Zero-copy ready | ❌ No | ✅ Yes |

### Maintenance

- ✅ Consistent with other CUDA nodes
- ✅ Less manual memory management
- ✅ Automatic adaptation to input type
- ✅ Future-proof for CUDA pipelines

## Migration Checklist

### Phase 1: Update Interfaces
- [ ] Update `cuda_occlusion_predictor.hpp` interface
- [ ] Update `cuda_occlusion_predictor.cu` implementation
- [ ] Update includes and forward declarations

### Phase 2: Update Node
- [ ] Replace sensor_msgs subscription with CUDA blackboard
- [ ] Remove manual memory management helpers
- [ ] Update synchronization logic
- [ ] Update callback signatures

### Phase 3: Update Build System
- [ ] Add cuda_blackboard to package.xml
- [ ] Update CMakeLists.txt dependencies
- [ ] Verify compilation

### Phase 4: Update Tests
- [ ] Update unit tests to use CUDA blackboard
- [ ] Update integration tests
- [ ] Verify all tests pass

### Phase 5: Documentation
- [ ] Update README with CUDA blackboard info
- [ ] Update architecture diagrams
- [ ] Add migration notes

## Testing Strategy

### Unit Tests
1. ✅ Empty CUDA blackboard message
2. ✅ Valid CUDA blackboard with data
3. ✅ Null CUDA blackboard pointer
4. ✅ Zero-width/height cloud

### Integration Tests
1. ✅ Subscribe from sensor_msgs source
2. ✅ Subscribe from CUDA blackboard source
3. ✅ Verify zero-copy behavior (performance test)

### Regression Tests
1. ✅ All existing functionality works
2. ✅ No performance degradation
3. ✅ Memory leaks check

## Rollout Plan

### Development
1. Create feature branch `feature/cuda-blackboard`
2. Implement changes step-by-step
3. Run tests after each step
4. Document changes

### Testing
1. Unit tests pass
2. Integration tests pass
3. Performance benchmarks
4. Memory leak checks

### Deployment
1. Merge to main
2. Update documentation
3. Notify team of interface change

## Expected Timeline

- **Step 1-2** (Interface update): ~30 minutes
- **Step 3** (Node update): ~45 minutes
- **Step 4** (Dependencies): ~15 minutes
- **Step 5** (Cleanup): ~30 minutes
- **Step 6** (Tests): ~1 hour
- **Documentation**: ~30 minutes

**Total**: ~3-4 hours

---

**Status**: READY TO IMPLEMENT
**Priority**: HIGH
**Risk**: LOW (Well-tested pattern in Autoware)

