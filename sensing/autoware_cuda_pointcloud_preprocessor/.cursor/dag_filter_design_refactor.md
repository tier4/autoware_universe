# DAG Filter Design Refactor - Lightweight Wrapper Approach

## Problem with Previous Design

The OrganizeFilter implementation duplicated significant CUDA logic:
- Copied device vector management
- Duplicated organize/gather kernel launching
- Recreated buffer resizing logic
- ~280 lines of duplicated code

**This violates DRY (Don't Repeat Yourself) principle and makes maintenance difficult.**

## New Design: Lightweight Wrappers

### Core Principle

**Filters should be THIN wrappers that delegate to existing implementations.**

### Design Pattern

Instead of reimplementing functionality, filters should:
1. Extract parameters from `std::map<std::string, std::any>`
2. Call existing class methods
3. Convert input/output types
4. That's it!

### Architecture

```
┌─────────────────────────────────────────────┐
│  DAG Filter (Thin Wrapper)                  │
│  - IFilter interface implementation         │
│  - Parameter extraction (5-10 lines)        │
│  - Input/output type conversion (5-10 lines)│
│  - Delegate to existing class (1 line)      │
│  Total: ~20-30 lines per filter             │
└─────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────┐
│  Existing Implementation                    │
│  - CudaPointcloudPreprocessor               │
│  - CudaVoxelGridDownsampleFilter            │
│  - CudaPolarVoxelOutlierFilter              │
│  NO CHANGES NEEDED                          │
└─────────────────────────────────────────────┘
```

## Implementation Strategy

### Option 1: Use Existing Monolithic Preprocessor

**For the complete pipeline**, create ONE filter that wraps `CudaPointcloudPreprocessor`:

```cpp
class FullPreprocessorFilter : public IFilter {
  std::unique_ptr<CudaPointcloudPreprocessor> preprocessor_;
  
  void process(...) override {
    // Just call existing process() method
    auto output = preprocessor_->process(input, transform, twist, imu, timestamp);
    outputs["output"] = output;
  }
};
```

**Pros**: 
- Minimal code (~50 lines)
- Reuses 100% of existing code
- Works immediately

**Cons**:
- Not granular - can't reorder steps
- Doesn't achieve the DAG flexibility goal

### Option 2: Refactor Existing Preprocessor (RECOMMENDED)

**Extract individual processing steps as separate methods**, then wrap them:

#### Step 1: Refactor CudaPointcloudPreprocessor

Make internal methods public or create accessors:

```cpp
// cuda_pointcloud_preprocessor.hpp
class CudaPointcloudPreprocessor {
public:
  // Existing
  std::unique_ptr<CudaPointCloud2> process(...);
  
  // NEW: Expose individual steps
  void organizePointcloud(
    const sensor_msgs::msg::PointCloud2 & input,
    cuda_blackboard::CudaPointCloud2 & output);
    
  void transformPointcloud(
    const cuda_blackboard::CudaPointCloud2 & input,
    const geometry_msgs::msg::TransformStamped & transform,
    cuda_blackboard::CudaPointCloud2 & output);
    
  void cropBox(
    const cuda_blackboard::CudaPointCloud2 & input,
    const std::vector<CropBoxParameters> & boxes,
    cuda_blackboard::CudaPointCloud2 & output);
    
  void correctDistortion(
    const cuda_blackboard::CudaPointCloud2 & input,
    const std::deque<TwistMsg> & twist_queue,
    const std::deque<AngularVelMsg> & imu_queue,
    UndistortionType type,
    cuda_blackboard::CudaPointCloud2 & output);
    
  void filterRingOutliers(
    const cuda_blackboard::CudaPointCloud2 & input,
    const RingOutlierFilterParameters & params,
    cuda_blackboard::CudaPointCloud2 & output);
};
```

#### Step 2: Create Lightweight Filter Wrappers

```cpp
// organize_filter.hpp (15-20 lines)
class OrganizeFilter : public IFilter {
  CudaPointcloudPreprocessor preprocessor_;  // Or shared instance
  
  void process(...) override {
    auto input = std::static_pointer_cast<PointCloud2>(inputs.at("pointcloud"));
    auto output = std::make_unique<CudaPointCloud2>();
    preprocessor_.organizePointcloud(*input, *output);
    outputs["organized"] = std::move(output);
  }
};

// crop_box_filter.hpp (20-25 lines)
class CropBoxFilter : public IFilter {
  CudaPointcloudPreprocessor preprocessor_;
  std::vector<CropBoxParameters> boxes_;
  
  void initialize(const std::map<std::string, std::any> & params) override {
    // Extract crop_boxes from params
    boxes_ = extractCropBoxes(params);
  }
  
  void process(...) override {
    auto input = std::static_pointer_cast<CudaPointCloud2>(inputs.at("input"));
    auto output = std::make_unique<CudaPointCloud2>();
    preprocessor_.cropBox(*input, boxes_, *output);
    outputs["output"] = std::move(output);
  }
};
```

### Option 3: Shared Preprocessor Instance (MOST EFFICIENT)

Instead of each filter having its own preprocessor, share ONE instance:

```cpp
// In FilterContext
struct FilterContext {
  cudaStream_t stream{};
  cudaMemPool_t memory_pool{};
  
  // SHARED preprocessing instance
  CudaPointcloudPreprocessor * shared_preprocessor{nullptr};
  
  tf2_ros::Buffer * tf_buffer{nullptr};
  // ...
};

// Filters just call methods on shared instance
class OrganizeFilter : public IFilter {
  void process(..., FilterContext & context) override {
    context.shared_preprocessor->organizePointcloud(...);
  }
};
```

**Benefits**:
- Zero duplication
- Shared CUDA resources
- Minimal memory overhead
- Each filter is 10-20 lines

## Recommended Implementation Plan

### Phase 1: Minimal Refactoring (Quick Win)

1. **Add public wrapper methods** to `CudaPointcloudPreprocessor`:
   - Keep all private implementation
   - Add thin public methods that expose individual steps
   - No changes to internal logic

2. **Create lightweight filter wrappers**:
   - Each filter: 15-25 lines
   - Just parameter extraction + method call
   - No CUDA code duplication

3. **Use shared preprocessor instance**:
   - Add to `FilterContext`
   - Filters call methods on shared instance

### Phase 2: Full Granularity (If Needed)

If we need finer control:
1. Extract individual kernel launching into separate classes
2. But still reuse existing kernels (no duplication)
3. Share buffers through context

## Code Example: Proper Lightweight Wrapper

```cpp
// organize_filter.hpp (~20 lines)
#ifndef ...ORGANIZE_FILTER_HPP_
#define ...ORGANIZE_FILTER_HPP_

#include "autoware/cuda_pointcloud_preprocessor/dag/filter_interface.hpp"
#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"

namespace autoware::cuda_pointcloud_preprocessor::dag {

class OrganizeFilter : public IFilter {
public:
  void initialize(const std::map<std::string, std::any> &) override {}
  void process(
    const std::map<std::string, std::shared_ptr<void>> & inputs,
    std::map<std::string, std::shared_ptr<void>> & outputs,
    FilterContext & context) override;
  FilterMetadata getMetadata() const override;
  bool validateInputs(const std::map<std::string, std::shared_ptr<void>> &) const override;
};

}  // namespace
#endif
```

```cpp
// organize_filter.cpp (~30 lines)
#include "organize_filter.hpp"

void OrganizeFilter::process(...) {
  // Get input
  auto input = std::static_pointer_cast<sensor_msgs::msg::PointCloud2>(
    inputs.at("pointcloud"));
  
  // Delegate to shared preprocessor
  auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  context.shared_preprocessor->organizePointcloud(*input, *output);
  
  // Return output
  outputs["organized"] = std::static_pointer_cast<void>(std::move(output));
}

FilterMetadata OrganizeFilter::getMetadata() const {
  return {
    .filter_type = "OrganizeFilter",
    .required_inputs = {"pointcloud"},
    .outputs = {"organized"}
  };
}

bool OrganizeFilter::validateInputs(...) const {
  return inputs.find("pointcloud") != inputs.end();
}
```

**Total: ~50 lines vs 280 lines!**

## Migration Path

### Immediate (This PR)

1. Delete current `organize_filter.cpp` (280 lines)
2. Add lightweight wrapper methods to `CudaPointcloudPreprocessor`
3. Create thin filter wrappers (~20 lines each)
4. Add `shared_preprocessor` to `FilterContext`

### Future (Follow-up PR)

1. Extract individual processing steps if needed
2. Create more granular filter options
3. Optimize buffer management

## Summary

**Key Insight**: Filters should be INTERFACES, not IMPLEMENTATIONS.

- ❌ Bad: Duplicate CUDA logic in each filter
- ✅ Good: Delegate to existing implementations
- ✅ Better: Share one preprocessor instance across all filters

This approach:
- Minimizes code duplication
- Maintains existing implementations
- Keeps filters simple and maintainable
- Achieves DAG flexibility without reimplementation

