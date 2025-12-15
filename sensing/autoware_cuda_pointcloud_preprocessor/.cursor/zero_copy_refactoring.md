# Zero-Copy Refactoring Plan

## Problem
Current implementation copies GPU data at every filter:
```cpp
// transformPointcloudPublic creates new CudaPointCloud2 and copies data
output->data = make_unique<uint8_t[]>(size);
cudaMemcpyAsync(output->data.get(), device_transformed_points_.data(), ...);
```

This is wasteful! We should only work on internal GPU buffers and create output only when publishing.

## Solution: Pass Metadata + Buffer References

### New Processing State Structure
```cpp
struct PointcloudProcessingState {
  // Metadata
  std_msgs::msg::Header header;
  uint32_t width;
  uint32_t height;
  std::vector<sensor_msgs::msg::PointField> fields;
  bool is_dense;
  bool is_bigendian;
  
  // Which internal buffer to use
  enum class BufferType {
    ORGANIZED,      // device_organized_points_
    TRANSFORMED,    // device_transformed_points_
  };
  BufferType active_buffer;
  
  // Reference to shared preprocessor (owns the actual GPU buffers)
  CudaPointcloudPreprocessor* preprocessor;
};
```

### Filter Changes
Filters no longer return `unique_ptr<CudaPointCloud2>`, they:
1. Update metadata
2. Set which buffer is active
3. Write to GPU buffers directly

### Only Materialize at Output
```cpp
// In DAG node publishing
auto final_state = get_processing_state_from_dag();
auto output_unique = final_state->materializeToCudaPointCloud2();
publisher->publish(std::move(output_unique));
```

## Benefits
- **Zero copies** during pipeline (only at final output)
- **Minimal memory allocations**
- **Maximum GPU pipeline efficiency**
- **Simple finalization** (just extract from the final buffer)

## Implementation Steps
1. Define `PointcloudProcessingState` struct
2. Update filters to work with state instead of CudaPointCloud2
3. Update DAG executor to pass state through
4. Implement `materializeToCudaPointCloud2()` for final output
5. Remove all intermediate copy operations

---

**Expected Performance**: 3-4x faster than current, approaching monolithic preprocessor speed

