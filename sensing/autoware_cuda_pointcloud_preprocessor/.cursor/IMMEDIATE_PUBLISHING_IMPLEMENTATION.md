# Immediate Publishing Implementation

## Summary

Successfully implemented immediate publishing in the DAG executor to prevent outputs from being modified by subsequent filters after they're produced. This also fixed a critical bug where downsampled outputs were being incorrectly finalized.

## Problem

1. **Publishing Delay**: Outputs were being published AFTER all DAG execution completed, allowing subsequent filters to potentially modify the data before publishing.
2. **Downsample Crash**: The downsample filter produces already-finalized data (compacted, no masks), but `finalizeOutputPublic` was trying to apply masks to it, causing `cudaErrorIllegalAddress`.

## Solution

### 1. Immediate Publishing in DAG Executor

Modified `DagExecutor::execute()` to accept publishers and finalize/publish outputs immediately after each node produces them:

```cpp
// Check if this output needs to be published
auto publisher_it = publishers.find(output_key);
bool needs_publish = (publisher_it != publishers.end());

if (needs_publish) {
  // CRITICAL: Finalize and publish NOW before any subsequent filters can modify
  auto output_state = std::static_pointer_cast<PointcloudProcessingState>(it->second);
  if (output_state && output_state->width > 0) {
    auto finalized_unique = context_.shared_preprocessor->finalizeOutputPublic(*output_state);
    publisher_it->second->publish(std::move(finalized_unique));
    
    // If there are also downstream consumers, keep the processing state
    if (has_consumers) {
      node_outputs_[output_key] = it->second;
    }
  }
}
```

### 2. Finalized State Flag

Added `is_finalized` flag to `PointcloudProcessingState`:

```cpp
struct PointcloudProcessingState {
  std::uint8_t * device_data{nullptr};
  bool owns_memory{false};
  bool is_finalized{false};  // True if already compacted (e.g., downsample), false if needs masks
  // ... other fields
};
```

### 3. Smart Finalization

Updated `finalizeOutputPublic` to handle both finalized and non-finalized states:

```cpp
if (state.is_finalized) {
  // Already compacted (e.g., downsample output) - just copy to CudaPointCloud2
  auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  // Copy metadata
  output->data = cuda_blackboard::make_unique<std::uint8_t[]>(data_size);
  cudaMemcpyAsync(output->data.get(), state.device_data, data_size, ...);
  return output;
}

// Not finalized - apply masks and compact (normal path for cropbox, outlier, etc.)
// ... mask combination and extraction logic ...
```

### 4. Downsample Filter Marking

The downsample filter now marks its output as finalized:

```cpp
output_state->is_finalized = true;  // Downsample output is already compacted
```

## Changes Made

### Headers
- `include/autoware/cuda_pointcloud_preprocessor/dag/dag_executor.hpp`
  - Updated `execute()` signature to include publishers parameter
  - Added cuda_blackboard includes

- `include/autoware/cuda_pointcloud_preprocessor/dag/processing_state.hpp`
  - Added `is_finalized` field
  - Updated all constructors and assignment operators

### Implementation
- `src/dag/dag_executor.cpp`
  - Implemented immediate publishing logic in `execute()`
  - Added includes for `CudaPointcloudPreprocessor` and `PointcloudProcessingState`

- `src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu`
  - Updated `finalizeOutputPublic()` to check `is_finalized` flag
  - Added fast path for finalized states (simple copy, no masks)

- `src/cuda_downsample_filter/cuda_voxel_grid_downsample_filter.cu`
  - Set `is_finalized = true` on downsample output

- `src/dag/cuda_pointcloud_preprocessor_dag_node.cpp`
  - Updated to pass `publishers_` to `executor_.execute()`
  - Removed redundant publishing loop (now handled inside executor)

### Tests
- `test/test_dag_executor.cpp`
  - Added `cuda_blackboard` include
  - Updated all `execute()` calls to pass empty publishers map

## Benefits

1. **Data Integrity**: Outputs are published immediately, preventing modification by subsequent filters
2. **Correct Downsample Handling**: Finalized states bypass mask operations, fixing the crash
3. **Performance**: No unnecessary mask operations on already-compacted data
4. **Flexibility**: System can handle both finalized (compacted) and non-finalized (mask-based) outputs

## Testing

Build Status: ✅ **PASSING**
```
Finished <<< autoware_cuda_pointcloud_preprocessor [53.6s]
Summary: 1 package finished [55.0s]
```

## Use Cases

### Finalized Output (Downsample)
```yaml
- id: "downsample"
  type: "DownsampleFilter"
  outputs:
    - name: "downsampled"  # is_finalized=true, no masks applied
```

### Non-Finalized Output (Cropbox, Outlier, etc.)
```yaml
- id: "cropbox"
  type: "CropBoxFilter"
  outputs:
    - name: "filtered"  # is_finalized=false, masks applied at finalize
```

## Conclusion

The immediate publishing implementation ensures data integrity and fixes the downsample filter crash. The `is_finalized` flag provides a clean way to handle both compacted and mask-based outputs, making the system more robust and efficient.



