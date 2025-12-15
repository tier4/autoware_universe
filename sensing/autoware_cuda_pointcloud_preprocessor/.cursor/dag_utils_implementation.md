# DAG Utils: Shared Pointer to Unique Pointer Conversion

## Context

Following the existing code patterns in `cuda_polar_voxel_outlier_filter.cu`, we identified a standard pattern used throughout the CUDA pointcloud preprocessor:

1. **Internal Processing**: Use `shared_ptr` for flexibility and data sharing
2. **Publishing**: Convert to `unique_ptr` for ownership transfer to ROS2

## Implementation

### Pattern from Existing Code

From `cuda_polar_voxel_outlier_filter.cu`:

```cpp
// Input: line 539, 914
const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud

// Output creation: lines 916-928
std::unique_ptr<cuda_blackboard::CudaPointCloud2> & output_cloud
{
  output_cloud->header = input_cloud->header;
  // ... copy metadata ...
  output_cloud->data = cuda_blackboard::make_unique<std::uint8_t[]>(output_cloud->row_step);
  // copy_valid_points_kernel or cudaMemcpy
}
```

### New Utility Function

Created `dag_utils.hpp` with `convert_shared_to_unique_for_publishing()`:

**Purpose**: Convert `shared_ptr<CudaPointCloud2>` to `unique_ptr<CudaPointCloud2>` for publishing

**Strategy**:
1. Create new `unique_ptr<CudaPointCloud2>`
2. Copy all metadata fields
3. Allocate new CUDA memory buffer using `cuda_blackboard::make_unique<std::uint8_t[]>()`
4. Copy data Device-to-Device using `cudaMemcpy()`

**Rationale**:
- Follows the exact pattern used in `cuda_polar_voxel_outlier_filter.cu:913-937`
- Ensures proper CUDA memory management
- Clean ownership transfer for publishing
- Compatible with `cuda_blackboard::CudaBlackboardPublisher` expectations

## Usage in DAG Node

```cpp
// DAG executor returns shared_ptr for internal use
auto outputs = executor_.execute(external_inputs);

// Convert to unique_ptr for publishing
auto output_shared = std::static_pointer_cast<CudaPointCloud2>(outputs["finalize.output"]);
auto output_unique = convert_shared_to_unique_for_publishing(output_shared);
pub_->publish(std::move(output_unique));
```

## Benefits

1. **Consistency**: Aligns with existing CUDA filter patterns across the codebase
2. **Flexibility**: DAG can use `shared_ptr` internally for multiple consumers
3. **Safety**: Proper CUDA memory management and ownership semantics
4. **Reusability**: Common utility function for all DAG outputs

## Files Modified

- `include/autoware/cuda_pointcloud_preprocessor/dag/dag_utils.hpp` (new)
- `src/dag/cuda_pointcloud_preprocessor_dag_node.cpp` (updated publishing logic)

## Testing

All tests passing:
- `test_filter_registry`: 5/5 tests passed ✓
- `test_dag_executor`: 5/5 tests passed ✓

## Future Considerations

If performance profiling shows the memory copy is a bottleneck, we could explore:
1. Custom deleter approach to avoid copy (complex lifetime management)
2. Move semantics if cuda_blackboard supports it
3. Direct unique_ptr in DAG (loses flexibility for multi-output scenarios)

For now, following the existing proven pattern is the safest approach.

