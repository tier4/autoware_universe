# DAG Implementation Status - Corrected Approach

## Summary

The design and core infrastructure have been corrected to properly support user-defined DAG structures in YAML configuration. The previous implementation incorrectly used a monolithic wrapper approach; the new design supports individual filter components that can be composed in arbitrary DAG structures.

## What Has Been Corrected

### 1. Design Documentation ✅
- **File**: `docs/dag_execution_engine_design.md`
- **Changes**: Completely rewritten to reflect:
  - YAML defines complete DAG structure (nodes, connections, execution order)
  - Support for multiple filter instances with different parameters
  - Support for multiple outputs to different topics
  - Flexible ordering of operations
  - Examples showing complex DAG configurations

### 2. Core Data Structures ✅
- **File**: `include/autoware/cuda_pointcloud_preprocessor/dag/dag_executor.hpp`
- **Changes**:
  - Introduced `DagNodeInput` structure to unify external inputs and node outputs
  - Updated `DagNodeConfig` to use `DagNodeInput` instead of separate fields
  - Better representation of node connections

### 3. DAG Executor ✅
- **File**: `src/dag/dag_executor.cpp`
- **Changes**:
  - Updated `resolveInputs()` to handle new `DagNodeInput` structure
  - Updated `topologicalSort()` to work with unified input structure
  - Updated `hasCycle()` and `hasCycleDFS()` for cycle detection
  - Updated `validateDag()` to check dependencies correctly

### 4. Removed Incorrect Code ✅
- Deleted `preprocessor_filter.hpp/cpp` (monolithic wrapper approach)
- Deleted `filter_registrations.cpp` (will be recreated with individual filters)

## What Remains to be Implemented

### 1. YAML Parser (In Progress)
- **File**: `include/autoware/cuda_pointcloud_preprocessor/dag/dag_config_parser.hpp` (created)
- **File**: `src/dag/dag_config_parser.cpp` (needs implementation)
- **Tasks**:
  - Parse YAML file/string/ROS2 parameters
  - Convert YAML nodes to `DagNodeConfig` structures
  - Handle nested parameter structures
  - Validate YAML structure

### 2. Individual Filter Wrappers (Pending)
Each processing step needs to be a separate filter:

- **OrganizeFilter**: Organizes raw pointcloud into ring structure
  - Wraps `organizePointcloud()` functionality from `CudaPointcloudPreprocessor`
  - Input: `sensor_msgs::msg::PointCloud2`
  - Output: `cuda_blackboard::CudaPointCloud2` (organized)

- **TransformFilter**: Transforms pointcloud to target frame
  - Wraps transform functionality
  - Input: `cuda_blackboard::CudaPointCloud2`, transform parameters
  - Output: `cuda_blackboard::CudaPointCloud2` (transformed)

- **CropBoxFilter**: Filters points outside crop boxes
  - Wraps crop box functionality
  - Input: `cuda_blackboard::CudaPointCloud2`, crop box parameters
  - Output: `cuda_blackboard::CudaPointCloud2` (cropped)

- **DistortionCorrectionFilter**: Corrects motion distortion
  - Wraps distortion correction (2D/3D)
  - Input: `cuda_blackboard::CudaPointCloud2`, twist queue, angular velocity queue
  - Output: `cuda_blackboard::CudaPointCloud2` (undistorted)

- **RingOutlierFilter**: Filters outliers based on ring structure
  - Wraps ring outlier filtering
  - Input: `cuda_blackboard::CudaPointCloud2`, filter parameters
  - Output: `cuda_blackboard::CudaPointCloud2` (filtered)

- **VoxelGridDownsampleFilter**: Downsamples using voxel grid
  - Can reuse existing `CudaVoxelGridDownsampleFilter` class
  - Input: `cuda_blackboard::CudaPointCloud2`, voxel size
  - Output: `cuda_blackboard::CudaPointCloud2` (downsampled)

### 3. Filter Registration (Pending)
- **File**: `src/dag/filter_registrations.cpp` (needs recreation)
- **Tasks**:
  - Register all individual filters
  - Provide factory functions
  - Provide metadata getters

### 4. DAG Node Updates (Pending)
- **File**: `src/dag/cuda_pointcloud_preprocessor_dag_node.cpp`
- **Tasks**:
  - Remove parameter-based configuration
  - Add YAML file parsing
  - Create dynamic subscribers based on DAG inputs
  - Create dynamic publishers based on DAG outputs
  - Handle multiple outputs correctly

### 5. Template YAML Configuration (Pending)
- **File**: `config/dag/standard_preprocessing.yaml`
- **Tasks**:
  - Create YAML file that replicates existing preprocessor
  - Example with multiple outputs
  - Documentation comments

## Implementation Strategy

### Phase 1: Core Infrastructure (Current)
- ✅ Design documentation
- ✅ Core data structures
- ✅ DAG executor updates
- ⏳ YAML parser (basic implementation)

### Phase 2: Filter Implementation
- Create at least one filter wrapper (OrganizeFilter) as template
- Implement remaining filters one by one
- Register all filters

### Phase 3: Integration
- Update DAG node to use YAML parser
- Create template YAML configuration
- Test with simple DAG

### Phase 4: Validation
- Test against existing preprocessor
- Test with complex DAGs (multiple outputs, parallel branches)
- Performance benchmarking

## Key Design Principles

1. **YAML-First**: All configuration comes from YAML, not ROS parameters
2. **Individual Filters**: Each processing step is a separate, composable filter
3. **Flexible Composition**: Users can create arbitrary DAG structures
4. **Multiple Instances**: Same filter type can be used multiple times
5. **Multiple Outputs**: Different outputs can be published to different topics

## Example: What Users Can Do

### Reorder Operations
```yaml
nodes:
  - id: "organize"
    type: "OrganizeFilter"
    ...
  - id: "downsample"
    type: "VoxelGridDownsampleFilter"
    inputs:
      - source: "organized"
        from_node: "organize"
    ...
  - id: "cropbox"
    type: "CropBoxFilter"
    inputs:
      - source: "downsampled"
        from_node: "downsample"  # Crop AFTER downsample
    ...
```

### Multiple Instances
```yaml
nodes:
  - id: "downsample_0.1"
    type: "VoxelGridDownsampleFilter"
    parameters:
      voxel_size_x: 0.1
    ...
  - id: "downsample_0.2"
    type: "VoxelGridDownsampleFilter"
    parameters:
      voxel_size_x: 0.2
    ...
```

### Multiple Outputs
```yaml
outputs:
  - name: "output_1"
    source: "filtered_1"
    from_node: "filter_1"
    topic: "~/output/pointcloud_1"
  - name: "output_2"
    source: "filtered_2"
    from_node: "filter_2"
    topic: "~/output/pointcloud_2"
```

## Next Steps

1. Implement basic YAML parser (parseFromFile, parseFromString)
2. Create OrganizeFilter as first filter wrapper
3. Register OrganizeFilter
4. Update DAG node to use YAML parser
5. Create minimal YAML configuration for testing
6. Test with simple DAG (organize → output)

