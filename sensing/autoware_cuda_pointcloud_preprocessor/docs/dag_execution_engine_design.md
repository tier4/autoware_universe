# DAG-Based CUDA Pointcloud Preprocessor Design Document

## Overview

This document describes the design of a Directed Acyclic Graph (DAG) based execution engine for CUDA pointcloud preprocessing. The design enables users to configure custom processing pipelines by defining the complete DAG structure in YAML configuration files, allowing flexible composition of existing CUDA filters and kernels.

## Motivation

The current `cuda_pointcloud_preprocessor` node implements a fixed workflow:
1. Organize pointcloud
2. Transform to base frame
3. Crop box filtering
4. Distortion correction (2D/3D)
5. Ring outlier filtering
6. Output pointcloud

This fixed workflow limits flexibility. Users may want to:
- **Reorder processing steps**: e.g., downsample before cropbox or cropbox before downsample
- **Use multiple instances**: e.g., downsample with two different voxel sizes to produce two outputs
- **Create parallel pipelines**: e.g., process the same input through different filter chains
- **Skip certain filters**: e.g., skip distortion correction for certain use cases
- **Conditionally apply filters**: based on input characteristics

## Design Goals

1. **Flexibility**: Users define the complete DAG structure in YAML, including node connections and execution order
2. **Reusability**: Existing filter implementations are reused without modification
3. **Extensibility**: New filters can be added with minimal changes
4. **Performance**: Maintain zero-copy CUDA memory transfers where possible
5. **Compatibility**: Support existing ROS2 message types and interfaces
6. **Type Safety**: Compile-time and runtime validation of DAG configurations

## Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              DAG Execution Engine Node                       │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         YAML Configuration Parser                     │  │
│  │  - Parse complete DAG structure from YAML             │  │
│  │  - Validate DAG structure (cycles, dependencies)     │  │
│  │  - Build execution graph                              │  │
│  │  - Create dynamic subscribers/publishers             │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         DAG Executor                                  │  │
│  │  - Topological sort for execution order               │  │
│  │  - Dependency resolution                              │  │
│  │  - Memory management                                  │  │
│  │  - Error handling and recovery                        │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Filter Registry                               │  │
│  │  - Filter factory functions                           │  │
│  │  - Filter metadata (inputs, outputs, parameters)      │  │
│  │  - Filter lifecycle management                       │  │
│  └──────────────────────────────────────────────────────┘  │
│                          │                                  │
│                          ▼                                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Individual Filter Implementations             │  │
│  │  - OrganizeFilter                                    │  │
│  │  - TransformFilter                                  │  │
│  │  - CropBoxFilter                                     │  │
│  │  - DistortionCorrectionFilter (2D/3D)                │  │
│  │  - RingOutlierFilter                                 │  │
│  │  - VoxelGridDownsampleFilter                         │  │
│  │  - PolarVoxelOutlierFilter                           │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## DAG Configuration Format

The DAG structure is defined entirely in YAML. Users specify:
- Which filters to use
- In what order (via node connections)
- With what parameters
- What outputs to publish

### Complete YAML Structure

```yaml
dag:
  name: "pointcloud_preprocessing_pipeline"
  version: "1.0"
  
  # External inputs (ROS topics)
  inputs:
    - name: "pointcloud"
      type: "sensor_msgs::msg::PointCloud2"
      topic: "~/input/pointcloud"
    - name: "twist"
      type: "geometry_msgs::msg::TwistWithCovarianceStamped"
      topic: "~/input/twist"
    - name: "imu"
      type: "sensor_msgs::msg::Imu"
      topic: "~/input/imu"
      optional: true
  
  # Processing nodes (filters)
  nodes:
    # Node 1: Organize pointcloud
    - id: "organize_1"
      type: "OrganizeFilter"
      inputs:
        - source: "pointcloud"  # External input
      outputs:
        - name: "organized"
      parameters: {}
    
    # Node 2: Transform to base frame
    - id: "transform_1"
      type: "TransformFilter"
      inputs:
        - source: "organized"
          from_node: "organize_1"  # Input from another node
      outputs:
        - name: "transformed"
      parameters:
        target_frame: "base_link"
    
    # Node 3: Crop box filtering
    - id: "cropbox_1"
      type: "CropBoxFilter"
      inputs:
        - source: "transformed"
          from_node: "transform_1"
      outputs:
        - name: "cropped"
      parameters:
        crop_boxes:
          - min_x: 0.0
            max_x: 100.0
            min_y: -10.0
            max_y: 10.0
            min_z: -2.0
            max_z: 2.0
    
    # Node 4: First downsample (fine)
    - id: "downsample_fine"
      type: "VoxelGridDownsampleFilter"
      inputs:
        - source: "cropped"
          from_node: "cropbox_1"
      outputs:
        - name: "downsampled_fine"
      parameters:
        voxel_size_x: 0.1
        voxel_size_y: 0.1
        voxel_size_z: 0.1
    
    # Node 5: Second downsample (coarse) - parallel branch
    - id: "downsample_coarse"
      type: "VoxelGridDownsampleFilter"
      inputs:
        - source: "cropped"
          from_node: "cropbox_1"  # Same input as downsample_fine
      outputs:
        - name: "downsampled_coarse"
      parameters:
        voxel_size_x: 0.2
        voxel_size_y: 0.2
        voxel_size_z: 0.2
    
    # Node 6: Distortion correction (only on fine branch)
    - id: "distortion_1"
      type: "DistortionCorrectionFilter"
      inputs:
        - source: "downsampled_fine"
          from_node: "downsample_fine"
        - source: "twist"  # External input
        - source: "imu"    # External input (optional)
          optional: true
      outputs:
        - name: "undistorted"
      parameters:
        type: "2D"  # or "3D"
        use_imu: true
    
    # Node 7: Ring outlier filter (fine branch)
    - id: "ring_outlier_fine"
      type: "RingOutlierFilter"
      inputs:
        - source: "undistorted"
          from_node: "distortion_1"
      outputs:
        - name: "filtered_fine"
      parameters:
        distance_ratio: 1.03
        object_length_threshold: 0.05
        enabled: true
    
    # Node 8: Ring outlier filter (coarse branch)
    - id: "ring_outlier_coarse"
      type: "RingOutlierFilter"
      inputs:
        - source: "downsampled_coarse"
          from_node: "downsample_coarse"
      outputs:
        - name: "filtered_coarse"
      parameters:
        distance_ratio: 1.05
        object_length_threshold: 0.1
        enabled: true
  
  # Output definitions (what to publish)
  outputs:
    - name: "preprocessed_fine"
      source: "filtered_fine"
      from_node: "ring_outlier_fine"
      topic: "~/output/pointcloud_fine"
      type: "cuda_blackboard::CudaPointCloud2"
    
    - name: "preprocessed_coarse"
      source: "filtered_coarse"
      from_node: "ring_outlier_coarse"
      topic: "~/output/pointcloud_coarse"
      type: "cuda_blackboard::CudaPointCloud2"
```

### Key Features of the Configuration

1. **Node IDs**: Each node has a unique `id` that can be referenced by other nodes
2. **Input Sources**: 
   - External inputs: `source: "pointcloud"` (from DAG inputs)
   - Node outputs: `source: "organized", from_node: "organize_1"` (from another node)
3. **Multiple Instances**: Same filter type can be used multiple times with different IDs and parameters
4. **Parallel Branches**: Multiple nodes can consume the same output (e.g., `downsample_fine` and `downsample_coarse` both use `cropped`)
5. **Output Publishing**: Multiple outputs can be published to different topics

### Example: Reordered Pipeline

Users can reorder operations:

```yaml
nodes:
  - id: "organize_1"
    type: "OrganizeFilter"
    inputs:
      - source: "pointcloud"
    outputs:
      - name: "organized"
  
  - id: "downsample_1"
    type: "VoxelGridDownsampleFilter"
    inputs:
      - source: "organized"
        from_node: "organize_1"
    outputs:
      - name: "downsampled"
    parameters:
      voxel_size_x: 0.1
  
  - id: "cropbox_1"
    type: "CropBoxFilter"
    inputs:
      - source: "downsampled"
        from_node: "downsample_1"  # Crop AFTER downsample
    outputs:
      - name: "cropped"
```

## Core Components

### 1. Filter Interface

All filters implement a common interface:

```cpp
class IFilter
{
public:
  virtual ~IFilter() = default;
  
  // Initialize with parameters
  virtual void initialize(const std::map<std::string, std::any> & params) = 0;
  
  // Process inputs and produce outputs
  virtual void process(
    const std::map<std::string, std::shared_ptr<void>> & inputs,
    std::map<std::string, std::shared_ptr<void>> & outputs,
    FilterContext & context) = 0;
  
  // Get filter metadata
  virtual FilterMetadata getMetadata() const = 0;
  
  // Validate inputs
  virtual bool validateInputs(
    const std::map<std::string, std::shared_ptr<void>> & inputs) const = 0;
};
```

### 2. Individual Filter Implementations

Each processing step is a separate filter:

- **OrganizeFilter**: Organizes raw pointcloud into ring structure
- **TransformFilter**: Transforms pointcloud to target frame
- **CropBoxFilter**: Filters points outside crop boxes
- **DistortionCorrectionFilter**: Corrects motion distortion (2D/3D)
- **RingOutlierFilter**: Filters outliers based on ring structure
- **VoxelGridDownsampleFilter**: Downsamples using voxel grid
- **PolarVoxelOutlierFilter**: Polar voxel-based outlier filtering

### 3. DAG Executor

The executor:
1. Parses YAML configuration
2. Validates DAG structure (no cycles, valid dependencies)
3. Performs topological sort to determine execution order
4. Resolves dependencies for each node
5. Executes nodes in order
6. Manages intermediate results

### 4. YAML Parser

The parser:
1. Reads YAML file
2. Validates structure
3. Creates `DagNodeConfig` objects for each node
4. Builds dependency graph
5. Creates dynamic subscribers/publishers based on inputs/outputs

## Implementation Details

### Node Execution Flow

1. **Parse YAML** → Create node configurations
2. **Validate DAG** → Check for cycles, missing dependencies
3. **Topological Sort** → Determine execution order
4. **For each node in order**:
   - Resolve inputs (from external sources or previous nodes)
   - Validate inputs
   - Execute filter
   - Store outputs
5. **Publish outputs** → Send to configured topics

### Memory Management

- Shared CUDA stream and memory pool across all filters
- Intermediate results stored in `node_outputs_` map
- Zero-copy transfers when possible (CUDA device memory)
- Automatic cleanup after execution

### Error Handling

- DAG validation at initialization
- Input validation before each filter execution
- Exception handling with proper error messages
- Graceful degradation for optional inputs

## Example Configurations

### Example 1: Replicating Current Preprocessor

```yaml
dag:
  inputs:
    - name: "pointcloud"
      topic: "~/input/pointcloud"
    - name: "twist"
      topic: "~/input/twist"
    - name: "imu"
      topic: "~/input/imu"
      optional: true
  
  nodes:
    - id: "organize"
      type: "OrganizeFilter"
      inputs:
        - source: "pointcloud"
      outputs:
        - name: "organized"
    
    - id: "transform"
      type: "TransformFilter"
      inputs:
        - source: "organized"
          from_node: "organize"
      outputs:
        - name: "transformed"
      parameters:
        target_frame: "base_link"
    
    - id: "cropbox"
      type: "CropBoxFilter"
      inputs:
        - source: "transformed"
          from_node: "transform"
      outputs:
        - name: "cropped"
      parameters:
        crop_boxes: [...]
    
    - id: "distortion"
      type: "DistortionCorrectionFilter"
      inputs:
        - source: "cropped"
          from_node: "cropbox"
        - source: "twist"
        - source: "imu"
          optional: true
      outputs:
        - name: "undistorted"
      parameters:
        type: "2D"
        use_imu: true
    
    - id: "ring_outlier"
      type: "RingOutlierFilter"
      inputs:
        - source: "undistorted"
          from_node: "distortion"
      outputs:
        - name: "filtered"
      parameters:
        distance_ratio: 1.03
        object_length_threshold: 0.05
  
  outputs:
    - name: "output"
      source: "filtered"
      from_node: "ring_outlier"
      topic: "~/output/pointcloud"
```

### Example 2: Multiple Downsample Outputs

```yaml
nodes:
  - id: "organize"
    type: "OrganizeFilter"
    inputs:
      - source: "pointcloud"
    outputs:
      - name: "organized"
  
  - id: "cropbox"
    type: "CropBoxFilter"
    inputs:
      - source: "organized"
        from_node: "organize"
    outputs:
      - name: "cropped"
  
  - id: "downsample_0.1"
    type: "VoxelGridDownsampleFilter"
    inputs:
      - source: "cropped"
        from_node: "cropbox"
    outputs:
      - name: "downsampled_0.1"
    parameters:
      voxel_size_x: 0.1
      voxel_size_y: 0.1
      voxel_size_z: 0.1
  
  - id: "downsample_0.2"
    type: "VoxelGridDownsampleFilter"
    inputs:
      - source: "cropped"
        from_node: "cropbox"
    outputs:
      - name: "downsampled_0.2"
    parameters:
      voxel_size_x: 0.2
      voxel_size_y: 0.2
      voxel_size_z: 0.2
  
  - id: "downsample_0.5"
    type: "VoxelGridDownsampleFilter"
    inputs:
      - source: "cropped"
        from_node: "cropbox"
    outputs:
      - name: "downsampled_0.5"
    parameters:
      voxel_size_x: 0.5
      voxel_size_y: 0.5
      voxel_size_z: 0.5

outputs:
  - name: "output_0.1"
    source: "downsampled_0.1"
    from_node: "downsample_0.1"
    topic: "~/output/pointcloud_0.1"
  - name: "output_0.2"
    source: "downsampled_0.2"
    from_node: "downsample_0.2"
    topic: "~/output/pointcloud_0.2"
  - name: "output_0.5"
    source: "downsampled_0.5"
    from_node: "downsample_0.5"
    topic: "~/output/pointcloud_0.5"
```

## File Structure

```
sensing/autoware_cuda_pointcloud_preprocessor/
├── include/autoware/cuda_pointcloud_preprocessor/
│   └── dag/
│       ├── dag_executor.hpp
│       ├── dag_config_parser.hpp
│       ├── filter_interface.hpp
│       ├── filter_registry.hpp
│       ├── filter_context.hpp
│       ├── processing_state.hpp
│       ├── filters/
│       │   ├── transform_filter.hpp
│       │   ├── cropbox_filter.hpp
│       │   ├── distortion_filter.hpp
│       │   ├── ring_outlier_filter.hpp
│       │   └── downsample_filter.hpp
│       └── cuda_pointcloud_preprocessor_dag_node.hpp
├── src/
│   └── dag/
│       ├── dag_executor.cpp
│       ├── dag_config_parser.cpp
│       ├── filter_registry.cpp
│       ├── filter_registrations.cpp
│       ├── filters/
│       │   ├── transform_filter.cpp
│       │   ├── cropbox_filter.cpp
│       │   ├── distortion_filter.cpp
│       │   ├── ring_outlier_filter.cpp
│       │   └── downsample_filter.cpp
│       └── cuda_pointcloud_preprocessor_dag_node.cpp
NOTE: OrganizeFilter and FinalizeFilter are INTERNALIZED in the DAG node
      (automatic at entry/exit, not user-configurable)
├── config/
│   └── dag/
│       ├── standard_preprocessing.yaml
│       └── multi_output_example.yaml
└── schema/
    └── dag_config.schema.json
```

## Migration Path

1. **Phase 1**: Implement core DAG infrastructure and YAML parser
2. **Phase 2**: Implement individual filter wrappers
3. **Phase 3**: Create template configuration matching existing preprocessor
4. **Phase 4**: Test and validate against existing implementation
5. **Phase 5**: Add additional filters and examples
6. **Phase 6**: Documentation and user guides

## References

- Existing filter implementations in `autoware_cuda_pointcloud_preprocessor`
- CUDA best practices document: `.cursor/cuda_pointcloud_best_practices.md`
- ROS2 component architecture
- YAML parsing libraries (yaml-cpp)
