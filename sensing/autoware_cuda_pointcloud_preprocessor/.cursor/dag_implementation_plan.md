# DAG Implementation Plan - Corrected Approach

## Current Status

I've rewritten the design documentation to properly reflect user-defined DAG structures. The key changes:

1. **YAML defines complete DAG structure** - Users specify nodes, connections, and execution order
2. **Multiple filter instances** - Same filter type can be used multiple times with different parameters
3. **Multiple outputs** - Different outputs can be published to different topics
4. **Flexible ordering** - Users can reorder operations (e.g., downsample before cropbox)

## What Needs to be Done

### 1. Update DAG Executor (dag_executor.cpp)
- Update `resolveInputs()` to handle new `DagNodeInput` structure
- Update `topologicalSort()` to work with new input structure
- Update `hasCycle()` to work with new input structure

### 2. Implement YAML Parser (dag_config_parser.cpp)
- Parse YAML file/string/parameters
- Convert YAML nodes to `DagNodeConfig` structures
- Handle parameter parsing (nested structures, arrays, etc.)

### 3. Create Individual Filter Wrappers
Each filter needs to be a separate implementation:
- `OrganizeFilter` - Wraps organize functionality
- `TransformFilter` - Wraps transform functionality  
- `CropBoxFilter` - Wraps crop box functionality
- `DistortionCorrectionFilter` - Wraps distortion correction
- `RingOutlierFilter` - Wraps ring outlier filtering
- `VoxelGridDownsampleFilter` - Wraps voxel downsample (already exists as separate node)

### 4. Update DAG Node
- Parse YAML configuration
- Create dynamic subscribers based on DAG inputs
- Create dynamic publishers based on DAG outputs
- Execute DAG on incoming messages

### 5. Create Template YAML Configuration
- YAML file that replicates existing preprocessor functionality
- Example with multiple outputs

## Implementation Order

1. ✅ Rewrite design documentation
2. ✅ Delete incorrect PreprocessorFilter wrapper
3. Update DAG executor to handle new input structure
4. Implement YAML parser
5. Create at least one filter wrapper (OrganizeFilter) as template
6. Update DAG node to use YAML parser
7. Create template YAML configuration
8. Test with simple DAG

## Key Design Decisions

- **DagNodeInput structure**: Unified structure for both external inputs and node outputs
- **YAML-first approach**: All configuration comes from YAML, not ROS parameters
- **Individual filters**: Each processing step is a separate filter, not a monolithic wrapper
- **Dynamic publishers**: Publishers created based on DAG output definitions

