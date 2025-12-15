# DAG Implementation - Next Steps

## Current Status

### ✅ Completed
1. **Design Documentation** - Complete redesign with proper DAG structure
2. **Core Data Structures** - Updated `DagNodeInput` and `DagNodeConfig`
3. **DAG Executor** - Updated to handle new input structure
4. **Filter Registry** - Basic infrastructure in place
5. **YAML Configuration Templates** - Three example configurations created:
   - `standard_preprocessing.yaml` - Replicates existing preprocessor
   - `multi_output_example.yaml` - Multiple outputs with parallel branches
   - `reordered_pipeline_example.yaml` - Downsample before crop
6. **CMakeLists.txt** - Cleaned up to match current file structure

### ❌ Removed/Deleted
- `preprocessor_filter.hpp/cpp` - Monolithic wrapper approach (incorrect)
- `filter_registrations.cpp` - Will be recreated with individual filters
- `cuda_pointcloud_preprocessor_dag_node.cpp` - Needs complete rewrite
- `standard_preprocessing.param.yaml` - Replaced with YAML format

## What Needs to be Implemented

### 1. YAML Parser (Priority: HIGH)
**File**: `src/dag/dag_config_parser.cpp`

**Tasks**:
```cpp
// Implement these functions in dag_config_parser.cpp:
DagConfig DagConfigParser::parseFromFile(const std::string & yaml_file_path);
DagConfig DagConfigParser::parseFromString(const std::string & yaml_content);
DagNodeConfig DagConfigParser::parseNode(const YAML::Node & node_yaml);
std::map<std::string, std::any> DagConfigParser::parseParameters(const YAML::Node & params_yaml);
```

**Key considerations**:
- Use yaml-cpp library
- Handle nested structures (crop_boxes array, etc.)
- Validate required fields
- Provide helpful error messages

### 2. Individual Filter Wrappers (Priority: HIGH)

Each filter needs to implement the `IFilter` interface:

#### **OrganizeFilter** (CRITICAL - First to implement)
- **Purpose**: Organize raw pointcloud into ring structure
- **Input**: `sensor_msgs::msg::PointCloud2` (raw)
- **Output**: `cuda_blackboard::CudaPointCloud2` (organized)
- **Implementation**: Wrap `CudaPointcloudPreprocessor::organizePointcloud()`
- **Parameters**: None (uses automatic detection)

#### **TransformFilter**
- **Purpose**: Transform pointcloud to target frame
- **Input**: `cuda_blackboard::CudaPointCloud2`
- **Output**: `cuda_blackboard::CudaPointCloud2` (transformed)
- **Parameters**: `target_frame` (string)
- **Implementation**: Use transform from tf_buffer

#### **CropBoxFilter**
- **Purpose**: Filter points outside crop boxes
- **Input**: `cuda_blackboard::CudaPointCloud2`
- **Output**: `cuda_blackboard::CudaPointCloud2` (cropped)
- **Parameters**: `crop_boxes` (array of min/max x/y/z)
- **Implementation**: Wrap crop box kernel from existing preprocessor

#### **DistortionCorrectionFilter**
- **Purpose**: Correct motion distortion
- **Input**: `cuda_blackboard::CudaPointCloud2`, twist queue, angular velocity queue
- **Output**: `cuda_blackboard::CudaPointCloud2` (undistorted)
- **Parameters**: `type` ("2D" or "3D"), `use_imu` (bool)
- **Implementation**: Wrap distortion correction kernel

#### **RingOutlierFilter**
- **Purpose**: Filter outliers based on ring structure
- **Input**: `cuda_blackboard::CudaPointCloud2`
- **Output**: `cuda_blackboard::CudaPointCloud2` (filtered)
- **Parameters**: `distance_ratio`, `object_length_threshold`, `enabled`
- **Implementation**: Wrap ring outlier kernel

#### **VoxelGridDownsampleFilter**
- **Purpose**: Downsample using voxel grid
- **Input**: `cuda_blackboard::CudaPointCloud2`
- **Output**: `cuda_blackboard::CudaPointCloud2` (downsampled)
- **Parameters**: `voxel_size_x`, `voxel_size_y`, `voxel_size_z`
- **Implementation**: Can reuse existing `CudaVoxelGridDownsampleFilter`

### 3. Filter Registration (Priority: HIGH)
**File**: `src/dag/filter_registrations.cpp`

```cpp
void registerAllFilters()
{
  auto & registry = getFilterRegistry();
  
  registry.registerFilter(
    "OrganizeFilter",
    []() { return std::make_unique<OrganizeFilter>(); },
    []() { return OrganizeFilter::getStaticMetadata(); });
  
  registry.registerFilter(
    "TransformFilter",
    []() { return std::make_unique<TransformFilter>(); },
    []() { return TransformFilter::getStaticMetadata(); });
  
  // ... register all other filters
}
```

### 4. DAG Node (Priority: MEDIUM)
**File**: `src/dag/cuda_pointcloud_preprocessor_dag_node.cpp`

**Key changes needed**:
```cpp
// Constructor should:
1. Parse YAML configuration file
2. Create DagConfig from parsed YAML
3. Build DAG using executor
4. Create dynamic subscribers based on DAG inputs
5. Create dynamic publishers based on DAG outputs

// Callback should:
1. Collect all required inputs
2. Execute DAG
3. Publish all outputs
```

**Implementation outline**:
```cpp
CudaPointcloudPreprocessorDagNode::CudaPointcloudPreprocessorDagNode(...)
{
  // Get YAML file path from parameter
  std::string yaml_file = declare_parameter<std::string>("dag_config_file");
  
  // Parse YAML
  auto dag_config = DagConfigParser::parseFromFile(yaml_file);
  
  // Build DAG
  executor_.buildDag(dag_config.nodes, context_);
  
  // Create subscribers for each input
  for (const auto & input : dag_config.inputs) {
    createSubscriber(input);
  }
  
  // Create publishers for each output
  for (const auto & output : dag_config.outputs) {
    createPublisher(output);
  }
}
```

### 5. Launch File Updates (Priority: LOW)
**File**: `launch/cuda_pointcloud_preprocessor_dag.launch.xml`

Update to pass YAML file path:
```xml
<arg name="dag_config_file" default="$(find-pkg-share autoware_cuda_pointcloud_preprocessor)/config/dag/standard_preprocessing.yaml"/>

<node pkg="autoware_cuda_pointcloud_preprocessor" exec="cuda_pointcloud_preprocessor_dag_node" name="cuda_pointcloud_preprocessor_dag" output="screen">
  <param name="dag_config_file" value="$(var dag_config_file)"/>
</node>
```

## Implementation Order

### Phase 1: Foundation (Start Here)
1. Implement YAML parser basic functions
2. Create OrganizeFilter as template
3. Register OrganizeFilter
4. Test parsing and filter creation

### Phase 2: Core Filters
1. Implement TransformFilter
2. Implement CropBoxFilter
3. Implement DistortionCorrectionFilter
4. Implement RingOutlierFilter
5. Register all filters

### Phase 3: Integration
1. Rewrite DAG node to use YAML parser
2. Update CMakeLists.txt to include DAG node
3. Test with simple DAG (organize → output)

### Phase 4: Validation
1. Test with standard_preprocessing.yaml
2. Test with multi_output_example.yaml
3. Compare output with existing preprocessor
4. Performance benchmarking

## Code Template: OrganizeFilter

```cpp
// organize_filter.hpp
class OrganizeFilter : public IFilter
{
public:
  OrganizeFilter();
  void initialize(const std::map<std::string, std::any> & params) override;
  void process(
    const std::map<std::string, std::shared_ptr<void>> & inputs,
    std::map<std::string, std::shared_ptr<void>> & outputs,
    FilterContext & context) override;
  FilterMetadata getMetadata() const override;
  bool validateInputs(
    const std::map<std::string, std::shared_ptr<void>> & inputs) const override;
  
  static FilterMetadata getStaticMetadata();

private:
  // Internal state for organizing
  thrust::device_vector<InputPointType> device_input_points_;
  thrust::device_vector<InputPointType> device_organized_points_;
  // ... other buffers
};

// organize_filter.cpp
void OrganizeFilter::process(
  const std::map<std::string, std::shared_ptr<void>> & inputs,
  std::map<std::string, std::shared_ptr<void>> & outputs,
  FilterContext & context)
{
  // 1. Get input pointcloud
  auto pointcloud = std::static_pointer_cast<sensor_msgs::msg::PointCloud2>(
    inputs.at("pointcloud"));
  
  // 2. Copy to device
  // 3. Call organize kernels
  // 4. Create output
  // 5. Return organized pointcloud
  
  outputs["organized"] = output;
}
```

## Dependencies

- **yaml-cpp**: Already a common ROS2 dependency
- **Existing kernels**: Reuse from `CudaPointcloudPreprocessor`
- **CUDA streams**: Share stream from FilterContext

## Testing Strategy

1. **Unit tests**: Test each filter individually
2. **Integration tests**: Test DAG execution
3. **Regression tests**: Compare with existing preprocessor
4. **Performance tests**: Benchmark execution time

## Notes

- The DAG executor is complete and handles complex graphs
- Filter registry infrastructure is in place
- YAML configuration format is defined and documented
- Next critical step: Implement YAML parser and at least one filter

