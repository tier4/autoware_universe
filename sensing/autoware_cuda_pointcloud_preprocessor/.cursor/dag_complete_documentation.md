# CUDA Pointcloud Preprocessor - DAG System Complete Documentation

**Version**: 2.0  
**Date**: 2025-12-10  
**Author**: DAG Execution Engine Development Team

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture](#architecture)
3. [Core Components](#core-components)
4. [Filter Interface](#filter-interface)
5. [DAG Configuration](#dag-configuration)
6. [DAG Executor](#dag-executor)
7. [Type System](#type-system)
8. [Dynamic Features](#dynamic-features)
9. [Testing Strategy](#testing-strategy)
10. [Implementation Guidelines](#implementation-guidelines)

---

## 1. System Overview

### Purpose

The DAG (Directed Acyclic Graph) execution engine allows users to define custom pointcloud preprocessing pipelines through YAML configuration, without modifying code. The system executes CUDA-accelerated filters in a dependency-aware order.

### Key Features

- **Configuration-Driven**: Complete pipeline definition in YAML
- **Zero-Copy GPU Processing**: Uses `cuda_blackboard` for GPU-to-GPU data transfer
- **Dynamic I/O**: Subscribers and publishers created based on DAG configuration
- **Typed Inputs**: DAG executor handles type identification and casting
- **Dynamic Output Names**: Filters use output names from YAML configuration
- **Shared Preprocessor Pattern**: Filters delegate to existing CUDA implementations
- **Validation**: Comprehensive parameter validation at parse time
- **Cycle Detection**: Automatic detection of circular dependencies

---

## 2. Architecture

### System Layers

```
┌─────────────────────────────────────────────────────────────┐
│  ROS2 Node Layer (cuda_pointcloud_preprocessor_dag_node)   │
│  - Dynamic subscribers/publishers based on DAG config       │
│  - ROS2 parameter handling                                   │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│  DAG Executor Layer                                          │
│  - Topological sort & execution order                       │
│  - Dependency resolution                                     │
│  - Type identification & casting                            │
│  - Cycle detection                                          │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│  Filter Layer                                                │
│  - Individual filter implementations                        │
│  - Parameter management per filter                          │
│  - Input validation                                         │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│  CUDA Kernel Layer (Shared Preprocessor)                    │
│  - organize_kernels.cu                                       │
│  - transform_kernels.cu                                      │
│  - cropbox_kernels.cu                                        │
│  - undistort_kernels.cu                                      │
│  - outlier_kernels.cu                                        │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

```
External Input (ROS2 Topic)
  ↓
DAG Executor (resolveInputs)
  ↓
Type Identification (prepareTypedInputs)
  ↓
Filter 1 (process with TypedInputs)
  ↓
Intermediate Storage (node_outputs_)
  ↓
Filter 2 (process with TypedInputs)
  ↓
...
  ↓
DAG Output (published to ROS2 Topics)
```

---

## 3. Core Components

### 3.1 DagConfigParser

**File**: `dag_config_parser.hpp/cpp`

**Responsibilities**:
- Parse YAML configuration files
- Parse ROS2 parameters
- Validate filter-specific parameters
- Type inference for parameters

**Key Methods**:

```cpp
static DagConfig parseFromFile(const std::string & yaml_file_path);
static DagConfig parseFromString(const std::string & yaml_content);
static DagConfig parseFromParameters(rclcpp::Node * node);
```

**Parameter Validation**:
- `TransformFilter`: Requires `target_frame` (string)
- `CropBoxFilter`: Requires `crop_boxes` (array of maps)
- `DistortionFilter`: Requires `use_3d` (bool), `use_imu` (bool)
- `RingOutlierFilter`: Requires `distance_ratio` (double), `object_length_threshold` (double), `enabled` (bool)

---

### 3.2 DagExecutor

**File**: `dag_executor.hpp/cpp`

**Responsibilities**:
- Build DAG from configuration
- Topological sorting for execution order
- Dependency resolution
- Type identification and casting
- Cycle detection
- Execute filters in correct order

**Key Methods**:

```cpp
void buildDag(const std::vector<DagNodeConfig> & node_configs, const FilterContext & context);
std::map<std::string, std::shared_ptr<void>> execute(
  const std::map<std::string, std::shared_ptr<void>> & inputs);
bool validateDag() const;
std::vector<std::size_t> getExecutionOrder() const;
```

**Internal Algorithms**:
- **Topological Sort**: Kahn's algorithm for determining execution order
- **Cycle Detection**: DFS-based algorithm with recursion stack
- **Dependency Resolution**: Maps inputs from external sources or upstream nodes

---

### 3.3 FilterRegistry

**File**: `filter_registry.hpp/cpp`

**Responsibilities**:
- Singleton registry for all filter types
- Factory pattern for filter creation
- Metadata storage and retrieval

**Key Methods**:

```cpp
template<typename FilterType>
void registerFilterType(const std::string & type_name);

std::unique_ptr<IFilter> createFilter(const std::string & type_name);
bool isRegistered(const std::string & type_name) const;
FilterMetadata getMetadata(const std::string & type_name) const;
```

**Registered Filters**:
- `OrganizeFilter`: Organize unstructured pointcloud into structured format
- `TransformFilter`: Transform pointcloud to target frame
- `CropBoxFilter`: Apply crop box masks
- `DistortionFilter`: Correct motion distortion
- `RingOutlierFilter`: Filter ring-based outliers
- `FinalizeFilter`: Apply accumulated masks and finalize output

---

### 3.4 CudaPointcloudPreprocessor (Shared)

**File**: `cuda_pointcloud_preprocessor.hpp/cu`

**Role**: Provides stateless CUDA processing functions that all filters can reuse

**Key Design**:
- **Stateless Methods**: All parameters passed as function arguments
- **Shared Instance**: Single instance passed through `FilterContext`
- **No Filter-Specific State**: Filters manage their own parameters

**Public Methods**:

```cpp
cuda_blackboard::CudaPointCloud2 organizePointcloudPublic(
  const sensor_msgs::msg::PointCloud2 & input);

cuda_blackboard::CudaPointCloud2 organizePointcloudPublic(
  const cuda_blackboard::CudaPointCloud2 & input);  // GPU-to-GPU

void applyCropBoxPublic(
  const cuda_blackboard::CudaPointCloud2 & input,
  const std::vector<std::map<std::string, double>> & crop_boxes);

void correctDistortionPublic(
  const cuda_blackboard::CudaPointCloud2 & input,
  const std::string & undistortion_type,
  bool use_imu);

void applyRingOutlierFilterPublic(
  const cuda_blackboard::CudaPointCloud2 & input,
  const RingOutlierFilterParams & params,
  bool enabled);

cuda_blackboard::CudaPointCloud2 finalizeOutputPublic();
```

---

## 4. Filter Interface

### 4.1 IFilter Base Class

**File**: `filter_interface.hpp`

All filters must implement this interface:

```cpp
class IFilter
{
public:
  virtual ~IFilter() = default;

  // Initialize with parameters from YAML
  virtual void initialize(const std::map<std::string, std::any> & params) = 0;

  // Process input data (TypedInputs provided by DAG executor)
  virtual void process(
    const TypedInputs & inputs,
    std::map<std::string, std::shared_ptr<void>> & outputs,
    FilterContext & context,
    const std::vector<std::string> & output_names) = 0;

  // Get filter metadata for introspection
  virtual FilterMetadata getMetadata() const = 0;

  // Validate inputs before processing
  virtual bool validateInputs(const TypedInputs & inputs) const = 0;
};
```

### 4.2 TypedInputs Structure

**Purpose**: Strongly-typed inputs provided by DAG executor to filters

```cpp
struct TypedInputs
{
  // Pointcloud inputs (multiple allowed, identified by name)
  std::map<std::string, std::shared_ptr<cuda_blackboard::CudaPointCloud2>> pointclouds;
  
  // Special input markers (twist, imu) - actual data comes from context queues
  std::vector<std::string> special_inputs;
};
```

**Key Points**:
- DAG executor handles type identification and casting
- Filters receive strongly-typed `CudaPointCloud2::ConstSharedPtr`
- No need for filters to perform `static_pointer_cast`

### 4.3 FilterContext Structure

**Purpose**: Shared execution resources for all filters

```cpp
struct FilterContext
{
  cudaStream_t stream;                        // CUDA stream for async operations
  cudaMemPool_t memory_pool;                  // CUDA memory pool
  tf2_ros::Buffer * tf_buffer;                // TF2 buffer for transforms
  rclcpp::Clock * clock;                       // ROS2 clock
  rclcpp::Logger * logger;                     // ROS2 logger
  
  // Shared preprocessor instance (KEY!)
  CudaPointcloudPreprocessor * shared_preprocessor;
  
  // Queues for distortion correction
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> * twist_queue;
  std::deque<geometry_msgs::msg::Vector3Stamped> * angular_velocity_queue;
};
```

### 4.4 FilterMetadata Structure

**Purpose**: Describes filter capabilities and requirements

```cpp
struct FilterMetadata
{
  std::string filter_type;                    // e.g., "OrganizeFilter"
  std::vector<std::string> required_inputs;   // ["*"] or specific names
  std::vector<std::string> optional_inputs;   // ["twist", "imu"]
  std::vector<std::string> outputs;           // ["organized"]
  std::map<std::string, std::string> input_types;   // name -> type
  std::map<std::string, std::string> output_types;  // name -> type
};
```

---

## 5. DAG Configuration

### 5.1 YAML Structure

```yaml
dag:
  name: "pipeline_name"
  version: "1.0"
  
  inputs:
    - name: "pointcloud"
      type: "sensor_msgs::msg::PointCloud2"
      topic: "~/input/pointcloud"
      optional: false
    
    - name: "twist"
      type: "geometry_msgs::msg::TwistWithCovarianceStamped"
      topic: "~/input/twist"
      optional: false
  
  nodes:
    - id: "organize"
      type: "OrganizeFilter"
      inputs:
        - source: "pointcloud"
          # from_node: ""  # Empty means external input
      outputs:
        - name: "organized"
      parameters: {}
    
    - id: "transform"
      type: "TransformFilter"
      inputs:
        - source: "organized"
          from_node: "organize"
      outputs:
        - name: "transformed"
      parameters:
        target_frame: "base_link"
  
  outputs:
    - name: "output"
      source: "transformed"
      from_node: "transform"
      topic: "~/output/pointcloud"
      type: "cuda_blackboard::CudaPointCloud2"
```

### 5.2 Input Connection Rules

**External Input**:
```yaml
inputs:
  - source: "pointcloud"      # References DAG input
    # from_node: ""            # Empty or omitted
```

**Node Output**:
```yaml
inputs:
  - source: "organized"       # References output name from upstream node
    from_node: "organize"     # Node ID that produces this output
```

**Optional Name Aliasing**:
```yaml
inputs:
  - source: "organized"       # Output name from upstream
    from_node: "organize"
    name: "pointcloud"        # Passed to filter with this name (optional)
```

### 5.3 Output Naming

**Dynamic Output Names**: Filters use the `name` field from YAML outputs:

```yaml
outputs:
  - name: "my_custom_name"    # Filter will output to this name
```

**Access in DAG**:
- Stored internally as: `"node_id.output_name"`
- Referenced by downstream nodes: `source: "my_custom_name", from_node: "node_id"`

### 5.4 Parameter Types

Parameters are automatically inferred:

```yaml
parameters:
  target_frame: "base_link"              # string
  use_3d: false                          # bool
  distance_ratio: 1.03                   # double
  num_rings: 128                         # int
  crop_boxes:                            # array of maps
    - min_x: -50.0
      max_x: 50.0
      min_y: -50.0
      max_y: 50.0
      min_z: -2.0
      max_z: 3.0
```

---

## 6. DAG Executor

### 6.1 Execution Flow

```cpp
// 1. Build DAG
executor_.buildDag(node_configs, context);

// 2. Prepare external inputs
std::map<std::string, std::shared_ptr<void>> external_inputs;
external_inputs["pointcloud"] = pointcloud_ptr;

// 3. Execute DAG
auto outputs = executor_.execute(external_inputs);

// 4. Retrieve outputs
auto final_output = outputs["node_id.output_name"];
```

### 6.2 Internal Process

**buildDag()**:
1. Create filter instances from registry
2. Initialize filters with parameters
3. Validate DAG structure (no cycles)
4. Store filter context

**execute()**:
1. Clear previous outputs
2. Get execution order (topological sort)
3. For each node in order:
   a. Resolve inputs (external or from upstream nodes)
   b. Prepare typed inputs (cast pointclouds, identify special inputs)
   c. Validate inputs
   d. Call filter->process() with typed inputs and output_names
   e. Store outputs with keys: `"node_id.output_name"`
4. Return all node outputs

### 6.3 Topological Sort (Kahn's Algorithm)

```cpp
std::vector<std::size_t> DagExecutor::topologicalSort() const
{
  // Calculate in-degrees
  std::vector<int> in_degree(nodes_.size(), 0);
  for (const auto & node : nodes_) {
    for (const auto & input : node.config.inputs) {
      if (!input.from_node.empty()) {
        in_degree[find_node_index(input.from_node)]++;
      }
    }
  }
  
  // Kahn's algorithm
  std::queue<std::size_t> queue;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    if (in_degree[i] == 0) queue.push(i);
  }
  
  std::vector<std::size_t> result;
  while (!queue.empty()) {
    auto u = queue.front();
    queue.pop();
    result.push_back(u);
    
    // Decrease in-degree for dependent nodes
    for (size_t v = 0; v < nodes_.size(); ++v) {
      for (const auto & input : nodes_[v].config.inputs) {
        if (input.from_node == nodes_[u].config.id) {
          in_degree[v]--;
          if (in_degree[v] == 0) queue.push(v);
        }
      }
    }
  }
  
  if (result.size() != nodes_.size()) {
    throw std::runtime_error("Cycle detected in DAG");
  }
  
  return result;
}
```

### 6.4 Cycle Detection (DFS)

```cpp
bool DagExecutor::hasCycleDFS(
  std::size_t node_index,
  std::vector<bool> & visited,
  std::vector<bool> & rec_stack) const
{
  visited[node_index] = true;
  rec_stack[node_index] = true;
  
  // Check all dependencies
  for (const auto & input : nodes_[node_index].config.inputs) {
    if (!input.from_node.empty()) {
      std::size_t dep_index = find_node_index(input.from_node);
      
      if (!visited[dep_index]) {
        if (hasCycleDFS(dep_index, visited, rec_stack)) {
          return true;  // Cycle found
        }
      } else if (rec_stack[dep_index]) {
        return true;  // Back edge detected (cycle)
      }
    }
  }
  
  rec_stack[node_index] = false;
  return false;
}
```

---

## 7. Type System

### 7.1 Type Identification

The DAG executor identifies and casts types before passing to filters:

```cpp
TypedInputs DagExecutor::prepareTypedInputs(
  const std::map<std::string, std::shared_ptr<void>> & raw_inputs) const
{
  TypedInputs typed_inputs;
  
  for (const auto & [name, data] : raw_inputs) {
    // Try to cast to CudaPointCloud2
    auto pc2_ptr = std::dynamic_pointer_cast<cuda_blackboard::CudaPointCloud2>(
      std::static_pointer_cast<cuda_blackboard::CudaPointCloud2>(data)
    );
    
    if (pc2_ptr) {
      typed_inputs.pointclouds[name] = pc2_ptr;
    } else {
      // Mark as special input (twist, imu, etc.)
      typed_inputs.special_inputs.push_back(name);
    }
  }
  
  return typed_inputs;
}
```

### 7.2 Supported Types

**Primary Data Type**:
- `cuda_blackboard::CudaPointCloud2`: GPU-resident pointcloud data

**Special Input Markers**:
- `"twist"`: References `FilterContext::twist_queue`
- `"imu"`: References `FilterContext::angular_velocity_queue`

### 7.3 Type Conversion

**CPU to GPU** (OrganizeFilter for external input):
```cpp
auto organized_output = context.shared_preprocessor->organizePointcloudPublic(
  cpu_pointcloud);  // sensor_msgs::msg::PointCloud2
```

**GPU to GPU** (OrganizeFilter for internal data):
```cpp
auto organized_output = context.shared_preprocessor->organizePointcloudPublic(
  gpu_pointcloud);  // cuda_blackboard::CudaPointCloud2
```

---

## 8. Dynamic Features

### 8.1 Dynamic Output Names

**Problem**: Filters had hardcoded output names, conflicting with YAML configuration.

**Solution**: Filters receive `output_names` parameter:

```cpp
void MyFilter::process(
  const TypedInputs & inputs,
  std::map<std::string, std::shared_ptr<void>> & outputs,
  FilterContext & context,
  const std::vector<std::string> & output_names)  // ← NEW
{
  // Process data...
  
  // Use dynamic output name from YAML
  outputs[output_names[0]] = processed_data;
}
```

### 8.2 Dynamic Input Names

**Problem**: Filters expected specific input names (e.g., `"pointcloud"`).

**Solution**: Filters access inputs by iterator:

```cpp
bool MyFilter::validateInputs(const TypedInputs & inputs) const
{
  return !inputs.pointclouds.empty();  // Accept any name
}

void MyFilter::process(...) {
  auto input_pc2 = inputs.pointclouds.begin()->second;  // Get first
}
```

### 8.3 Dynamic Subscribers/Publishers

**Node Layer** creates I/O dynamically based on DAG configuration:

```cpp
void CudaPointcloudPreprocessorDagNode::createDynamicSubscribers()
{
  for (const auto & input_config : dag_.inputs) {
    if (input_config.type == "sensor_msgs::msg::PointCloud2" ||
        input_config.type == "cuda_blackboard::CudaPointCloud2") {
      // Create CudaBlackboardSubscriber
      auto sub = std::make_shared<CudaBlackboardSubscriber<CudaPointCloud2>>(...);
      dag_input_subscribers_[input_config.name] = sub;
    }
    else if (input_config.type.find("TwistWithCovarianceStamped") != std::string::npos) {
      // Create InterProcessPollingSubscriber for twist
      auto sub = std::make_shared<InterProcessPollingSubscriber<...>>(...);
      twist_sub_ = sub;
    }
    // ... similar for IMU
  }
}

void CudaPointcloudPreprocessorDagNode::createDynamicPublishers()
{
  for (const auto & output_config : dag_.outputs) {
    auto pub = std::make_shared<CudaBlackboardPublisher<CudaPointCloud2>>(...);
    dag_output_publishers_[output_config.name] = pub;
  }
}
```

---

## 9. Testing Strategy

### 9.1 Test Hierarchy

```
Unit Tests
├── test_dag_executor.cpp          - DAG execution logic, topological sort, cycles
├── test_dag_yaml_parsing.cpp      - YAML parsing, validation, error handling
└── test_filter_integration.cpp    - Filter metadata, registration, basic execution

Integration Tests
└── (Future) Full pipeline tests with real ROS2 node
```

### 9.2 Test Coverage Requirements

#### 9.2.1 DAG Executor Tests (`test_dag_executor.cpp`)

**Core Functionality**:
- [x] Build simple single-node DAG
- [x] Build chained multi-node DAG
- [x] Execute simple DAG with external inputs
- [x] Execute chained DAG
- [ ] Execute DAG with multiple outputs per node
- [ ] Execute DAG with optional inputs

**Error Handling**:
- [x] Missing filter type throws exception
- [x] Cyclic dependency detected and throws
- [ ] Missing required input throws exception
- [ ] Invalid input type throws exception

**Graph Algorithms**:
- [ ] Topological sort produces correct order for linear chain
- [ ] Topological sort produces correct order for diamond dependency
- [ ] Topological sort produces correct order for parallel branches
- [ ] Cycle detection works for self-loop
- [ ] Cycle detection works for long cycles (A→B→C→A)

**Dynamic Features**:
- [ ] Dynamic output names: Filter outputs match YAML config
- [ ] Dynamic input names: Filters accept any input name
- [ ] Type identification: Pointclouds correctly identified and cast
- [ ] Special inputs: Twist/IMU markers correctly identified

#### 9.2.2 YAML Parsing Tests (`test_dag_yaml_parsing.cpp`)

**Valid Configuration**:
- [x] Parse complete valid configuration
- [x] Parse all filter types
- [x] Parse standard_preprocessing.yaml (production config)
- [ ] Parse configuration with optional inputs
- [ ] Parse configuration with multiple outputs per node

**Parameter Validation**:
- [x] TransformFilter: Missing `target_frame` throws
- [x] RingOutlierFilter: Missing `distance_ratio` throws
- [x] RingOutlierFilter: Missing `object_length_threshold` throws
- [ ] CropBoxFilter: Invalid crop_boxes format throws
- [ ] DistortionFilter: Missing `use_3d` throws
- [ ] Parameter type inference: string, bool, int, double, arrays

**Error Handling**:
- [x] Missing node ID throws
- [x] Missing node type throws
- [x] Missing input source throws
- [x] Empty output name throws
- [x] File not found throws
- [ ] Invalid YAML syntax throws
- [ ] Duplicate node IDs throw
- [ ] Reference to non-existent node throws

#### 9.2.3 Filter Integration Tests (`test_filter_integration.cpp`)

**Filter Registration**:
- [x] All filters registered in registry
- [x] Filter metadata accessible
- [ ] Filter factory creates correct instances

**Filter Behavior**:
- [x] OrganizeFilter basic DAG build
- [x] DagTopologicalOrder: organize → transform → crop
- [ ] Each filter type: Initialize with valid parameters
- [ ] Each filter type: Initialize with invalid parameters throws
- [ ] Each filter type: validateInputs rejects invalid inputs
- [ ] Each filter type: process produces expected output type

**Shared Preprocessor**:
- [ ] Filters correctly use shared_preprocessor from context
- [ ] Multiple filters can use shared_preprocessor sequentially
- [ ] Stateless methods: Same filter type with different parameters

#### 9.2.4 New Test Requirements (Based on Current Features)

**TypedInputs Tests**:
- [ ] DAG executor correctly casts pointcloud to CudaPointCloud2
- [ ] DAG executor correctly identifies special inputs (twist, imu)
- [ ] Filter receives strongly-typed pointcloud input
- [ ] Filter with multiple pointcloud inputs

**Dynamic Output Names Tests**:
- [ ] Filter outputs to name specified in YAML
- [ ] Multiple nodes with same filter type, different output names
- [ ] Downstream node correctly references custom output name

**Dynamic I/O Tests**:
- [ ] Node creates subscribers only for inputs defined in DAG
- [ ] Node creates publishers for all outputs defined in DAG
- [ ] Multiple output publishers work correctly

---

## 10. Implementation Guidelines

### 10.1 Creating a New Filter

**Step 1**: Define filter class:

```cpp
// my_filter.hpp
class MyFilter : public IFilter
{
public:
  void initialize(const std::map<std::string, std::any> & params) override;
  void process(
    const TypedInputs & inputs,
    std::map<std::string, std::shared_ptr<void>> & outputs,
    FilterContext & context,
    const std::vector<std::string> & output_names) override;
  FilterMetadata getMetadata() const override;
  bool validateInputs(const TypedInputs & inputs) const override;

private:
  // Store filter-specific parameters
  double my_threshold_;
  bool my_flag_;
};
```

**Step 2**: Implement filter methods:

```cpp
// my_filter.cpp
void MyFilter::initialize(const std::map<std::string, std::any> & params)
{
  // Parse and store parameters
  auto it = params.find("my_threshold");
  if (it != params.end()) {
    my_threshold_ = std::any_cast<double>(it->second);
  } else {
    throw std::runtime_error("MyFilter: Missing required parameter 'my_threshold'");
  }
  
  my_flag_ = params.count("my_flag") ? std::any_cast<bool>(params.at("my_flag")) : false;
}

void MyFilter::process(
  const TypedInputs & inputs,
  std::map<std::string, std::shared_ptr<void>> & outputs,
  FilterContext & context,
  const std::vector<std::string> & output_names)
{
  // Get input
  auto input_pc2 = inputs.pointclouds.begin()->second;
  
  // Process using shared preprocessor or custom CUDA kernel
  auto result = context.shared_preprocessor->myProcessingFunction(*input_pc2, my_threshold_, my_flag_);
  
  // Output with dynamic name
  if (!output_names.empty()) {
    outputs[output_names[0]] = std::static_pointer_cast<void>(result);
  }
}

bool MyFilter::validateInputs(const TypedInputs & inputs) const
{
  return !inputs.pointclouds.empty();
}

FilterMetadata MyFilter::getMetadata() const
{
  return {
    .filter_type = "MyFilter",
    .required_inputs = {"*"},  // Accept any pointcloud name
    .optional_inputs = {},
    .outputs = {"processed"},
    .input_types = {{"*", "cuda_blackboard::CudaPointCloud2"}},
    .output_types = {{"processed", "cuda_blackboard::CudaPointCloud2"}}
  };
}
```

**Step 3**: Register filter:

```cpp
// filter_registrations.cpp
#include "autoware/cuda_pointcloud_preprocessor/dag/filters/my_filter.hpp"

registerFilterType<MyFilter>("MyFilter");
```

**Step 4**: Add parameter validation to parser:

```cpp
// dag_config_parser.cpp
void DagConfigParser::validateFilterParameters(const DagNodeConfig & config)
{
  // ...
  else if (config.type == "MyFilter") {
    if (config.parameters.find("my_threshold") == config.parameters.end()) {
      throw std::runtime_error("MyFilter requires 'my_threshold' parameter");
    }
  }
}
```

### 10.2 Adding New CUDA Kernel

**Step 1**: Implement kernel in shared preprocessor:

```cpp
// cuda_pointcloud_preprocessor.cu
__global__ void myKernel(/* ... */) {
  // CUDA kernel implementation
}

void CudaPointcloudPreprocessor::myProcessingFunctionPublic(
  const cuda_blackboard::CudaPointCloud2 & input,
  double threshold,
  bool flag)
{
  // Launch kernel with provided parameters
  myKernel<<<blocks, threads, 0, stream_>>>(/* ... */, threshold, flag);
}
```

**Step 2**: Use in filter:

```cpp
void MyFilter::process(/* ... */)
{
  auto result = context.shared_preprocessor->myProcessingFunctionPublic(
    *input_pc2, my_threshold_, my_flag_);
}
```

### 10.3 Configuring a Pipeline

**Example**: Downsample → Transform → Crop → Filter

```yaml
dag:
  name: "custom_pipeline"
  version: "1.0"
  
  inputs:
    - name: "raw_pointcloud"
      type: "sensor_msgs::msg::PointCloud2"
      topic: "~/input/pointcloud"
  
  nodes:
    - id: "downsample"
      type: "DownsampleFilter"
      inputs:
        - source: "raw_pointcloud"
      outputs:
        - name: "downsampled"
      parameters:
        voxel_size: 0.1
    
    - id: "transform"
      type: "TransformFilter"
      inputs:
        - source: "downsampled"
          from_node: "downsample"
      outputs:
        - name: "transformed"
      parameters:
        target_frame: "base_link"
    
    - id: "crop"
      type: "CropBoxFilter"
      inputs:
        - source: "transformed"
          from_node: "transform"
      outputs:
        - name: "cropped"
      parameters:
        crop_boxes:
          - min_x: -10.0
            max_x: 10.0
            min_y: -10.0
            max_y: 10.0
            min_z: -2.0
            max_z: 2.0
    
    - id: "finalize"
      type: "FinalizeFilter"
      inputs:
        - source: "cropped"
          from_node: "crop"
      outputs:
        - name: "final"
      parameters: {}
  
  outputs:
    - name: "filtered_output"
      source: "final"
      from_node: "finalize"
      topic: "~/output/pointcloud"
      type: "cuda_blackboard::CudaPointCloud2"
```

---

## Summary

### Key Design Principles

1. **Separation of Concerns**: DAG logic, filter logic, and CUDA kernels are cleanly separated
2. **Shared Resources**: Single preprocessor instance reused by all filters
3. **Stateless Processing**: Filters pass parameters to shared methods, no internal state
4. **Type Safety**: DAG executor handles all type casting, filters receive strongly-typed data
5. **Configuration-Driven**: Complete pipeline definition in YAML, no code changes needed
6. **Validation Early**: Parameter validation at parse time, fail fast
7. **Dynamic Everything**: I/O, output names, input names all configurable

### Testing Priorities

1. ✅ **Core DAG functionality** (building, execution, topological sort)
2. ✅ **YAML parsing and validation**
3. ⚠️ **Dynamic features** (output names, typed inputs) - NEEDS MORE TESTS
4. ⚠️ **Filter-specific behavior** - NEEDS COMPREHENSIVE TESTS
5. ⚠️ **Error handling** - NEEDS MORE EDGE CASES
6. ❌ **Integration tests** - NOT YET IMPLEMENTED

### Next Steps for Testing

Based on this documentation, the test improvement priorities are:

1. **Add TypedInputs tests** to `test_dag_executor.cpp`
2. **Add dynamic output names tests** to `test_dag_executor.cpp`
3. **Add graph algorithm tests** (diamond dependencies, parallel branches)
4. **Expand YAML parsing error tests** (duplicate IDs, invalid references)
5. **Add filter-specific behavior tests** to `test_filter_integration.cpp`
6. **Create integration tests** with real ROS2 node and full pipeline

---

**END OF DOCUMENTATION**

