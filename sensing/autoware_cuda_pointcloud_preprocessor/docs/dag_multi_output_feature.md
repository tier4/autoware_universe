# DAG Multi-Output Feature Documentation

## Overview

The CUDA Pointcloud Preprocessor DAG node supports **multiple simultaneous outputs**, allowing different stages of the processing pipeline to be published to separate ROS topics. This is a core feature that enables:

1. **Debugging and visualization** of intermediate processing stages
2. **Flexible pipeline branching** where different outputs can be consumed by different downstream nodes
3. **Performance monitoring** by comparing data at different processing stages

## Architecture

### Dynamic Publisher Creation

Publishers are created dynamically based on the `dag.outputs` configuration in the YAML file. Each output specifies:
- **name**: A descriptive name for the output
- **source**: The output port name from a filter node
- **from_node**: The ID of the node producing this output
- **topic**: The ROS topic to publish to
- **type**: The message type (currently only `cuda_blackboard::CudaPointCloud2` is supported)

### Implementation Details

```cpp
// Publishers are stored in a map: output_key -> publisher
std::map<std::string, std::unique_ptr<CudaBlackboardPublisher<CudaPointCloud2>>> publishers_;

// Output key format: "node_id.output_port"
// Example: "organize.organized", "finalize.output"
```

During execution, the DAG executor produces a map of outputs with keys matching the `node_id.output_port` format. The node iterates through all configured publishers and publishes the corresponding data.

## Configuration Example

### Single Output (Standard)

```yaml
dag:
  name: "standard_preprocessing"
  # ... nodes configuration ...
  
  outputs:
    - name: "preprocessed_pointcloud"
      source: "output"
      from_node: "finalize"
      topic: "~/output/pointcloud"
      type: "cuda_blackboard::CudaPointCloud2"
```

### Multiple Outputs (Advanced)

```yaml
dag:
  name: "multi_output_preprocessing"
  # ... nodes configuration ...
  
  outputs:
    # Intermediate output: organized pointcloud
    - name: "organized_pointcloud"
      source: "organized"
      from_node: "organize"
      topic: "~/output/organized"
      type: "cuda_blackboard::CudaPointCloud2"
    
    # Final output: fully processed pointcloud
    - name: "preprocessed_pointcloud"
      source: "output"
      from_node: "finalize"
      topic: "~/output/pointcloud"
      type: "cuda_blackboard::CudaPointCloud2"
    
    # Additional output: cropped pointcloud (before distortion correction)
    - name: "cropped_pointcloud"
      source: "cropped"
      from_node: "cropbox"
      topic: "~/output/cropped"
      type: "cuda_blackboard::CudaPointCloud2"
```

## Use Cases

### 1. Debug Visualization

Publish intermediate stages for RViz visualization to debug processing issues:

```yaml
outputs:
  - name: "raw_organized"
    source: "organized"
    from_node: "organize"
    topic: "~/debug/organized"
    
  - name: "after_crop"
    source: "cropped"
    from_node: "cropbox"
    topic: "~/debug/cropped"
    
  - name: "after_distortion"
    source: "undistorted"
    from_node: "distortion"
    topic: "~/debug/undistorted"
    
  - name: "final"
    source: "output"
    from_node: "finalize"
    topic: "~/output/pointcloud"
```

### 2. Pipeline Branching

Process the same input data through different paths and publish both results:

```yaml
nodes:
  # ... organize, transform ...
  
  # Branch 1: Aggressive filtering
  - id: "cropbox_aggressive"
    type: "CropBoxFilter"
    parameters:
      crop_boxes:
        - min_x: -20.0
          max_x: 20.0
          # ... tight bounds
  
  # Branch 2: Conservative filtering
  - id: "cropbox_conservative"
    type: "CropBoxFilter"
    parameters:
      crop_boxes:
        - min_x: -100.0
          max_x: 100.0
          # ... wide bounds

outputs:
  - name: "aggressive_filtered"
    source: "output"
    from_node: "finalize_aggressive"
    topic: "~/output/aggressive"
    
  - name: "conservative_filtered"
    source: "output"
    from_node: "finalize_conservative"
    topic: "~/output/conservative"
```

### 3. Performance Monitoring

Publish both raw and processed data to measure filter effectiveness:

```yaml
outputs:
  - name: "input_stats"
    source: "organized"
    from_node: "organize"
    topic: "~/monitor/input"
    
  - name: "output_stats"
    source: "output"
    from_node: "finalize"
    topic: "~/monitor/output"
```

## Launch File Configuration

When using multiple outputs, remap each topic in the launch file:

```xml
<launch>
  <arg name="output/organized" default="/sensing/lidar/organized"/>
  <arg name="output/cropped" default="/sensing/lidar/cropped"/>
  <arg name="output/pointcloud" default="/sensing/lidar/pointcloud"/>

  <node pkg="autoware_cuda_pointcloud_preprocessor" 
        exec="cuda_pointcloud_preprocessor_dag_node" 
        name="cuda_pointcloud_preprocessor_dag">
    <!-- ... input remaps ... -->
    <remap from="~/output/organized" to="$(var output/organized)"/>
    <remap from="~/output/cropped" to="$(var output/cropped)"/>
    <remap from="~/output/pointcloud" to="$(var output/pointcloud)"/>
    
    <param name="dag_config_file" value="$(find-pkg-share autoware_cuda_pointcloud_preprocessor)/config/dag/multi_output_example.yaml"/>
  </node>
</launch>
```

## Performance Considerations

### Zero-Copy Publishing

All outputs use CUDA blackboard for zero-copy GPU memory sharing:
- Data stays on GPU throughout the pipeline
- Publishing only copies metadata and device pointers
- Multiple outputs do NOT duplicate GPU memory (shared_ptr reference counting)

### Memory Overhead

Publishing multiple outputs has minimal memory overhead:
- **Metadata**: ~200 bytes per output (ROS message header + point cloud metadata)
- **GPU Memory**: Shared via `shared_ptr`, no duplication
- **Copy on publish**: Only when converting `shared_ptr` to `unique_ptr` for ROS publishing

### Bandwidth Considerations

Each output topic consumes network bandwidth when transmitted over DDS:
- **CUDA topics** (local): Minimal overhead, pointer sharing only
- **Remote topics**: Full pointcloud data transfer
- **Recommendation**: Use `SensorDataQoS` with `keep_last(1)` to avoid queue buildup

## Validation and Error Handling

### Output Validation

The node validates each output configuration:
```cpp
if (output_config.type != "cuda_blackboard::CudaPointCloud2") {
  RCLCPP_WARN(get_logger(), 
    "Output '%s' has unsupported type '%s', skipping.",
    output_config.name.c_str(), output_config.type.c_str());
  continue;
}
```

### Missing Output Detection

If a configured output is not produced by the DAG:
```cpp
RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
  "Output '%s' not found in DAG execution results", output_key.c_str());
```

### Empty Output Handling

Empty outputs (width == 0) are silently skipped:
```cpp
if (output_shared && output_shared->width > 0) {
  publisher->publish(std::move(output_unique));
} else {
  RCLCPP_DEBUG(get_logger(), "Output '%s' is empty, skipping", output_key.c_str());
}
```

## Testing

### Manual Testing

```bash
# Launch with multi-output config
ros2 launch autoware_cuda_pointcloud_preprocessor cuda_pointcloud_preprocessor_dag_multi_output.launch.xml

# Check created topics
ros2 topic list | grep cuda_pointcloud_preprocessor_dag

# Expected output:
# /cuda_pointcloud_preprocessor_dag/output/organized/cuda
# /cuda_pointcloud_preprocessor_dag/output/pointcloud/cuda
```

### Automated Testing

```bash
# Run the multi-output test script
./src/autoware/universe/sensing/autoware_cuda_pointcloud_preprocessor/test/test_multi_output.sh
```

## Limitations

1. **Type Support**: Currently only `cuda_blackboard::CudaPointCloud2` is supported
2. **GPU Memory**: All outputs must fit in GPU memory simultaneously (shared references)
3. **Cycle Detection**: DAG must remain acyclic; publishing intermediate outputs doesn't create cycles as publishers are output-only

## Future Enhancements

- [ ] Support for non-pointcloud output types (e.g., diagnostic messages)
- [ ] Conditional outputs (publish only when conditions are met)
- [ ] Output rate limiting (publish intermediate outputs at lower frequency)
- [ ] CPU fallback for outputs (copy to host memory for CPU-based consumers)

## References

- **Multi-output example**: `config/dag/multi_output_example.yaml`
- **Implementation**: `src/dag/cuda_pointcloud_preprocessor_dag_node.cpp`
- **Test script**: `test/test_multi_output.sh`

