# CLAUDE.md - Autoware TensorRT VAD

This file provides guidance to Claude Code when working with the Autoware TensorRT VAD ROS 2 package.

## Package Overview

This package integrates VAD (Vectorized Scene Representation for Efficient Autonomous Driving) into the Autoware autonomous driving stack as a ROS 2 component. It provides real-time trajectory generation and object prediction using TensorRT-optimized neural networks.

**Supported Models:**
- **nuScenes Models**: Standard VAD models (tiny/base) trained on nuScenes dataset
- **CARLA Tier4 Models**: Models trained on CARLA/B2D data with Tier4 coordinate system

See [MODEL_COMPARISON.md](docs/MODEL_COMPARISON.md) for detailed differences between models.

## Architecture

### Core Components

1. **VadModel** (`vad_model.hpp/cpp`):
   - Manages TensorRT engines for backbone and head networks
   - Handles ONNX to TRT conversion and caching
   - Performs inference with optimized CUDA operations
   - Maintains temporal state between frames

2. **VadInterface** (`vad_interface.hpp/cpp`):
   - Bridges ROS data to VAD model inputs
   - Handles coordinate transformations (camera → ego → map frames)
   - Converts VAD outputs to Autoware message formats
   - Manages multi-camera synchronization

3. **VadNode** (`vad_node.hpp/cpp`):
   - Main ROS 2 component node
   - Subscribes to camera and vehicle state topics
   - Publishes trajectories, predicted objects, and visualizations
   - Implements synchronization strategy for multi-camera inputs

4. **Network Components** (`networks/`):
   - `backbone.hpp/cpp`: Feature extraction network wrapper
   - `head.hpp/cpp`: Prediction head network wrapper
   - `net.hpp/cpp`: Base network class with TensorRT management
   - `tensor.hpp/cpp`: CUDA tensor utilities and operations

### Data Flow

```
Camera Images (6x) + Camera Info → Synchronization → Feature Extraction (Backbone)
                                                            ↓
Vehicle State (Odometry, Acceleration) → Input Processing → Prediction Head
                                                            ↓
                                                    Output Processing
                                                            ↓
                            [Trajectories, Predicted Objects, Map Visualization]
```

## Key Features

### Multi-Camera Synchronization
- Uses front camera (ID 0) as anchor for temporal alignment
- Configurable synchronization tolerance (default 100ms)
- Handles both raw and compressed image formats

### TensorRT Optimization
- Automatic ONNX to TRT engine conversion with caching
- FP16 inference support for improved performance
- Custom CUDA plugins for specialized operations

### Temporal Consistency
- Maintains BEV features between frames
- Separate models for first frame vs subsequent frames
- Handles temporal feature propagation

### Coordinate Systems
- **Camera Frame**: Raw image coordinates
- **Ego Frame**: Vehicle-centered coordinates
- **Map Frame**: Global map coordinates
- Proper transformation chain maintained throughout

## Configuration

### Model Parameters (`vad_tiny.param.yaml`)
```yaml
model_params:
  plugins_path: "path/to/tensorrt_plugins.so"  # Custom TRT plugins
  default_command: 2  # Navigation command (0: right, 1: left, 2: forward)
  map_confidence_thresholds: [0.9, 0.9, 0.9]  # Per-class thresholds
  object_confidence_thresholds: [0.5, ...]     # 10 object classes
```

### Interface Parameters
```yaml
interface_params:
  map_colors: [r,g,b, ...]  # Visualization colors for map elements
```

### Synchronization Parameters
```yaml
sync_params:
  front_camera_id: 0        # Anchor camera for sync
  sync_tolerance_ms: 100.0  # Max time difference between cameras
```

## Object Classes

The model predicts 10 object classes (nuScenes format):
0. Car
1. Truck
2. Bus
3. Trailer
4. Construction Vehicle
5. Pedestrian
6. Motorcycle
7. Bicycle
8. Traffic Cone
9. Barrier

## Map Elements

Three types of map polylines are predicted:
- **Divider**: Lane dividers (blue visualization)
- **Ped Crossing**: Pedestrian crossings (red visualization)
- **Boundary**: Road boundaries (grey visualization)

## Building and Testing

### Build Commands
```bash
# Build with Release mode for optimal performance
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to autoware_tensorrt_vad

# Run unit tests
colcon test --packages-select autoware_tensorrt_vad
colcon test-result --all
```

### Required Dependencies
- ROS 2 (Humble or later)
- CUDA (11.8+ recommended)
- TensorRT (8.5+ recommended)
- OpenCV 4.x
- Eigen3
- autoware_tensorrt_common
- autoware_tensorrt_plugins

## ONNX Model Export

Before running, export VAD models to ONNX format using the scripts in `trt/export_eval/`:

```bash
# Export backbone (feature extraction)
python trt/export_eval/export_backbone.py

# Export head for first frame
python trt/export_eval/export_no_prev.py

# Export head for subsequent frames (with temporal features)
python trt/export_eval/export_prev.py
```

Place exported ONNX files in `~/autoware_data/vad/`:
- `sim_vadv1.extract_img_feat.onnx`
- `sim_vadv1.pts_bbox_head.forward.onnx`
- `sim_vadv1_prev.pts_bbox_head.forward.onnx`

## Launch Configuration

### For nuScenes Models
The launch file (`launch/vad.launch.xml`) configures:
- Camera topic remapping (6 cameras)
- Vehicle state topic remapping
- Output topic namespaces
- Model and data paths

### For CARLA Tier4 Models
Use the dedicated launch file (`launch/vad_carla_tier4.launch.xml`) which:
- Loads CARLA-specific configuration files
- Uses correct object class remapper for 9 classes
- Configures identity transformation matrix
- Sets appropriate BEV dimensions (64×120)

See [CARLA_TIER4_INTEGRATION.md](docs/CARLA_TIER4_INTEGRATION.md) for detailed setup instructions.

### Topic Mappings

**Input Topics**:
- `/sensing/camera/camera[0-5]/image_rect_color`: Rectified camera images
- `/sensing/camera/camera[0-5]/camera_info`: Camera calibration
- `/localization/kinematic_state`: Vehicle odometry
- `/localization/acceleration`: Vehicle acceleration

**Output Topics**:
- `/planning/vad/trajectory`: Main planned trajectory
- `/planning/vad/trajectories`: Multiple candidate trajectories
- `/perception/object_recognition/vad/objects`: Predicted objects
- `/debug/vad/map`: Map visualization markers

## Performance Considerations

1. **GPU Memory**: Requires ~4-6GB GPU memory for tiny model
2. **Inference Time**: Target <100ms per frame for real-time operation
3. **Input Resolution**: Designed for 900x1600 camera images (nuScenes resolution)
4. **Batch Size**: Processes all 6 cameras in single batch for efficiency

## Debugging

### Enable Debug Logging
```bash
ros2 launch autoware_tensorrt_vad vad.launch.xml log_level:=debug
```

### Common Issues

1. **TensorRT Engine Build**: First run takes longer due to engine optimization
2. **Synchronization Timeout**: Adjust `sync_tolerance_ms` if cameras have varying latencies
3. **Memory Allocation**: Ensure sufficient GPU memory available
4. **Plugin Loading**: Verify `plugins_path` points to compiled TensorRT plugins

## Integration with Autoware

This package is designed to work within Autoware's modular architecture:
- Replaces traditional perception/prediction modules with end-to-end approach
- Compatible with Autoware's planning interfaces
- Can be used alongside other perception modules for redundancy

## Future Improvements

- [ ] Support for variable number of cameras
- [ ] Dynamic resolution handling
- [ ] INT8 quantization support
- [ ] Multi-GPU inference distribution
- [ ] Online model updating/adaptation