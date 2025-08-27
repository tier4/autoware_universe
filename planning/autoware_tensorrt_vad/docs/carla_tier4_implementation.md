# CARLA Tier4 VAD Model Implementation

## Overview
This document describes the implementation of CARLA Tier4 VAD model support in the autoware_tensorrt_vad package. The CARLA model is trained on the CARLA simulator dataset with Tier4's specific configurations and provides 9-class object detection capabilities.

## Key Differences from NuScenes Model

### Object Classes
- **CARLA Model**: 9 classes
  - car, truck, bus, trailer, construction_vehicle, pedestrian, motorcycle, bicycle, traffic_cone
- **NuScenes Model**: 10 classes  
  - Includes barrier class not present in CARLA model

### Model Architecture
- Both models use the same VAD architecture
- Input/output tensor shapes remain consistent
- Main difference is in the object classification head (9 vs 10 classes)

## Implementation Details

### Configuration Files

#### 1. `vad_carla_tier4.param.yaml`
Main VAD configuration with CARLA-specific parameters:
- Model path pointing to CARLA Tier4 ONNX file
- Object class configuration for 9 classes
- Adjusted confidence thresholds per class
- CARLA-specific NMS settings

#### 2. `object_class_remapper_carla_tier4.param.yaml`
Maps CARLA's 9 object classes to Autoware's object types:
```yaml
car → CAR
truck → TRUCK  
bus → BUS
trailer → TRAILER
construction_vehicle → TRUCK
pedestrian → PEDESTRIAN
motorcycle → MOTORCYCLE
bicycle → BICYCLE
traffic_cone → UNKNOWN
```

#### 3. `ml_package_vad_carla_tier4.param.yaml`
ML package configuration specifying:
- CARLA Tier4 model location
- TensorRT precision settings
- Input/output tensor configurations

### Launch File
`vad_carla_tier4.launch.xml` provides:
- Proper topic remapping for CARLA simulation
- Loading of CARLA-specific parameter files
- Component configuration for TensorRT optimization

### Code Modifications
- Updated `vad_model.hpp` to handle variable number of object classes
- Modified launch files to support model selection via parameters

## Usage

### Running with CARLA Tier4 Model
```bash
ros2 launch autoware_tensorrt_vad vad_carla_tier4.launch.xml
```

### Testing Configuration
A test script `test_carla_tier4_config.py` is provided to validate:
- Parameter file syntax
- Object class mapping consistency
- Configuration completeness

## Integration with CARLA Simulator

### Coordinate System
- CARLA uses the same coordinate system as Autoware (ROS REP-103)
- No additional coordinate transformation required

### Topic Mapping
The launch file handles necessary topic remapping:
- Input: `/sensing/camera/*/image_raw` (6 cameras)
- Output: `/perception/object_recognition/objects`

## Performance Considerations

### TensorRT Optimization
- First launch generates TensorRT engine (takes 5-10 minutes)
- Cached engine files stored in model directory
- FP16 precision enabled by default for better performance

### Resource Requirements
- GPU: Minimum 8GB VRAM recommended
- Similar performance characteristics to NuScenes model

## Future Improvements

1. **Dynamic Class Handling**: Implement runtime object class detection to automatically handle different model configurations
2. **Unified Launch**: Create a single launch file with model selection parameter
3. **Performance Tuning**: CARLA-specific threshold optimization based on simulation data
4. **Validation Tools**: Automated testing against CARLA ground truth data

## Troubleshooting

### Common Issues

1. **Model Not Found**
   - Ensure ONNX model file is downloaded and placed in correct directory
   - Check model path in `vad_carla_tier4.param.yaml`

2. **Class Mismatch Errors**
   - Verify object_class_names array has exactly 9 elements
   - Check remapper configuration matches model output

3. **TensorRT Engine Generation Failed**
   - Ensure sufficient GPU memory available
   - Check CUDA and TensorRT compatibility

## References

- CARLA Simulator: https://carla.org/
- Tier4 Autoware: https://github.com/tier4/autoware
- VAD Paper: [Vision-Centric BEV Perception](https://arxiv.org/abs/2203.10981)