# autoware_traffic_light_whole_image_detector

ROS2 package that runs the CoMLOps large detection model (regression model similar to [trt-lightnet](https://github.com/your-org/trt-lightnet)) on whole images and publishes **traffic light detection ROIs** as `tier4_perception_msgs/msg/TrafficLightRoiArray`.

Inference uses **TensorRT** via `autoware_tensorrt_common` (same pattern as `autoware_traffic_light_classifier` ComlopsTLR classifier): load ONNX, build/load engine, preprocess (blobFromImage 1/255), enqueueV3, decode multi-head outputs (Scaled-YOLOv4 style + TLR head), NMS, filter to TRAFFIC_LIGHT class.

## Overview

- **Input:** `sensor_msgs/msg/Image` (e.g. camera image)
- **Output:** `tier4_perception_msgs/msg/TrafficLightRoiArray` (list of ROIs for traffic lights)
- **Model:** ONNX under `models/` (e.g. `CoMLOps-Large-Detection-Model-v1.0.1.onnx`); TensorRT builds a `.engine` alongside it. Same decoding/NMS logic as trt-lightnet (multi-head detection + TLR head).

## Parameters (same as original trt-lightnet)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `onnx_path` | string | set from launch | Path to ONNX model file (absolute or relative to package share). Default in launch: `/home/dainguyen/autoware_data/traffic_light_classifier/CoMLOps-Large-Detection-Model-v1.0.1.onnx`. |
| `names_file` | string | `data/T4x.names` | Class names file (one per line) |
| `rgb_colormap` | string | `data/T4x.colormap` | Colormap for visualization (optional) |
| `precision` | string | `fp16` | Inference precision (fp32, fp16; int8 requires calibration) |
| `calibration_images` | string | "" | Path to calibration image list for INT8 |
| `calib` | string | `Entropy` | Calibration type (Entropy, Minmax, Legacy) |
| `sparse` | bool | true | 2:4 sparsity (build-time for TensorRT; ignored for ONNX) |
| `anchors` | int[] | (see config) | Anchor sizes |
| `num_anchors` | int | 5 | Number of anchors |
| `num_classes` | int | 10 | Number of classes |
| `score_thresh` | double | 0.2 | Detection score threshold |
| `nms_thresh` | double | 0.6 | NMS IoU threshold |
| `use_cuda` | bool | true | Use CUDA execution provider (ONNX Runtime) |
| `traffic_light_class_name` | string | `TRAFFIC_LIGHT` | Only publish ROIs for this class; empty = all detections |

## Dependencies

- **TensorRT, CUDA, cuDNN** (required for real inference): Same as `autoware_traffic_light_classifier`. If not all found at build time, the package builds a **stub** that publishes empty ROIs and logs: *"Whole image detector built without TensorRT"*.

### Enable inference (fix "publishing empty ROIs")

Build in a workspace where TensorRT/CUDA/cuDNN are available (e.g. Autoware full build). You should see: **"TensorRT found: building whole_image_detector with inference (ComlopsDetector)"**. The node will load the ONNX model, build or load the TensorRT engine (`.engine` next to the `.onnx`), and run inference.

- Data files from trt-lightnet can be used: copy or symlink `T4x.names` and `T4x.colormap` from `/home/dainguyen/semseg/trt-lightnet/data` into this package’s `data/`, or set `names_file` / `rgb_colormap` to absolute paths.

## Build and run

```bash
# From your Autoware/universe workspace
colcon build --packages-select autoware_traffic_light_whole_image_detector
source install/setup.bash

# Run (with default config)
ros2 launch autoware_traffic_light_whole_image_detector traffic_light_whole_image_detector.launch.xml

# Override input image topic
ros2 launch autoware_traffic_light_whole_image_detector traffic_light_whole_image_detector.launch.xml input/image:=/sensor/camera/image_raw
```

## Topics

- **Subscribe:** `~/input/image` (`sensor_msgs/msg/Image`)
- **Publish:** `~/output/rois` (`tier4_perception_msgs/msg/TrafficLightRoiArray`)
- **Publish (optional):** `~/output/debug/image` (`sensor_msgs/msg/Image`) — input image with detected ROIs drawn (green boxes and label `TL#id score%`). Only published when at least one subscriber exists.
- **Publish (debug):** `~/debug/cyclic_time_ms`, `~/debug/processing_time_ms`, `~/debug/pipeline_latency_ms` (`autoware_internal_debug_msgs/msg/Float64Stamped`) — processing time diagnostics (same pattern as other Autoware perception packages).
