# Traffic_light_detector

## Purpose

It is a package for traffic light detection using YoloX-s which use full-size image as input.

## Training information

### Training model

The model is trained based on [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) using TIER IV's internal autoware-ml platform.

### Training Data

[wip]

### Trained Onnx Model

## Inner-working / Algorithm

The trained model take whole size image as input and detects all the traffic lights in the images.
The interest traffic lights will be extract by rough/rois and expected/rois from `traffic_light_map_based_detection`.
The package will ouptut separate rois for car traffic light and pedestrian traffic light

## Inputs / Outputs

### Input

| Name            | Type                | Description                |
| --------------- | ------------------- | -------------------------- |
| `~/input/image` | `sensor_msgs/Image` | The full size camera image |

### Output

| Name            | Type                                               | Description                |
| --------------- | -------------------------------------------------- | -------------------------- |
| `~/output/rois` | `tier4_perception_msgs::msg::TrafficLightRoiArray` | The detected accurate rois |
