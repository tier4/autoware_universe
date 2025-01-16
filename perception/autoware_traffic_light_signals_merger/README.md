# The `autoware_traffic_light_signal_merger` Package

## Overview

`autoware_traffic_light_signal_merger` receives the Traffic Light (TL) classification result from Car/Pedestrian classifiers and expected ROIs to merge into single classification result. The expect ROIs TL without classification result will be filled as Unknown.

## Input topics

| Name                       | Type                                           | Description                   |
| -------------------------- | ---------------------------------------------- | ----------------------------- |
| `input/car_signals`        | tier4_perception_msgs::msg::TrafficLightArray  | Car TLs classification        |
| `input/pedestrian_signals` | tier4_perception_msgs::msg::TrafficLightArray  | Pedestrian TLs classification |
| `input/expect_rois`        | autoware_perception_msgs::TrafficLightRoiArray | expected TL ROIs              |

## Output topics

| Name                           | Type                                          | Description                           |
| ------------------------------ | --------------------------------------------- | ------------------------------------- |
| `output/traffic_light_signals` | tier4_perception_msgs::msg::TrafficLightArray | Car and Pedestrian TLs classification |

## Node parameters

N/A
