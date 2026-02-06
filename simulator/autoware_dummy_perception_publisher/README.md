# dummy_perception_publisher

## Purpose

This node publishes the result of the dummy detection with the type of perception.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name           | Type                                      | Description             |
| -------------- | ----------------------------------------- | ----------------------- |
| `/tf`          | `tf2_msgs/TFMessage`                      | TF (self-pose)          |
| `input/object` | `tier4_simulation_msgs::msg::DummyObject` | dummy detection objects |

### Output

| Name                                | Type                                                     | Description             |
| ----------------------------------- | -------------------------------------------------------- | ----------------------- |
| `output/dynamic_object`             | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | dummy detection objects |
| `output/points_raw`                 | `sensor_msgs::msg::PointCloud2`                          | point cloud of objects  |
| `output/debug/ground_truth_objects` | `autoware_perception_msgs::msg::TrackedObjects`          | ground truth objects    |

## Parameters

| Name                        | Type   | Default Value | Explanation                                      |
| --------------------------- | ------ | ------------- | ------------------------------------------------ |
| `visible_range`             | double | 100.0         | sensor visible range [m]                         |
| `detection_successful_rate` | double | 0.8           | sensor detection rate. (min) 0.0 - 1.0(max)      |
| `enable_ray_tracing`        | bool   | true          | if True, use ray tracking                        |
| `use_object_recognition`    | bool   | true          | if True, publish objects topic                   |
| `use_base_link_z`           | bool   | true          | if True, node uses z coordinate of ego base_link |
| `publish_ground_truth`      | bool   | false         | if True, publish ground truth objects            |
| `use_fixed_random_seed`     | bool   | false         | if True, use fixed random seed                   |
| `random_seed`               | int    | 0             | random seed                                      |

### Node Parameters

None.

### Core Parameters

None.

## Assumptions / Known limits

| Name                               | Type   | Default Value | Explanation                                                             |
| ---------------------------------- | ------ | ------------- | ----------------------------------------------------------------------- |
| `min_predicted_path_keep_duration` | double | 3.0           | minimum time (seconds) to keep using same prediction                    |
| `switch_time_threshold`            | double | 2.0           | time threshold (seconds) to switch from straight-line to predicted path |

#### Common Remapping Parameters

The plugin uses `CommonParameters` for both vehicle and pedestrian object types. Each parameter is prefixed with either `vehicle.` or `pedestrian.`:

| Parameter Name               | Type   | Explanation                                               |
| ---------------------------- | ------ | --------------------------------------------------------- |
| `max_remapping_distance`     | double | maximum distance (meters) for remapping validation        |
| `max_speed_difference_ratio` | double | maximum speed difference ratio tolerance                  |
| `min_speed_ratio`            | double | minimum speed ratio relative to dummy object speed        |
| `max_speed_ratio`            | double | maximum speed ratio relative to dummy object speed        |
| `speed_check_threshold`      | double | speed threshold (m/s) above which speed checks apply      |
| `path_selection_strategy`    | string | path selection strategy: "highest_confidence" or "random" |

## Auxiliary Nodes

### empty_objects_publisher

A simple node that publishes empty `PredictedObjects` messages at 10 Hz on `~/output/objects`. This is used as a fallback when no object recognition is running, ensuring downstream nodes still receive the expected topic.

### empty_traffic_light_group_array_publisher

A simple node that publishes empty `TrafficLightGroupArray` messages at 10 Hz on `~/output/traffic_light_group_array`. This is used as a fallback when no traffic light recognition is running, ensuring downstream nodes still receive the expected topic.

In the launch file, this node is enabled by the `use_empty_traffic_light_recognition` argument (default: `true`) and remaps its output to `/perception/traffic_light_recognition/traffic_signals`.
