# autoware_perception_filter

## Purpose

This package provides a perception filter node that filters perception data based on external approval signals. It receives perception outputs (objects and pointcloud) and publishes filtered versions based on approval status.

## Inputs / Outputs

### Input

| Name               | Type                                              | Description                                |
| ------------------ | ------------------------------------------------- | ------------------------------------------ |
| `input/objects`    | `autoware_perception_msgs::msg::PredictedObjects` | Predicted objects from perception module   |
| `input/pointcloud` | `sensor_msgs::msg::PointCloud2`                   | Obstacle pointcloud from perception module |
| `input/approval`   | `std_msgs::msg::Bool`                             | External approval signal                   |

### Output

| Name                         | Type                                              | Description                  |
| ---------------------------- | ------------------------------------------------- | ---------------------------- |
| `output/filtered_objects`    | `autoware_perception_msgs::msg::PredictedObjects` | Filtered predicted objects   |
| `output/filtered_pointcloud` | `sensor_msgs::msg::PointCloud2`                   | Filtered obstacle pointcloud |

## Parameters

### Core Parameters

| Name                          | Type | Default Value | Description                         |
| ----------------------------- | ---- | ------------- | ----------------------------------- |
| `enable_object_filtering`     | bool | true          | Enable/disable object filtering     |
| `enable_pointcloud_filtering` | bool | true          | Enable/disable pointcloud filtering |

## Usage

### Launch

```bash
ros2 launch autoware_perception_filter perception_filter.launch.xml
```

### Default Topic Mapping

- Input objects: `/perception/object_recognition/objects`
- Input pointcloud: `/perception/obstacle_segmentation/pointcloud`
- Input approval: `/external/approval`
- Output filtered objects: `/perception/object_recognition/filtered_objects`
- Output filtered pointcloud: `/perception/obstacle_segmentation/filtered_pointcloud`

## Behavior

- **When approval is received (true)**: All perception data is passed through unchanged
- **When approval is not received (false)**:
  - Objects are filtered out (empty object list is published)
  - Pointcloud is filtered out (empty pointcloud is published)

## Node Graph

```text
/external/approval ────────┐
                           │
/perception/object_recognition/objects ──┐
                                         │
                                    [perception_filter_node]
                                         │
/perception/obstacle_segmentation/pointcloud ──┘
                                         │
                                         ├── /perception/object_recognition/filtered_objects
                                         │
                                         └── /perception/obstacle_segmentation/filtered_pointcloud
```
