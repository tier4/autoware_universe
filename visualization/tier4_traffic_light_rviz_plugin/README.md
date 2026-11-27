# tier4_traffic_light_rviz_plugin

## Purpose

This package provides RViz panels to publish dummy traffic light signals for simulation.

## Panels

### TrafficLightPublishPanel

Manually select traffic light IDs from the map and publish configured signals.

### TrafficLightRouteOverridePanel

Automatically extracts traffic lights on the current route (including conflicting crosswalk signals) and publishes override signals at 10 Hz. Intended for Planning Simulator where traffic light recognition is disabled by default.

## Inputs / Outputs

### TrafficLightPublishPanel

#### Output

| Name                                                    | Type                                                    | Description                   |
| ------------------------------------------------------- | ------------------------------------------------------- | ----------------------------- |
| `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | Publish traffic light signals |

### TrafficLightRouteOverridePanel

#### Input

| Name                                                    | Type                                                    | Description                              |
| ------------------------------------------------------- | ------------------------------------------------------- | ---------------------------------------- |
| `/map/vector_map`                                       | `autoware_map_msgs::msg::LaneletMapBin`                 | Vector map for route traffic light query |
| `/planning/mission_planning/route`                      | `autoware_planning_msgs::msg::LaneletRoute`             | Route to extract traffic lights          |
| `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | Current traffic light states             |

#### Output

| Name                                                    | Type                                                    | Description                   |
| ------------------------------------------------------- | ------------------------------------------------------- | ----------------------------- |
| `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | Publish traffic light signals |

## HowToUse

### TrafficLightPublishPanel

<div align="center">
  <img src="images/select_panels.png" width=50%>
</div>
<div align="center">
  <img src="images/select_traffic_light_publish_panel.png" width=50%>
</div>
<div align="center">
  <img src="images/select_traffic_light_id.png" width=50%>
</div>

1. Start rviz and select panels/Add new panel.
2. Select TrafficLightPublishPanel and press OK.
3. Set `Traffic Light ID` & `Traffic Light Status` and press `SET` button.
4. Traffic light signals are published, while `PUBLISH` button is pushed.

<div align="center">
  <img src="images/traffic_light_publish_panel.gif">
</div>

### TrafficLightRouteOverridePanel

1. Start Planning Simulator and open RViz.
2. Select Panels / Add new panel / TrafficLightRouteOverridePanel.
3. Set a route in RViz. Traffic lights on the route are listed automatically.
4. Optionally change the override color for each signal (default: GREEN for vehicle signals, RED for pedestrian signals).
5. Press the toggle button to switch to `ENABLED (10Hz)` and publish override signals.

When a new route is set, the list is refreshed and override colors are reset to GREEN for vehicle signals and RED for pedestrian signals.
