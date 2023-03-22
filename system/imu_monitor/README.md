# imu_monitor

## Purpose

This module compares the yaw rate obtained from the IMU with that obtained from the vehicle and outputs ERROR diagnostics if the difference is large.

## Inputs / Outputs

### Input

| Name            | Type                                             | Description                                        |
| --------------- | ------------------------------------------------ | -------------------------------------------------- |
| `~/input/imu`   | `sensor_msgs::msg::Imu`                          | output from IMU                                    |
| `~/input/twist` | `geometry_msgs::msg::TwistWithCovarianceStamped` | twist with covariance converted from VehicleReport |

### Output

| Name           | Type                                    | Description |
| -------------- | --------------------------------------- | ----------- |
| `/diagnostics` | `diagnostic_msgs::msg::DiagnosticArray` | Diagnotics  |

## Parameters

| Name                      | Type   | Description                      |
| ------------------------- | ------ | -------------------------------- |
| `frame_id`                | string | frame id for base_link           |
| `yaw_rate_diff_threshold` | double | threshold of yaw rate difference |
