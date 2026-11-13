# Brake defect detector

This node passively compares requested deceleration with measured longitudinal acceleration. It does not publish or modify control commands.

## Interfaces

| Topic                            | Type                                             | Purpose                                                                          |
| -------------------------------- | ------------------------------------------------ | -------------------------------------------------------------------------------- |
| `~/input/control_cmd`            | `autoware_control_msgs/msg/Control`              | Requested longitudinal acceleration                                              |
| `~/input/actuation_cmd`          | `tier4_vehicle_msgs/msg/ActuationCommandStamped` | Brake actuation command                                                          |
| `~/input/kinematics`             | `nav_msgs/msg/Odometry`                          | Longitudinal speed and orientation                                               |
| `~/input/measured_acceleration`  | `geometry_msgs/msg/AccelWithCovarianceStamped`   | Measured longitudinal acceleration                                               |
| `~/output/brake_defect_detected` | `std_msgs/msg/Bool`                              | True only while valid monitoring conditions hold and CUSUM exceeds the threshold |
| `~/output/filtered_residual`     | `std_msgs/msg/Float64`                           | Filtered missing-braking residual, in m/s²                                       |
| `~/output/cusum_statistic`       | `std_msgs/msg/Float64`                           | Upper-side CUSUM statistic, in m/s                                               |
| `/diagnostic`                    | `diagnostic_msgs/msg/DiagnosticArray`            | Fault level and all detector outputs in one diagnostic status                    |

The launch file maps the inputs to `/control/command/control_cmd`, `/control/command/actuation_cmd`, `/localization/kinematic_state`, and `/localization/acceleration`, matching the recorded vehicle topics. It publishes diagnostics on `/diagnostic` by default. Launch it with `ros2 launch autoware_brake_defect_detector brake_defect_detector.launch.xml`.

The `/diagnostic` status reports `brake_defect_detected`, `filtered_residual`, `cusum_statistic`, and `valid_condition` as key-value fields. Its level is `ERROR` for a detected defect, `STALE` when inputs are missing or invalid or the delay history is warming up, and `OK` when monitoring is active without a defect or inactive under normal guardrails.

## Detection

The command history is timestamped at reception. At each acceleration sample, the node looks up the command from `actuation_delay_sec` earlier and computes expected acceleration as `delayed_command + 9.81 * sin(pitch)`. Pitch is extracted from the odometry quaternion. The residual is **measured minus expected** acceleration. Since braking commands are negative, insufficient braking makes this residual positive. The upper-side CUSUM integrates the filtered residual above `cusum_drift_k` until it reaches `cusum_threshold_h`.

Detection requires the current and delayed commands to request deceleration, brake command within the configured range, speed above the minimum, and no active settling lockout. A fast command transition starts the lockout. Outside valid conditions, the CUSUM decays and the fault flag is false. Missing, stale, nonfinite, or time-reversed data reset the detector and publish a false flag with zero metrics. The command history must warm up for the configured delay before evaluation starts.

The default CUSUM threshold is `0.05 m/s`, selected to flag the confirmed October 1 case 3 defect in replay. This setting still needs evaluation against alarms outside the six event clips. Brake command units depend on the vehicle interface; the default `0.05`–`0.35` range covers the low-command region observed in those bags. The thresholds and delays are set in `config/brake_defect_detector.param.yaml`.
