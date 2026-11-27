# Predicted Trajectory Validator

`predicted_trajectory_validator` evaluates the effect of the control predicted trajectory on the surroundings. It runs one cycle each time a predicted trajectory is received.

| Check                        | Class                                | Output                                                 |
| ---------------------------- | ------------------------------------ | ------------------------------------------------------ |
| Uncrossable bound departure  | `UncrossableBoundDepartureValidator` | `uncrossable_bound_departure_result`, diagnostics, MRM |
| Collision                    | `CollisionValidator`                 | hazard alert only                                      |
| Hazard alert (sector, latch) | `HazardWarning`                      | `~/output/hazard_alert`                                |

When the vehicle is not in autonomous mode (`mode != AUTONOMOUS` or `!is_autoware_control_enabled`), every `is_valid` is overwritten with `true` after the checks run. The hazard alert is not overwritten.

## Inputs / Outputs

### Input

| Name                             | Type                                      | Description                                     |
| -------------------------------- | ----------------------------------------- | ----------------------------------------------- |
| `~/input/predicted_trajectory`   | autoware_planning_msgs/Trajectory         | predicted trajectory of the controller; trigger |
| `~/input/kinematics`             | nav_msgs/Odometry                         | ego pose and twist                              |
| `~/input/measured_acceleration`  | geometry_msgs/AccelWithCovarianceStamped  | measured acceleration                           |
| `~/input/operational_mode_state` | autoware_adapi_v1_msgs/OperationModeState | operation mode                                  |
| `~/input/lanelet2_map`           | autoware_map_msgs/LaneletMapBin           | lanelet map (transient local)                   |
| `~/input/objects`                | autoware_perception_msgs/PredictedObjects | objects for the collision check                 |
| `~/input/pointcloud`             | sensor_msgs/PointCloud2                   | obstacle pointcloud for the collision check     |

### Output

| Name                                       | Type                                                                       | Description                                                   |
| ------------------------------------------ | -------------------------------------------------------------------------- | ------------------------------------------------------------- |
| `~/output/validation_status`               | autoware_predicted_trajectory_validator/PredictedTrajectoryValidatorStatus | result and invalid count of each check                        |
| `~/output/hazard_alert`                    | autoware_internal_control_msgs/HazardAlert                                 | hazard level of each sector                                   |
| `~/output/markers`                         | visualization_msgs/MarkerArray                                             | boundary departure markers                                    |
| `~/virtual_wall`                           | visualization_msgs/MarkerArray                                             | virtual wall while a check is invalid                         |
| `~/debug/hazard/markers`                   | visualization_msgs/MarkerArray                                             | hazard sources and sector wedges                              |
| `~/debug/processing_time_ms`               | autoware_internal_debug_msgs/Float64Stamped                                | processing time of one cycle                                  |
| `~/debug/hazard/processing_time_ms`        | autoware_internal_debug_msgs/Float64Stamped                                | processing time of the hazard alert                           |
| `~/debug/hazard/processing_time_detail_ms` | autoware_utils_debug/ProcessingTimeDetail                                  | processing time breakdown of the hazard alert                 |
| `/diagnostics`                             | diagnostic_msgs/DiagnosticArray                                            | `predicted_trajectory_validator: uncrossable_bound_departure` |

## Parameters

Definitions are in [param/predicted_trajectory_validator_parameters.yaml](param/predicted_trajectory_validator_parameters.yaml) and default values are in [config/predicted_trajectory_validator.param.yaml](config/predicted_trajectory_validator.param.yaml).
