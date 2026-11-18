# Control Validator

The `control_validator` is a module that checks the validity of the output of the control component. The status of the validation can be viewed in the `/diagnostics` topic.

![control_validator](./image/control_validator.drawio.svg)

## Supported features

The following features are supported for the validation and can have thresholds set by parameters.
The listed features below does not always correspond to the latest implementation.

| Description                                                                        | Arguments                                                                                       |                  Diagnostic equation                  |
| ---------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- | :---------------------------------------------------: |
| Inverse velocity: Measured velocity has a different sign from the target velocity. | measured velocity $v$, target velocity $\hat{v}$, and velocity parameter $c$                    |      $v \hat{v} < 0, \quad \lvert v \rvert > c$       |
| Overspeed: Measured speed exceeds target speed significantly.                      | measured velocity $v$, target velocity $\hat{v}$, ratio parameter $r$, and offset parameter $c$ | $\lvert v \rvert > (1 + r) \lvert \hat{v} \rvert + c$ |
| Overrun estimation: estimate overrun even if decelerate by assumed rate.           | assumed deceleration, assumed delay                                                             |                                                       |

- **Lateral jerk** : invalid when the lateral jerk exceeds the configured threshold. The validation uses the vehicle's velocity and steering angle rate to calculate the resulting lateral jerk. The calculation assumes constant velocity (acceleration is zero).
- **Deviation check between reference trajectory and predicted trajectory** : invalid when the largest deviation between the predicted trajectory and reference trajectory is greater than the given threshold.
- **Yaw deviation**: invalid when the difference between the yaw of the ego vehicle and the nearest (interpolated) trajectory yaw is greater than the given threshold.
  - 2 thresholds are implemented, one to trigger a warning diagnostic, and one to trigger an error diagnostic.
- **Boundary departure**: invalid when the control predicted trajectory would drive the ego footprint across an uncrossable map boundary (e.g. `road_border`). The check reuses the shared `autoware_boundary_departure_checker` library and applies braking-distance / time-to-departure logic with ON/OFF hysteresis before reporting a critical departure. Requires the lanelet2 map (`~/input/lanelet2_map`); it is skipped until the map is received. Can be disabled via `boundary_departure.enable`.

![trajectory_deviation](./image/trajectory_deviation.drawio.svg)

## Inputs/Outputs

### Inputs

The `control_validator` takes in the following inputs:

| Name                           | Type                              | Description                                                                    |
| ------------------------------ | --------------------------------- | ------------------------------------------------------------------------------ |
| `~/input/kinematics`           | nav_msgs/Odometry                 | ego pose and twist                                                             |
| `~/input/reference_trajectory` | autoware_planning_msgs/Trajectory | reference trajectory which is outputted from planning module to to be followed |
| `~/input/predicted_trajectory` | autoware_planning_msgs/Trajectory | predicted trajectory which is outputted from control module                    |
| `~/input/lanelet2_map`         | autoware_map_msgs/LaneletMapBin   | vector map, used by the boundary departure check                               |

### Outputs

It outputs the following:

| Name                         | Type                                     | Description                                                               |
| ---------------------------- | ---------------------------------------- | ------------------------------------------------------------------------- |
| `~/output/validation_status` | control_validator/ControlValidatorStatus | validator status to inform the reason why the trajectory is valid/invalid |
| `/diagnostics`               | diagnostic_msgs/DiagnosticStatus         | diagnostics to report errors                                              |

## Parameters

The following parameters can be set for the `control_validator`:

### System parameters

| Name                         | Type | Description                                                                                                                                                                                                                                | Default value |
| :--------------------------- | :--- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `publish_diag`               | bool | if true, diagnostics msg is published.                                                                                                                                                                                                     | true          |
| `diag_error_count_threshold` | int  | the Diag will be set to ERROR when the number of consecutive invalid trajectory exceeds this threshold. (For example, threshold = 1 means, even if the trajectory is invalid, the Diag will not be ERROR if the next trajectory is valid.) | true          |
| `display_on_terminal`        | bool | show error msg on terminal                                                                                                                                                                                                                 | true          |

### Algorithm parameters

#### Thresholds

The input trajectory is detected as invalid if the index exceeds the following thresholds.

| Name                                      | Type   | Description                                                                                                       | Default value |
| :---------------------------------------- | :----- | :---------------------------------------------------------------------------------------------------------------- | :------------ |
| `thresholds.max_distance_deviation`       | double | invalid threshold of the max distance deviation between the predicted path and the reference trajectory [m]       | 1.0           |
| `thresholds.lateral_jerk`                 | double | invalid threshold of the lateral jerk for steering rate validation [m/s^3]                                        | 10.0          |
| `thresholds.rolling_back_velocity`        | double | threshold velocity to validate the vehicle velocity [m/s]                                                         | 0.5           |
| `thresholds.over_velocity_offset`         | double | threshold velocity offset to validate the vehicle velocity [m/s]                                                  | 2.0           |
| `thresholds.over_velocity_ratio`          | double | threshold ratio to validate the vehicle velocity [*]                                                              | 0.2           |
| `thresholds.overrun_stop_point_dist`      | double | threshold distance to overrun stop point [m]                                                                      | 0.8           |
| `thresholds.acc_error_offset`             | double | threshold ratio to validate the vehicle acceleration [*]                                                          | 0.8           |
| `thresholds.acc_error_scale`              | double | threshold acceleration to validate the vehicle acceleration [m]                                                   | 0.2           |
| `thresholds.will_overrun_stop_point_dist` | double | threshold distance to overrun stop point [m]                                                                      | 1.0           |
| `thresholds.assumed_limit_acc`            | double | assumed acceleration for over run estimation [m]                                                                  | 5.0           |
| `thresholds.assumed_delay_time`           | double | assumed delay for over run estimation [m]                                                                         | 0.2           |
| `thresholds.yaw_deviation_error`          | double | threshold angle to validate the vehicle yaw related to the nearest trajectory yaw [rad]                           | 1.0           |
| `thresholds.yaw_deviation_warn`           | double | threshold angle to trigger a WARN diagnostic [rad]                                                                | 0.5           |
| `over_velocity.vel_lpf_gain`              | double | low-pass filter gain for filtering vehicle and target velocity [*] (time constant 2.0s at 30msec sampling period) | 0.985         |

#### Boundary departure

| Name                                            | Type         | Description                                                                       | Default value   |
| :---------------------------------------------- | :----------- | :-------------------------------------------------------------------------------- | :-------------- |
| `boundary_departure.enable`                     | bool         | if true, run the uncrossable boundary departure check on the predicted trajectory | true            |
| `boundary_departure.boundary_types`             | string array | lanelet linestring types treated as uncrossable boundaries                        | ["road_border"] |
| `boundary_departure.lateral_margin_m`           | double       | lateral margin from the boundary [m]                                              | 0.01            |
| `boundary_departure.longitudinal_margin_m`      | double       | longitudinal margin from the boundary [m]                                         | 1.0             |
| `boundary_departure.max_deceleration_mps2`      | double       | maximum deceleration used for the braking-distance estimation [m/s^2]             | -4.0            |
| `boundary_departure.max_jerk_mps3`              | double       | maximum jerk used for the braking-distance estimation [m/s^3]                     | -5.0            |
| `boundary_departure.brake_delay_s`              | double       | assumed brake delay [s]                                                           | 1.0             |
| `boundary_departure.time_to_departure_cutoff_s` | double       | time-to-departure cutoff beyond which a departure is not treated as critical [s]  | 2.0             |
| `boundary_departure.on_time_buffer_s`           | double       | continuous-violation time required to activate a critical departure [s]           | 0.20            |
| `boundary_departure.off_time_buffer_s`          | double       | continuous-safety time required to deactivate a critical departure [s]            | 0.10            |
| `boundary_departure.enable_developer_marker`    | bool         | if true, publish developer debug markers from the boundary departure checker      | false           |
