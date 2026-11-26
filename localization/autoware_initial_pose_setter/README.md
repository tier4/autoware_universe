# autoware_initial_pose_setter

Sets the initial pose of the vehicle from the pose estimated by the `autoware_ar_tag_based_localizer`
(ArUco markers placed at fixed positions, for example in an underground parking lot).

## Behavior

- While the Autoware localization state is not `INITIALIZED`, the node calls `/localization/initialize`
  once per `retry_period` (1 Hz) with the latest AR tag pose and `method = DIRECT`.
  - No request is sent while the state is `INITIALIZING` or while a previous request is still waiting for its response.
  - If no fresh pose is available (none received, or older than `pose_timeout`), no request is sent.
- Once the state is `INITIALIZED`, the node does nothing.
- When `/ar_tag_based_localizer/initialize` is called, the node sends exactly one request to
  `/localization/initialize` with the latest AR tag pose regardless of the state.
  The response only tells whether the request was sent. The result of the initialization is logged.

## Interface

| Kind         | Name                                                | Type                                                    |
| ------------ | --------------------------------------------------- | ------------------------------------------------------- |
| Subscription | `/localization/initialization_state`                | `autoware_adapi_v1_msgs/msg/LocalizationInitializationState` |
| Subscription | `/ar_tag_based_localizer/output/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped`           |
| Service      | `/ar_tag_based_localizer/initialize`                | `std_srvs/srv/Trigger`                                  |
| Client       | `/localization/initialize`                          | `autoware_localization_msgs/srv/InitializeLocalization` |

## Parameters

| Name           | Type   | Default | Description                                                              |
| -------------- | ------ | ------- | ------------------------------------------------------------------------ |
| `retry_period` | double | 1.0     | Period of the retry loop [s]                                             |
| `pose_timeout` | double | 5.0     | A pose older than this is not used [s]. Set 0.0 or less to disable.      |

## Usage

```bash
ros2 launch autoware_initial_pose_setter initial_pose_setter.launch.xml
# initialize once, regardless of the state
ros2 service call /ar_tag_based_localizer/initialize std_srvs/srv/Trigger
```

## Notes

- `autoware_ar_tag_based_localizer` discards a detection whose pose is more than `ekf_position_tolerance`
  (10 m by default) away from the `input_ekf_pose`, and publishes nothing until it receives an `input_ekf_pose`.
- `pose_initializer` refuses the request if the vehicle is not stopped (`stop_check_enabled`).
