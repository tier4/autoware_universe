# leader_election_converter

## Overview

The leader election converter node is responsible for relaying UDP packets and ROS2 topics between the leader_election invoked by systemd and Autoware executed on ROS2.

## availability converter

The availability converter subscribes `/system/operation_mode/availability` and `/vehicle/status/mrm_state`, adds them together into a structure called `Availability` and sends it as a udp packet.

### Interface

| Interface Type | Interface Name                        | Data Type                                          | Description                   |
| -------------- | ------------------------------------- | -------------------------------------------------- | ----------------------------- |
| subscriber     | `/system/operation_mode/availability` | `tier4_system_msgs/msg/OperationModeAvailability`  | Usable behavior of the ego.   |
| subscriber     | `/vehicle/status/mrm_state`           | `autoware_auto_vehicle_msgs/msg/ControlModeReport` | Ego control mode.             |
| udp sender     | none                                  | `struct Availability`                              | Combination of the above two. |

## mrm converter

The mrm converter subscribes `/system/fail_safe/mrm_state` into a structure called `MrmState` and sends it as a UDP packet.
In addition, it receives a udp packet`MrmState` and publish `/system/mrm_request`.

### Interface

| Interface Type | Interface Name                 | Data Type                           | Description              |
| -------------- | ------------------------------ | ----------------------------------- | ------------------------ |
| subscriber     | `/system/fail_safe/mrm_state`  | `tier4_system_msgs/msg/MrmState`    | MRM status of each ECU.  |
| udp sender     | none                           | `struct MrmState`                   | Same as above.           |
| publisher      | `/system/election/mrm_request` | `tier4_system_msgs/msg/MrmBehavior` | Request of MRM behavior. |
| udp receiver   | none                           | `struct MrmRequest`                 | Same as above.           |

## log converter

The log　converter receive udp packets into a structure called `ElectionCommunication` and `ElectionStatus`, and publish `/system/election/communication`,
`/system/election/status`, and `/system/fail_safe/over_all/mrm_state`.

### Interface

| Interface Type | Interface Name                         | Data Type                                     | Description                    |
| -------------- | -------------------------------------- | --------------------------------------------- | ------------------------------ |
| udp receiver   | none                                   | `struct ElectionCommunication`                | messages among election nodes. |
| udp receiver   | none                                   | `struct ElectionStatus`                       | Leader Election status.        |
| publisher      | `/system/election/communication`       | `tier4_system_msgs/msg/ElectionCommunication` | messages among election nodes. |
| publisher      | `/system/election/status`              | `tier4_system_msgs/msg/MrmState`              | Leader Election status.        |
| publisher      | `/system/fail_safe/over_all/mrm_state` | `autoware_adapi_v1_msgs/msg/mrm_state`        | System-wide MRM status.        |

## Parameters

{{ json_to_markdown("system/leader_election_converter/schema/leader_election_converter.schema.json") }}
