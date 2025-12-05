# autoware_creep_guidance_interface

## Overview

The `autoware_creep_guidance_interface` package provides an interface for managing creep guidance status and commands in autonomous driving behavior planning modules. This interface enables behavior modules (such as crosswalk, intersection, and intersection occlusion modules) to coordinate creep maneuvers with external systems.

## Usage

### Basic Integration Example

```cpp
#include "autoware/creep_guidance_interface/creep_guidance_interface.hpp"
#include <tier4_creep_guidance_msgs/msg/state.hpp>

class YourBehaviorModule : public rclcpp::Node
{
public:
  YourBehaviorModule(const int64_t module_id) : Node("your_module")
  {
    // Initialize the creep guidance interface with module name
    creep_interface_ = std::make_shared<autoware::creep_guidance_interface::CreepGuidanceInterface>(
      this, "crosswalk");
    creep_interface_->add(module_id);
  }

  ~YourBehaviorModule()
  {
    creep_interface_->remove(module_id);
  }

  void planPath()
  {
    creep_interface_->update_distance(module_id, getStartDistance(), getFinishDistance());

    if (creep_interface_->recieved_activation_command(module_id)) {
      // Execute creep maneuver
      executeCreepManeuver();
      creep_interface_->update_state(module_id, tier4_creep_guidance_msgs::msg::State::ACTIVATED);
    }

    // Publish status
    creep_interface_->publish_creep_status_array();
  }

private:
  std::shared_ptr<autoware::creep_guidance_interface::CreepGuidanceInterface> creep_interface_;
};
```

## API Reference

### Constructor

```cpp
CreepGuidanceInterface(rclcpp::Node * node, const std::string & name)
```

Creates a new interface instance for the specified module.

**Parameters:**

- `node`: Pointer to the ROS 2 node
- `name`: Module name (supported values: "crosswalk", "intersection", "intersection_occlusion")

### Methods

#### `void add(const int64_t id)`

Register a new creep guidance status entry with the given ID.

#### `bool remove(const int64_t id)`

Remove a registered creep guidance status entry. Returns `true` if the entry was found and removed, `false` otherwise.

#### `void update_distance(const int64_t id, const double start_distance, const double finish_distance)`

Update the distance information for a registered entry.

- `start_distance`: Distance from the ego vehicle to the start point
- `finish_distance`: Distance from the ego vehicle to the finish point

#### `void update_state(const int64_t id, const uint8_t state)`

Update the state of a registered entry. Use values from `tier4_creep_guidance_msgs::msg::State` enum.

#### `bool recieved_activation_command(const int64_t id) const`

Check if an activation command has been received for a registered entry. Returns `true` if activation command has been received, `false` otherwise.

#### `void publish_creep_status_array() const`

Publish the current creep status array to the ROS 2 topic.

## ROS 2 Interface

### Topics

#### Publishers

- `/planning/creep_guidance_status/{name}` (`tier4_creep_guidance_msgs/msg/CreepStatusArray`)
  - Publishes the array of all registered creep guidance statuses
  - QoS: Default (queue size: 1)
  - `{name}` is replaced with the module name provided in the constructor (e.g., "crosswalk", "intersection", "intersection_occlusion")

### Services

#### Servers

- `/planning/creep_guidance_commands/{name}` (`tier4_creep_guidance_msgs/srv/CreepTriggerCommand`)
  - Receives commands to trigger creep maneuvers
  - Request contains an array of commands, each with `id`, `module`, and `command`
  - Response contains an array of responses, each with `id`, `module`, and `success` status
  - `{name}` is replaced with the module name provided in the constructor

## Module Types

The interface supports the following module types (defined in `tier4_creep_guidance_msgs/msg/Module`):

- `NONE`: 0 - Default/unspecified module type
- `CROSSWALK`: 1 - For pedestrian crosswalk scenarios
- `INTERSECTION`: 2 - For intersection scenarios
- `INTERSECTION_OCCLUSION`: 3 - For intersection scenarios with occlusion handling

## Command Types

The interface supports the following command types (defined in `tier4_creep_guidance_msgs/msg/Command`):

- `DEACTIVATE`: 0 - Deactivate creep maneuver
- `ACTIVATE`: 1 - Activate creep maneuver

## State Types

The interface supports the following state types (defined in `tier4_creep_guidance_msgs/msg/State`):

- `DEACTIVATED`: 0 - Creep maneuver is deactivated
- `ACTIVATED`: 1 - Creep maneuver is activated
