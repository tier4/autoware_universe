# Integration Guide: Direction Change Module

## Overview

This guide explains how to integrate the `direction_change` module into the behavior path planner. The module handles direction changes by detecting cusp points and reversing path point orientations.

## Step 1: Build the Module

The module should be automatically detected by the build system. Build the workspace:

```bash
cd /home/tejaemmey/ws/workflows/pilot-auto.x1
colcon build --packages-select autoware_behavior_path_direction_change_module
```

## Step 2: Register the Module in Behavior Path Planner

The module is automatically registered via the `plugins.xml` file. The plugin system will discover it at runtime.

## Step 3: Add Module to Configuration

Add the module to your behavior path planner launch file or parameter file. The module name is:

```
direction_change
```

### Example Configuration

In your behavior path planner parameter file, ensure the module is included in the module list:

```yaml
behavior_path_planner:
  # ... other parameters ...
  
  # Module configuration
  direction_change:
    enable_cusp_detection: true
    enable_reverse_following: true
    # ... other parameters (see config/direction_change.param.yaml)
```

## Step 4: Verify Integration

1. **Check Plugin Registration**: The module should appear in the plugin list when the behavior path planner starts.

2. **Test Activation**: The module activates when:
   - `direction_change_area` tag is present in lanelets along the path
   - Module processes the path and reverses orientations at cusp points if detected

3. **Monitor Logs**: Check ROS2 logs for module activation messages:
   ```bash
   ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/markers
   ```

## Step 5: Customize Parameters

Edit `config/direction_change.param.yaml` to adjust:
- Cusp detection angle thresholds
- Direction change parameters
- Path generation parameters

## Troubleshooting

### Module Not Activating

1. **Check Plugin Registration**: Verify `plugins.xml` is in the correct location
2. **Check Parameters**: Ensure module parameters are properly loaded
3. **Check Path**: Verify the input path contains cusp points (direction changes)

### Compilation Errors

1. **Missing Dependencies**: Ensure all dependencies in `package.xml` are installed
2. **Include Paths**: Verify include paths in `CMakeLists.txt` are correct

### Runtime Errors

1. **Check Logs**: Review ROS2 logs for specific error messages
2. **Verify Data**: Ensure planner data (odometry, path, etc.) is available
3. **State Transitions**: Monitor module state transitions in debug output

## Module Execution Flow

1. **Tag Detection**: Behavior path planner checks for `direction_change_area` tag in lanelets
2. **Activation**: Module activates when tag is present (regardless of cusp points)
3. **Cusp Detection**: Module detects cusp points using angle-based detection
4. **Orientation Reversal**: Module reverses path point orientations (yaw) at cusp points
5. **Output**: Modified path with reversed orientations is passed to downstream modules

## Integration with Other Modules

This module works in conjunction with:
- **Behavior Velocity Planner**: Receives path with reversed orientations for geometry-based direction detection
- **Trajectory Follower**: Uses `isDrivingForward()` which works with orientation reversal
- **Other Path Modules**: Can be combined with lane change, avoidance, etc.

## Map Requirements

**Critical**: The module requires the `direction_change_area` tag in lanelet map:

```xml
<lanelet id="123">
  <tag k="direction_change_area" v="true"/>
  <!-- ... other lanelet data ... -->
</lanelet>
```

Without this tag, the module will not activate.

## Next Steps

1. Test with sample paths containing cusp points
2. Tune parameters for your specific use case
3. Add custom logic in `scene.cpp` if needed
4. Extend debug visualization if required

