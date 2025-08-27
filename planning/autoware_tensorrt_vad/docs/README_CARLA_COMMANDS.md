# CARLA Navigation Commands

## Command Definitions

The VAD model for CARLA generates trajectory predictions for different navigation commands. These commands are defined by the CARLA team:

```python
VOID = -1           # No command (defaults to LANEFOLLOW)
LEFT = 1            # Turn left at intersection
RIGHT = 2           # Turn right at intersection  
STRAIGHT = 3        # Go straight at intersection
LANEFOLLOW = 4      # Follow current lane
CHANGELANELEFT = 5  # Change to left lane
CHANGELANERIGHT = 6 # Change to right lane
```

## Model Output Modes

### 6-Mode Model (CARLA Tier4)
The 6-mode model outputs trajectories for different driving behaviors:
- **Mode 0**: LEFT - Turn left trajectory
- **Mode 1**: RIGHT - Turn right trajectory
- **Mode 2**: STRAIGHT - Go straight trajectory
- **Mode 3**: LANEFOLLOW - Lane following trajectory (default)
- **Mode 4**: CHANGELANELEFT - Left lane change trajectory
- **Mode 5**: CHANGELANERIGHT - Right lane change trajectory

### 3-Mode Model (Legacy)
The 3-mode model outputs simpler trajectories:
- **Mode 0**: RIGHT - Turn right trajectory
- **Mode 1**: LEFT - Turn left trajectory
- **Mode 2**: STRAIGHT - Go straight/forward trajectory

## Configuration

In the config files (`vad_carla_tier4.param.yaml`):
```yaml
model_params:
  default_command: 3  # Default to LANEFOLLOW mode for normal driving
```

The `default_command` parameter specifies which trajectory mode to use when no specific command is provided. For the 6-mode model, we default to mode 3 (LANEFOLLOW) for normal driving behavior.

## Automatic Mode Detection

The code automatically detects whether the model outputs 3 or 6 trajectory modes based on the output tensor size. This ensures compatibility with both model types without code changes.

## Command Selection Logic

The actual command used during runtime can come from:
1. **Default**: Uses the `default_command` from config (typically LANEFOLLOW)
2. **CARLA Planner**: If integrated with CARLA, uses the routing command from CARLA's waypoint planner
3. **User Override**: Can be overridden through ROS topics or parameters

## Safety Handling

If an invalid command index is requested (e.g., command 4 on a 3-mode model), the system automatically clamps to the nearest valid mode to prevent crashes.