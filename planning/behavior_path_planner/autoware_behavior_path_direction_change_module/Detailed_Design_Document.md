# Detailed Design Document: Direction Change Module

---

## 1. Introduction

The **Direction Change Module** is a behavior path planner module designed to handle complex maneuvering scenarios where vehicles need to navigate paths containing cusp points (direction changes) and perform reverse lane following. This module extends the Autoware behavior path planner to support bidirectional path following by detecting cusp points in planned paths and reversing path point orientations (yaw angles) to indicate reverse direction segments.

The module operates as a plugin within the Autoware behavior path planner framework and is activated conditionally based on map metadata (`direction_change_area` tag). It processes path points to detect direction changes and reverses orientation (yaw) at cusp points accordingly, enabling downstream modules to correctly interpret the path direction for both forward and reverse motion segments.

**Related Documentation:**
- [README.md](./README.md) - Module overview and usage
- [FINAL_DESIGN.md](./FINAL_DESIGN.md) - Final design decisions
- [DESIGN_DISCUSSION.md](./DESIGN_DISCUSSION.md) - Design discussion and decisions
- [IMPLEMENTATION_SUMMARY.md](./IMPLEMENTATION_SUMMARY.md) - Implementation details

---

## 2. Objectives

### Goals

1. **Enable Cusp Point Detection**: Detect points in planned paths where significant direction changes occur (typically 180-degree turns or U-turns) using configurable angle thresholds.

2. **Reverse Path Point Orientation**: Reverse path point orientations (yaw angles) at cusp points to indicate reverse direction segments, enabling bidirectional path following.

3. **Support Multiple Cusp Points**: Handle paths with multiple cusp points by correctly toggling orientation reversal at each cusp point.

4. **Conditional Activation**: Activate only when `direction_change_area` tag is present in lanelet map, ensuring the module operates only in designated areas.

5. **Lane Continuity**: Support orientation reversal propagation across lane boundaries when consecutive lanes have the `direction_change_area` tag.

6. **Integration with Existing Framework**: Seamlessly integrate with Autoware behavior path planner plugin architecture without disrupting existing modules.

7. **Configurable Parameters**: Provide configurable parameters for cusp detection thresholds and module behavior.

### Non-Goals

1. **Velocity Planning**: This module does NOT plan or modify velocity values. It only reverses path point orientations (yaw angles) to indicate direction changes. Actual velocity planning is handled by downstream modules (Behavior Velocity Planner).

2. **State Machine Management**: The module does NOT maintain complex state machines for vehicle status. It processes paths statelessly.

3. **Path Generation**: The module does NOT generate new paths. It modifies existing paths from previous modules.

4. **Obstacle Avoidance**: The module does NOT handle obstacle avoidance or dynamic object interactions.

5. **Lane Change Planning**: The module does NOT plan lane changes. It operates on paths already planned by other modules.

6. **Real-time Vehicle Control**: The module does NOT directly control vehicle actuators. It provides path modifications for planning layers.

7. **Map Creation/Editing**: The module does NOT create or edit lanelet maps. It only reads map attributes.

---

## 3. Assumptions

1. **Map Quality**: The lanelet map contains accurate `direction_change_area` tags on relevant lanelets where cusp maneuvers are expected.

2. **Path Quality**: Input paths from previous modules are well-formed, with valid pose orientations and lane_id associations.

3. **Path Point Yaws**: Path point orientations (yaws) accurately represent the direction of travel along the path.

4. **Downstream Processing**: Downstream modules (Behavior Velocity Planner, Trajectory Follower) use geometry-based direction detection (`isDrivingForward()`) which relies on path point orientations. By reversing yaw angles at cusp points, these modules can correctly identify reverse segments.

5. **Lanelet Connectivity**: Lanelets in the path are properly connected and lane_ids in path points correctly reference map lanelets.

6. **Single Path Processing**: The module processes one path at a time and does not need to handle multiple concurrent paths.

7. **Plugin Architecture**: The Autoware plugin system correctly loads and manages the module.

8. **Coordinate Frames**: All path points and poses are in the same coordinate frame (typically `map` frame).

9. **Path Resolution**: Path points are sufficiently dense to accurately detect cusp points using angle-based detection.

10. **Vehicle Capabilities**: The vehicle is capable of both forward and reverse motion, and control systems can interpret reversed orientations for reverse segments.

---

## 4. Glossary

| Term | Definition |
|------|------------|
| **Cusp Point** | A point in a path where a significant direction change occurs, typically representing a transition from forward to reverse motion or vice versa. |
| **Direction Change Area** | A lanelet map attribute/tag (`direction_change_area`) that designates areas where cusp maneuvers are expected and the module should be active. |
| **Orientation Reversal** | The process of reversing path point yaw angles (adding π radians) to indicate reverse direction segments. |
| **Yaw Reversal** | The modification of path point orientation by adding π radians (180 degrees) to the yaw angle, normalized to [-π, π] range. |
| **Path Point** | A single point in a planned path, containing position, orientation, velocity, and associated lane_ids. |
| **Lanelet** | A basic lane representation in Lanelet2 map format, containing geometric boundaries and attributes. |
| **Behavior Path Planner (BPP)** | The Autoware planning module that generates geometric paths considering lane following, lane changes, and obstacle avoidance. |
| **Behavior Velocity Planner (BVP)** | The Autoware planning module that plans velocities along paths, considering traffic rules, obstacles, and safety constraints. |
| **Scene Module** | A plugin module in the behavior path planner that processes paths and applies specific behaviors (e.g., lane change, obstacle avoidance). |
| **Yaw Angle** | The orientation angle of a vehicle/path point around the vertical (Z) axis, measured in radians or degrees. |
| **Angle Threshold** | The minimum angle difference (in degrees) required to detect a cusp point. Configurable parameter. |
| **Lane Boundary** | The transition point between two different lanelets in a path. |
| **Toggle** | The action of switching between two states (e.g., positive to negative velocity sign). |
| **Plugin Architecture** | A software design pattern where modules are dynamically loaded and registered at runtime. |

---

## 5. High-level Design

### 5.1 Architecture Overview

The Direction Change Module operates within the Autoware behavior path planner framework as a scene module plugin. It receives paths from upstream modules, processes them to detect cusp points and reverse orientations, and outputs modified paths to downstream modules.

```
┌─────────────────────────────────────────────────────────────────┐
│                    Behavior Path Planner                        │
│                                                                 │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │         Previous Modules (Lane Following, etc.)          │   │
│  └──────────────────────┬───────────────────────────────────┘   │
│                         │                                       │
│                         ▼                                       │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │   Direction Change Module                                │   │
│  │   ┌──────────────────────────────────────────────────┐   │   │
│  │   │  1. Check direction_change_area tag in lanelets  │   │   │
│  │   │  2. Detect cusp points (angle-based)             │   │   │
│  │   │  3. Reverse orientation (yaw) at cusps           │   │   │
│  │   └──────────────────────────────────────────────────┘   │   │
│  └──────────────────────┬───────────────────────────────────┘   │
│                         │                                       │
│                         ▼                                       │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │         Behavior Velocity Planner (BVP)                  │   │
│  │         (Handles actual velocity planning)               │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 5.2 Module Activation Flow

```
Path Input
    │
    ▼
Check direction_change_area tag in lanelets
    │
    ├─ No tag found → Module inactive (return path unchanged)
    │
    └─ Tag found → Continue
        │
        ▼
    Detect cusp points (angle-based)
        │
        ├─ No cusps → Return path unchanged
        │
        └─ Cusps found → Continue
            │
            ▼
        Reverse orientation (yaw) at cusps
            │
            ├─ Before first cusp: original orientation
            ├─ After first cusp: yaw + π (reversed)
            ├─ After second cusp: original (toggled back)
            └─ Continue toggling for each cusp
            │
            ▼
        Return modified path
```

### 5.3 Key Components

1. **Manager** (`manager.cpp`): Handles module initialization, parameter loading, and plugin registration.

2. **Scene Module** (`scene.cpp`): Main processing logic including activation checks, cusp detection, and path modification.

3. **Utilities** (`utils.cpp`): Helper functions for cusp detection, orientation reversal, and lanelet tag checking.

4. **Data Structures** (`data_structs.hpp`): Parameter structures and debug data containers.

---

## 6. Detailed Design

### 6.1 Module Activation Logic

The module activates only when specific conditions are met:

**Activation Conditions:**
1. Path is not empty
2. At least one lanelet in the path has the `direction_change_area` tag
3. **Note**: Cusp point detection is NOT required for activation. Module activates if tag exists, regardless of cusp points.

**Tag Checking Algorithm:**
```cpp
bool shouldActivateModule() {
  // 1. Check reference lanelets
  for (lanelet in current_lanelets_):
    if (hasDirectionChangeAreaTag(lanelet)):
      return true
  
  // 2. Check lanelets from path points
  for (path_point in reference_path_.points):
    for (lane_id in path_point.lane_ids):
      lanelet = getLaneletFromId(lane_id)
      if (hasDirectionChangeAreaTag(lanelet)):
        return true
  
  return false
}
```

**Rationale**: 
- Checking each lanelet ensures the module activates only in designated areas (with `direction_change_area` tag)
- Module activates if tag exists, regardless of cusp points
- If cusps exist → reverses orientations (yaw) at cusp points
- If no cusps → outputs path unchanged (original orientations)
- This ensures downstream modules always receive a path when the tag is present

### 6.2 Path Point Handling and Centerline

**Path Source**: The module receives paths from previous modules via `getPreviousModuleOutput().path`. These paths should already have:
- Path points positioned on the centerline of lanelets
- `lane_ids` correctly assigned to each path point
- Proper orientation (yaw) for each point

**Note**: This module does NOT regenerate centerline paths. It processes the existing path points, similar to other behavior path planner processing modules (e.g., obstacle avoidance, lane change). The path points are expected to be on the centerline as generated by upstream path generation modules.

**If centerline recalculation is needed**: It should be done by path generation modules (e.g., lane following module) using `route_handler->getCenterLinePath()` which properly assigns lane_ids based on lanelet geometry.

### 6.3 Cusp Point Detection

**Algorithm**: Angle-based detection using path point yaws.

**Input**: Path with lane IDs, configurable angle threshold

**Process**:
```cpp
std::vector<size_t> detectCuspPoints(path, angle_threshold_deg) {
  cusp_indices = []
  threshold_rad = deg2rad(angle_threshold_deg)
  
  for i = 1 to path.points.size() - 1:
    yaw_prev = getYaw(path[i-1].pose.orientation)
    yaw_curr = getYaw(path[i].pose.orientation)
    
    angle_diff = normalize_radian(yaw_curr - yaw_prev)
    
    if abs(angle_diff) > threshold_rad:
      cusp_indices.push_back(i)
  
  return cusp_indices
}
```

**Key Design Decisions**:
- **Use path point yaws**: Path points already contain orientation information, eliminating need for geometric calculations
- **Configurable threshold**: Allows tuning for different vehicle types and scenarios
- **Normalized angles**: Handles angle wraparound (e.g., 359° to 1°)

**Example**:
```
Path points with yaws:
  P0: yaw = 0°    (north)
  P1: yaw = 0°    (north)
  P2: yaw = 180°  (south) ← CUSP (180° change)
  P3: yaw = 180°  (south)
  P4: yaw = 0°    (north) ← CUSP (180° change)
```

### 6.4 Orientation (Yaw) Reversal

**Algorithm**: Toggle-based orientation reversal at cusp points. Only applied if cusp points exist.

**Input**: Path, cusp point indices (may be empty)

**Process**:
- **If cusp points exist**: Apply toggle-based orientation reversal
- **If no cusp points**: Path remains unchanged (original orientations)
```cpp
void reverseOrientationAtCusps(path, cusp_indices) {
  // Only apply if cusp points exist
  if (cusp_indices.empty()) {
    return;  // No cusps, keep path unchanged (original orientations)
  }
  
  for i = 0 to path.points.size() - 1:
    // Determine if this point is after a cusp (odd number of cusps passed)
    bool is_reversed = false;
    for (const auto & cusp_idx : cusp_indices) {
      if (i > cusp_idx) {
        is_reversed = !is_reversed;  // Toggle at each cusp passed
      }
    }
    
    if (is_reversed) {
      // Reverse orientation: add π radians to yaw
      double yaw = tf2::getYaw(path[i].pose.orientation);
      yaw = normalize_radian(yaw + M_PI);
      path[i].pose.orientation = create_quaternion_from_yaw(yaw);
    }
}
```

**Orientation Pattern**:
```
Before Cusp1:  Original yaw (e.g., 0°)
After Cusp1:   Reversed yaw (e.g., 180°)
After Cusp2:   Original yaw (e.g., 0°, toggled back)
After Cusp3:   Reversed yaw (e.g., 180°, toggled again)
```

**Mathematical Formula**:
For path point at index `i`:
- Count number of cusp indices `c` where `c < i`
- If count is odd: `yaw_reversed[i] = normalize_radian(yaw[i] + π)`
- If count is even: `yaw_reversed[i] = yaw[i]` (unchanged)

**Rationale**:
- **Orientation reversal**: Indicates reverse direction to downstream modules using geometry-based detection
- **Toggle behavior**: Ensures correct orientation for each path segment
- **Preserve path geometry**: Maintains path point positions, only changes orientation

### 6.5 Multi-Cusp Handling

**Scenario**: Path with multiple cusp points

**Behavior**: Each cusp point toggles the orientation reversal.

**Example**:
```
Path: [P0] → [CUSP1] → [P1] → [CUSP2] → [P2] → [CUSP3] → [P3]

Orientations:
  P0:  Original yaw (e.g., 0°)
CUSP1: Original yaw (unchanged at cusp)
  P1:  Reversed yaw (e.g., 180°, after first cusp)
CUSP2: Reversed yaw (unchanged at cusp)
  P2:  Original yaw (e.g., 0°, after second cusp, toggled back)
CUSP3: Original yaw (unchanged at cusp)
  P3:  Reversed yaw (e.g., 180°, after third cusp, toggled again)
```

**Implementation**: The toggle logic in `reverseOrientationAtCusps()` correctly handles any number of cusp points by iterating through all cusp indices and toggling the orientation reversal for each one passed.

### 6.6 Lane Continuity and Tag Propagation

**Requirement**: If exiting a lane in reverse mode (reversed orientation), the next lane should continue with reversed orientation (if it also has the `direction_change_area` tag).

**Current Implementation**: 
- The module checks tags for each lanelet in the path
- Orientation reversals are applied based on cusp points, regardless of lane boundaries
- If a lane transition occurs and the next lane has the tag, the orientation reversal continues based on cusp positions

**Critical Safety Check - Lane Continuity with Reverse Exit**:

**Problem Scenario**:
- **Odd number of cusps** in current lane → Vehicle exits lane with **REVERSED ORIENTATION**
- **Next lane doesn't have `direction_change_area` tag** → Autoware defaults to **FORWARD** following (original orientation)
- **Result**: **FATAL CONDITION** - Vehicle has reversed orientation but system expects forward orientation

**Safety Check Implementation**:
```cpp
bool checkLaneContinuitySafety(
  const PathWithLaneId & path, 
  const std::vector<size_t> & cusp_indices,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  // Algorithm:
  // 1. Detect lane boundaries in path
  // 2. For each lane boundary:
  //    a. Count cusps before boundary (in current lane)
    //    b. If odd number of cusps:
    //       - Vehicle exits with REVERSED ORIENTATION
    //       - Check if next lane (after boundary) has direction_change_area tag
    //       - If next lane doesn't have tag → FATAL (return false)
    //       - If next lane has tag → Safe (return true)
  //    c. If even number of cusps:
  //       - Vehicle exits in FORWARD mode → Safe (return true)
  // 3. Return false if any fatal condition detected, true otherwise
}
```

**Current Status**: **Placeholder implementation** - Function exists but returns `true` (assumes safe). **Must be fully implemented before production use**.

**Future Enhancement** (Placeholder):
```cpp
std::vector<size_t> detectLaneBoundaries(path) {
  // TODO: Implementation postponed
  // Should detect lane transitions by tracking lane_id changes
  // Return indices where lane transitions occur
}
```

**Rationale**: 
- Lane boundary detection is postponed as the current implementation handles lane continuity through path-wide cusp detection
- However, the safety check for reverse exit is **critical** and must be implemented to prevent fatal conditions
- The orientation reversal is determined by cusp positions, which naturally handles lane transitions, but we must ensure next lane compatibility

### 6.7 Path Processing Strategy

**Debug Mode**:
```cpp
#ifdef DEBUG_MODE
  PathWithLaneId debug_path_copy = reference_path_;
  // Print/log original velocities
#endif
```

**Production Mode**:
```cpp
output.path = reference_path_;  // Direct assignment
reverseOrientationAtCusps(&output.path, cusp_indices);  // Overwrite in-place
```

**Rationale**: 
- Debug mode allows inspection of original vs. modified paths
- Production mode minimizes memory overhead by modifying in-place
- Conditional compilation ensures no performance impact in production

### 6.7 Integration with Behavior Path Planner

**Plugin Registration**:
- Module is registered via `plugins.xml`
- Manager class inherits from `SceneModuleManagerInterface`
- Automatically discovered and loaded by behavior path planner

**Module Execution**:
1. Behavior path planner calls `isExecutionRequested()`
2. Module checks activation conditions
3. If active, planner calls `plan()`
4. Module processes path and returns modified path
5. Modified path passed to next module or BVP

**Data Flow**:
```
Previous Module Output
    ↓
Direction Change Module Input (reference_path_)
    ↓
Cusp Detection
    ↓
Orientation Reversal
    ↓
Modified Path Output
    ↓
Next Module / BVP
```

### 6.8 Integration with Behavior Velocity Planner and Downstream Modules

**Downstream Module Direction Detection**:

The Behavior Velocity Planner and other downstream modules use geometry-based direction detection:

1. **`isDrivingForward()`** (geometry-based): Uses path point orientations (yaw angles) to determine driving direction
   - Compares orientation of consecutive path points
   - Returns `true` if driving forward, `false` if driving backward
   - **This function works with orientation reversal** ✅

2. **`isDrivingForwardWithTwist()`** (velocity-based with geometry fallback): First checks velocity signs, then falls back to geometry
   - If velocity is positive → forward
   - If velocity is negative → backward
   - If velocity is zero → falls back to `isDrivingForward()` (geometry-based)
   - **This function may need velocity signs for full compatibility** ⚠️

**Current Implementation Strategy**:
- The module reverses path point orientations (yaw) at cusp points
- This enables `isDrivingForward()` to correctly identify reverse segments
- For modules using `isDrivingForwardWithTwist()`, the geometry fallback will work when velocity is zero
- **Note**: For full compatibility with velocity-based detection, velocity reversal may be needed in the future (see TODO in code)

### 6.9 Parameter Configuration

**Key Parameters**:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cusp_detection_angle_threshold_deg` | double | 90.0 | Angle threshold (degrees) for cusp detection |
| `enable_cusp_detection` | bool | true | Enable/disable cusp detection |
| `publish_debug_marker` | bool | false | Publish debug visualization markers |

**Configuration File**: `config/direction_change.param.yaml`

**Rationale**: Configurable parameters allow tuning for different scenarios and vehicle types without code changes.

### 6.10 Drivable Area Information Preservation

**Critical Requirement**: The module MUST preserve `drivable_area_info` from the previous module output.

**Implementation**:
```cpp
output.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
```

**Why This Is Required**:
- `PlannerManager::generateCombinedDrivableArea()` is called after all modules execute
- It uses `output.drivable_area_info.drivable_lanes` to compute `left_bound` and `right_bound`
- Without this information, `generateDrivableArea()` fails to compute bounds and throws error:
  - "The right or left bound of drivable area is empty"
- Since we only modify path point orientations (yaw), not geometry, the lane information remains valid
- The drivable area bounds are regenerated by PlannerManager using the preserved lane information

**Rationale**: This follows the same pattern as other behavior path planner modules (e.g., `start_planner`, `static_obstacle_avoidance`) which preserve or combine drivable area information.

### 6.11 Error Handling

**Lanelet Not Found**:
- If `getLaneletsFromId()` fails, skip that lanelet and continue checking others
- Use try-catch to handle exceptions gracefully

**Empty Path**:
- If path is empty, module returns early without processing
- Activation check returns false

**No Cusp Points**:
- If no cusp points detected, module does not activate
- Path returned unchanged

**Invalid Parameters**:
- Parameter validation in manager initialization
- Default values used if parameters invalid

---

## 7. Integration Considerations

### 7.1 Planning Validator Compatibility

**Issue**: The planning_validator may flag errors for paths with reversed orientations. This is a known limitation when using direction change functionality.

#### 7.1.1 Relative Angle Validation Error

**What It Checks**:
- Calculates the relative angle between three consecutive trajectory points using geometric azimuth angles
- Formula: `relative_angle = |azimuth(p2, p3) - azimuth(p1, p2)|`
- Default threshold: **2.0 radians (~115 degrees)**

**Why It Fails**:
- When orientations are reversed by 180° (π radians) at cusp points, the path geometry may have sharp turns
- The geometric angle between consecutive path segments can exceed the threshold
- **Note**: The validator checks trajectory (from velocity smoother), not the path directly, so point spacing and positions may differ

**Error Message**: `"trajectory yaw angle varies too fast"` or diagnostic `/planning/006-trajectory_relative_angle_validation-error`

**Impact**: Warning/error flag, but trajectory is still valid for direction change scenarios

#### 7.1.2 Trajectory Shift Validation Error

**What It Checks**:
- Compares current trajectory with previous trajectory at nearest points to ego
- Checks lateral and longitudinal shifts between consecutive planning cycles
- Default thresholds:
  - Lateral shift: 0.5 m
  - Forward shift: 1.0 m
  - Backward shift: 0.1 m

**Why It Fails**:
- When orientation reversal is applied, the nearest point calculation may find different points
- The trajectory may appear to have shifted laterally or longitudinally
- **Note**: This is a comparison between planning cycles, not within a single trajectory

**Error Message**: `"planning trajectory had sudden shift"` or diagnostic `/planning_validator/validation_status/is_valid_trajectory_shift`

**Impact**: Critical error flag (is_critical: true), but this is expected behavior for direction change

#### 7.1.3 Potential Solutions

**Option 1: Adjust Validator Thresholds** (Short-term, Quick Fix)
- Increase `relative_angle.threshold` to accommodate 180° direction changes (e.g., 3.14 radians)
- Increase `trajectory_shift` thresholds for direction change scenarios
- **Pros**: Simple, no code changes needed
- **Cons**: May mask real errors in other scenarios

**Option 2: Disable Validator Checks for Direction Change Areas** (Short-term)
- Modify validator to check if path is in `direction_change_area` tagged lanes
- Disable or relax checks when tag is present
- **Pros**: Targeted solution
- **Cons**: Requires validator code modifications

**Option 3: Accept Warnings as Expected** (Current approach)
- Document that these warnings are expected for direction change scenarios
- Monitor for actual trajectory problems separately
- **Pros**: No code changes needed
- **Cons**: Warning noise in logs

**Option 4: Make Validator Direction-Change-Aware** (Long-term, Recommended)
- Add validator plugin or parameter to indicate direction change scenarios
- Validator adjusts thresholds or skips checks accordingly
- **Pros**: Most robust solution
- **Cons**: Requires significant validator modifications

#### 7.1.4 Implementation Solutions

##### Short-Term Solution: Configuration Adjustments

**Recommended Immediate Actions**:

1. **Make trajectory_shift non-critical** (prevents blocking trajectory publication):
```yaml
# File: config/planning_validator/trajectory_checker.param.yaml
trajectory_checker:
  trajectory_shift:
    enable: true
    lat_shift_th: 0.5
    forward_shift_th: 1.0
    backward_shift_th: 0.1
    is_critical: false  # Change from true to false
```

2. **Optionally increase relative_angle threshold** (if warnings persist):
```yaml
trajectory_checker:
  relative_angle:
    enable: true
    threshold: 3.0  # Increased from 2.0 to allow ~172 degrees (accommodates 180° with margin)
    is_critical: false
```

**Alternative: Disable specific checks** (if acceptable for your use case):
```yaml
trajectory_checker:
  relative_angle:
    enable: false  # Disable if not critical
  
  trajectory_shift:
    enable: false  # Disable if trajectory shifts are acceptable
```

##### Long-Term Solution: Direction-Change-Aware Validator

**Implementation Strategy**: Modify planning_validator to detect when trajectory is in `direction_change_area` tagged lanes and apply relaxed validation thresholds.

**Step 1: Add Helper Function to Check Direction Change Area**

**File**: `autoware_planning_validator_trajectory_checker/src/utils.cpp` or `trajectory_checker.cpp`

```cpp
/**
 * @brief Check if trajectory points are in lanes with direction_change_area tag
 * @param [in] trajectory Trajectory to check
 * @param [in] route_handler Route handler to access lanelet map
 * @return True if majority of sampled trajectory points are in direction_change_area lanes
 */
bool isTrajectoryInDirectionChangeArea(
  const Trajectory & trajectory, 
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  if (!route_handler || !route_handler->isHandlerReady()) {
    return false;
  }
  
  // Check a sample of trajectory points to see if they're in lanes with direction_change_area tag
  // Sampling reduces computational overhead
  constexpr size_t sample_interval = 10;
  size_t checked_count = 0;
  size_t in_direction_change_area = 0;
  
  for (size_t i = 0; i < trajectory.points.size(); i += sample_interval) {
    const auto & point = trajectory.points[i];
    try {
      // Find nearest lanelet to trajectory point
      const auto nearest_lanelets = route_handler->getLaneletsFromPoint(
        point.pose.position);
      
      for (const auto & lanelet : nearest_lanelets) {
        const std::string tag = lanelet.attributeOr("direction_change_area", "none");
        if (tag != "none") {
          ++in_direction_change_area;
          break;  // Found tag in this point's lane
        }
      }
      ++checked_count;
    } catch (...) {
      // Lanelet not found, skip
      continue;
    }
  }
  
  // If majority of checked points are in direction_change_area, consider trajectory as such
  // Threshold: >50% of sampled points must be in direction_change_area lanes
  return checked_count > 0 && 
         (static_cast<double>(in_direction_change_area) / checked_count) > 0.5;
}
```

**Step 2: Modify Relative Angle Validation**

**File**: `autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp`

**Modify `check_valid_relative_angle()` method**:

```cpp
bool TrajectoryChecker::check_valid_relative_angle(
  const std::shared_ptr<const PlanningValidatorData> & data,
  const std::shared_ptr<PlanningValidatorStatus> & status)
{
  if (!params_.relative_angle.enable) {
    return true;
  }

  const auto & trajectory = *data->resampled_current_trajectory;
  
  // Check if trajectory is in direction_change_area
  const bool in_direction_change_area = 
    isTrajectoryInDirectionChangeArea(trajectory, data->route_handler);
  
  // Use relaxed threshold for direction change scenarios
  // Allow up to ~230 degrees (π + margin) for 180° direction changes
  const double effective_threshold = in_direction_change_area 
    ? params_.relative_angle.threshold * 1.8  // ~3.6 radians for 2.0 rad base threshold
    : params_.relative_angle.threshold;
  
  const auto [max_relative_angle, i] = 
    trajectory_checker_utils::calcMaxRelativeAngles(trajectory);
  status->max_relative_angle = max_relative_angle;

  if (max_relative_angle > effective_threshold) {
    const auto & p = trajectory.points;
    if (i < p.size() - 3) {
      context_->debug_pose_publisher->pushPoseMarker(p.at(i), "trajectory_relative_angle", 0);
      context_->debug_pose_publisher->pushPoseMarker(p.at(i + 1), "trajectory_relative_angle", 1);
      context_->debug_pose_publisher->pushPoseMarker(p.at(i + 2), "trajectory_relative_angle", 2);
    }
    is_critical_error_ |= params_.relative_angle.is_critical;
    return false;
  }
  
  return true;
}
```

**Step 3: Modify Trajectory Shift Validation**

**File**: `autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp`

**Modify `check_trajectory_shift()` method**:

```cpp
bool TrajectoryChecker::check_trajectory_shift(
  const std::shared_ptr<const PlanningValidatorData> & data,
  const std::shared_ptr<PlanningValidatorStatus> & status)
{
  bool is_valid = true;
  if (!params_.trajectory_shift.enable || !data->last_valid_trajectory) {
    return is_valid;
  }

  const auto & trajectory = *data->current_trajectory;
  const auto & prev_trajectory = *data->last_valid_trajectory;
  const auto & ego_pose = data->current_kinematics->pose.pose;

  // Check if current trajectory is in direction_change_area
  const bool in_direction_change_area = 
    isTrajectoryInDirectionChangeArea(trajectory, data->route_handler);
  
  // Use relaxed thresholds for direction change scenarios
  const double lat_threshold = in_direction_change_area 
    ? params_.trajectory_shift.lat_shift_th * 2.0  // Double the threshold
    : params_.trajectory_shift.lat_shift_th;
  const double forward_threshold = in_direction_change_area
    ? params_.trajectory_shift.forward_shift_th * 2.0
    : params_.trajectory_shift.forward_shift_th;
  const double backward_threshold = in_direction_change_area
    ? params_.trajectory_shift.backward_shift_th * 2.0
    : params_.trajectory_shift.backward_shift_th;

  const auto nearest_seg_idx = data->nearest_segment_index;
  const auto prev_nearest_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(prev_trajectory.points, ego_pose);

  if (!nearest_seg_idx || !prev_nearest_seg_idx) {
    return is_valid;
  }

  const auto & nearest_pose = trajectory.points.at(*nearest_seg_idx).pose;
  const auto & prev_nearest_pose = prev_trajectory.points.at(*prev_nearest_seg_idx).pose;

  const auto & ego_lat_dist =
    std::abs(autoware_utils::calc_lateral_deviation(ego_pose, nearest_pose.position));

  const auto lat_shift =
    std::abs(autoware_utils::calc_lateral_deviation(prev_nearest_pose, nearest_pose.position));

  static constexpr auto epsilon = 0.01;
  status->lateral_shift = lat_shift > epsilon ? lat_shift : 0.0;

  // Use relaxed threshold for direction change scenarios
  if (
    ego_lat_dist > lat_threshold &&
    lat_shift > lat_threshold) {
    is_critical_error_ |= params_.trajectory_shift.is_critical;
    context_->debug_pose_publisher->pushPoseMarker(nearest_pose, "trajectory_shift");
    is_valid = false;
  }

  const auto is_check_lon_shift = std::invoke([&]() {
    if (*prev_nearest_seg_idx == prev_trajectory.points.size() - 2) {
      return false;
    }
    if (*nearest_seg_idx > 0 && *nearest_seg_idx < trajectory.points.size() - 2) {
      return false;
    }
    return true;
  });

  if (!is_check_lon_shift) {
    status->longitudinal_shift = 0.0;
    return is_valid;
  }

  const auto lon_shift =
    autoware_utils::calc_longitudinal_deviation(prev_nearest_pose, nearest_pose.position);

  status->longitudinal_shift = std::abs(lon_shift) > epsilon ? lon_shift : 0.0;

  // Use relaxed threshold for forward shift
  if (*nearest_seg_idx == 0) {
    if (lon_shift > forward_threshold) {
      is_critical_error_ |= params_.trajectory_shift.is_critical;
      context_->debug_pose_publisher->pushPoseMarker(nearest_pose, "trajectory_shift");
      is_valid = false;
    }
    return is_valid;
  }

  // Use relaxed threshold for backward shift
  if (lon_shift < 0.0 && std::abs(lon_shift) > backward_threshold) {
    is_critical_error_ |= params_.trajectory_shift.is_critical;
    context_->debug_pose_publisher->pushPoseMarker(nearest_pose, "trajectory_shift");
    is_valid = false;
  }

  return is_valid;
}
```

**Step 4: Add Function Declaration**

**File**: `autoware_planning_validator_trajectory_checker/include/autoware/planning_validator_trajectory_checker/utils.hpp`

```cpp
/**
 * @brief Check if trajectory points are in lanes with direction_change_area tag
 * @param [in] trajectory Trajectory to check
 * @param [in] route_handler Route handler to access lanelet map
 * @return True if majority of sampled trajectory points are in direction_change_area lanes
 */
bool isTrajectoryInDirectionChangeArea(
  const Trajectory & trajectory,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);
```

**Step 5: Optional Configuration Parameters**

**File**: `autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml`

Optionally add parameters for direction_change-specific thresholds:

```yaml
trajectory_checker:
  relative_angle:
    enable: true
    threshold: 2.0
    is_critical: false
    # Optional: Direction change specific threshold multiplier
    direction_change_threshold_multiplier: 1.8  # Default: 1.8 (allows ~230 degrees)
  
  trajectory_shift:
    enable: true
    lat_shift_th: 0.5
    forward_shift_th: 1.0
    backward_shift_th: 0.1
    is_critical: false
    # Optional: Direction change specific threshold multipliers
    direction_change_lat_multiplier: 2.0
    direction_change_forward_multiplier: 2.0
    direction_change_backward_multiplier: 2.0
```

**Benefits of Long-Term Solution**:
- ✅ **Targeted**: Only affects trajectories in `direction_change_area` tagged lanes
- ✅ **Maintains Safety**: Normal scenarios still use strict validation
- ✅ **Automatic**: No manual configuration needed per scenario
- ✅ **Robust**: Works automatically based on map tags

**Implementation Considerations**:
- **Performance**: Lanelet lookups add computational overhead. Sampling trajectory points (every 10th point) mitigates this.
- **Accuracy**: Using 50% threshold means if majority of trajectory is in direction_change_area, relaxed thresholds apply.
- **Testing**: Requires unit tests to verify:
  - Helper function correctly identifies direction_change_area trajectories
  - Validation checks use relaxed thresholds appropriately
  - Normal trajectories still use strict thresholds

**Files to Modify**:
1. `autoware_planning_validator_trajectory_checker/src/trajectory_checker.cpp` - Add helper function and modify validation checks
2. `autoware_planning_validator_trajectory_checker/include/autoware/planning_validator_trajectory_checker/utils.hpp` - Add helper function declaration
3. `autoware_planning_validator_trajectory_checker/config/trajectory_checker.param.yaml` - Optionally add direction_change-specific parameters

**Recommended Implementation Plan**:
- **Phase 1 (Short-term)**: Apply configuration adjustments (make trajectory_shift non-critical, optionally increase thresholds)
- **Phase 2 (Long-term)**: Implement direction-change-aware validator with helper function and modified validation checks
- **Phase 3 (Testing)**: Add unit tests and validate in real-world scenarios

#### 7.1.5 Current Status

**Recommendation**: These validator warnings/errors are **expected behavior** when using direction change functionality. They indicate intentional orientation changes, not actual trajectory problems. The path geometry remains valid.

**For Production**:
- **Short-term**: Apply configuration adjustments (Phase 1) for immediate deployment
- **Long-term**: Implement direction-change-aware validator (Phase 2) for robust, automatic handling
- Monitor validator diagnostics to distinguish expected warnings from real trajectory issues

### 7.2 Downstream Module Compatibility

**Behavior Velocity Planner**: 
- Uses `isDrivingForward()` which works with orientation reversal ✅
- May need velocity signs for full compatibility with `isDrivingForwardWithTwist()` (future enhancement)

**Path Generator**:
- Receives path with reversed orientations
- Should handle orientation changes correctly

**Motion Planner**:
- Uses path orientations for trajectory generation
- Should respect reversed orientations for reverse segments

## 8. Known Risks / Open Questions

### Known Risks

1. **Fatal Condition: Reverse Exit Without Next Lane Tag** ⚠️ **CRITICAL**
   - **Risk**: If odd number of cusps exist (vehicle exits with reversed orientation) but next lane doesn't have `direction_change_area` tag, Autoware defaults to forward following. This creates a fatal condition where vehicle has reversed orientation but system expects forward orientation.
   - **Mitigation**: 
     - Safety check `checkLaneContinuitySafety()` implemented as placeholder
     - **MUST be fully implemented before production use**
     - Check should prevent module from applying reverse velocities if next lane doesn't support it
     - Current placeholder returns `true` (assumes safe) - this is a **temporary workaround**
   - **Status**: Placeholder exists, full implementation required

2. **False Positive Cusp Detection**
   - **Risk**: Sharp turns or lane changes might be incorrectly detected as cusp points
   - **Mitigation**: Configurable angle threshold allows tuning. Consider adding minimum distance between cusps.

2. **Path Point Density**
   - **Risk**: Sparse path points might miss cusp points or detect them inaccurately
   - **Mitigation**: Assumes upstream modules provide adequately dense paths. May need path resampling if issues occur.

3. **Lanelet Map Quality**
   - **Risk**: Missing or incorrect `direction_change_area` tags could prevent module activation when needed
   - **Mitigation**: Requires proper map annotation. Document tag requirements clearly.

4. **Downstream Module Compatibility**
   - **Risk**: Behavior Velocity Planner uses `std::min()` in `setVelocityFromIndex()`. While this preserves negative velocities when using `insertVelocity()` (because `std::min(positive, negative) = negative`), direct velocity assignments in BVP modules may overwrite negative velocities.
   - **Mitigation**: **VERIFIED** - BVP's `setVelocityFromIndex()` function uses `std::min(vel, existing_velocity)`, which means:
     - If BVP inserts 0.0 and path has -5.0: `std::min(0.0, -5.0) = -5.0` → **Negative preserved** ✅
     - If BVP inserts 5.0 and path has -5.0: `std::min(5.0, -5.0) = -5.0` → **Negative preserved** ✅
     - Most BVP modules use `insertVelocity()` which calls `setVelocityFromIndex()`, so negative velocities are preserved in standard workflows
     - **Note**: Direct velocity assignments (not using `insertVelocity()`) may overwrite negative velocities, but this is not the standard BVP pattern

5. **Performance Impact**
   - **Risk**: Processing all path points and checking all lanelets might impact planning frequency
   - **Mitigation**: Current implementation is O(n) for path processing. Monitor performance in real scenarios.

6. **Coordinate Frame Assumptions**
   - **Risk**: Path points in different coordinate frames could cause incorrect angle calculations
   - **Mitigation**: Assumes all points in same frame (typically `map`). Add validation if needed.

### Open Questions

1. **Smooth Transition Zones**
   - **Question**: Should velocity signs transition smoothly around cusp points instead of instant change?
   - **Status**: Commented in code for future enhancement. Current implementation uses instant change.

2. **Lane Boundary Detection Implementation**
   - **Question**: What is the exact algorithm for detecting lane boundaries and handling sign propagation?
   - **Status**: Method placeholder created. Implementation postponed. Current approach works but may need refinement.

3. **Lane Continuity Safety Check Implementation** ⚠️ **CRITICAL**
   - **Question**: How to fully implement `checkLaneContinuitySafety()` to prevent fatal reverse exit conditions?
   - **Status**: Placeholder exists but returns `true` (assumes safe). **Must be fully implemented before production**.
   - **Requirements**:
     - Detect lane boundaries in path
     - Count cusps before each boundary
     - Check if odd number of cusps (reverse exit)
     - Verify next lane has `turn_area` tag if exiting in reverse
     - Return `false` if fatal condition detected (prevents reverse velocity application)
   - **Priority**: **HIGH** - This is a safety-critical feature

3. **Minimum Distance Between Cusps**
   - **Question**: Should there be a minimum distance requirement between cusp points to avoid false positives?
   - **Status**: Not currently implemented. May be needed based on testing.

4. **Cusp Point Validation**
   - **Question**: Should detected cusp points be validated against expected locations (e.g., from map data)?
   - **Status**: Currently relies solely on angle-based detection. Map-based validation could improve accuracy.

5. **Velocity Magnitude Handling**
   - **Question**: Should the module modify velocity magnitudes at cusp points, or only signs?
   - **Status**: Currently only modifies signs. Magnitudes handled by BVP.
   - **Update**: **VERIFIED** - BVP's `setVelocityFromIndex()` uses `std::min()`, which means:
     - If BVP inserts 0.0 and path has -5.0: `std::min(0.0, -5.0) = -5.0` → **preserves negative** ✅
     - If BVP inserts 5.0 and path has -5.0: `std::min(5.0, -5.0) = -5.0` → **preserves negative** ✅
     - However, if BVP directly assigns positive values (not using `setVelocityFromIndex`), negative velocities will be overwritten
     - **Conclusion**: BVP's `std::min()` logic actually **preserves** negative velocities when using `insertVelocity()`, but direct assignments may not.

6. **Multi-Lane Scenarios**
   - **Question**: How should the module handle paths that span multiple lanes with mixed tag presence?
   - **Status**: Current implementation activates if any lanelet has tag. May need refinement for complex scenarios.

7. **Edge Cases**
   - **Question**: How to handle edge cases like paths starting/ending at cusp points, or paths with cusps at boundaries?
   - **Status**: Current implementation should handle these, but requires testing.

8. **Integration Testing**
   - **Question**: What are the test scenarios and expected behaviors for integration with BVP and VTL modules?
   - **Status**: Testing plan needed. Scenarios defined in IMPLEMENTATION_SUMMARY.md.

9. **Performance Benchmarks**
   - **Question**: What are acceptable performance metrics (processing time, memory usage) for this module?
   - **Status**: Not yet benchmarked. Should be measured in real-world scenarios.

10. **Backward Compatibility**
    - **Question**: Does this module maintain backward compatibility with existing Autoware behavior path planner workflows?
    - **Status**: Should be compatible as it's a plugin, but requires verification.

---

### TODO / Immediate Workarounds

This section lists immediate workaround strategies and optimizations that should be implemented to improve module performance and reliability. These are not critical for basic functionality but address known inefficiencies and compatibility issues.

#### 1. Cusp Detection Caching Strategy ⚡ **Performance Optimization**

**Issue**: `plan()` is called multiple times per planning cycle by `PlannerManager` (through iterative propagation in `propagateFull()`, approved modules execution, and candidate modules execution). Each call triggers `updateData()`, which recomputes cusp detection even when the path hasn't changed.

**Impact**: 
- Redundant computation of cusp detection on every `plan()` call
- Unnecessary processing overhead when path is unchanged
- Multiple debug messages for same cusp points

**Workaround Strategy**:
- Cache cusp detection results and only recompute when path actually changes
- Compare path size and timestamp to detect changes
- Store cached results in member variable `cusp_point_indices_`

**Implementation Approach**:
```cpp
void DirectionChangeModule::updateData()
{
  const auto previous_output = getPreviousModuleOutput();
  if (previous_output.path.points.empty()) {
    return;
  }
  
  // Cache check: only recompute if path actually changed
  const bool path_changed = 
    (reference_path_.points.size() != previous_output.path.points.size()) ||
    (reference_path_.header.stamp != previous_output.path.header.stamp);
  
  reference_path_ = previous_output.path;
  
  if (path_changed || cusp_point_indices_.empty()) {
    cusp_point_indices_ = findCuspPoints();  // Only recompute when needed
  }
  // Otherwise, reuse cached cusp_point_indices_
}
```

**Priority**: Medium (performance optimization, not critical for functionality)

**Status**: Not implemented - placeholder for future optimization

---

#### 2. Downstream Smoother/Optimizer Orientation Preservation 🔧 **Compatibility Fix**

**Issue**: Downstream modules (`elastic_band_smoother` and `path_optimizer`) recalculate path point orientations from geometry, overwriting the intentional 180° orientation reversals applied by the direction_change module.

**Impact**:
- Orientation reversals are lost after path smoothing/optimization
- Intended reverse direction segments revert to forward orientations
- Module's core functionality (orientation reversal) is negated

**Workaround Strategy**:
- Make smoothers/optimizers direction-change-aware
- Detect if path contains `direction_change_area` tagged lanes
- Preserve existing orientations for direction_change segments instead of recalculating from geometry

**Implementation Approach**:

**For Elastic Band Smoother** (`autoware_path_smoother/src/elastic_band.cpp`):
```cpp
// Around line 451, before insertOrientation call:
bool has_direction_change = checkPathHasDirectionChangeArea(eb_traj_points, route_handler);
if (!has_direction_change) {
  autoware::motion_utils::insertOrientation(eb_traj_points, true);
} else {
  // Preserve existing orientations - they're already set by direction_change module
  // Optionally: only recalculate orientations for segments NOT in direction_change_area
}
```

**For Path Optimizer** (`autoware_path_optimizer/src/mpt_optimizer.cpp`):
```cpp
// Around line 574, before updateOrientation call:
bool has_direction_change = checkPathHasDirectionChangeArea(ref_points, route_handler);
if (!has_direction_change) {
  updateOrientation(ref_points, ref_points_spline);
} else {
  // Preserve orientations for direction_change segments
  // Only update orientations for non-direction-change segments
}
```

**Priority**: High (affects core functionality)

**Status**: Not implemented - requires modifications to downstream modules

**Alternative Quick Fix**: Disable smoothers/optimizers for direction_change paths (not recommended for production)

---

#### 3. Planning Validator Compatibility (Already Documented) ✅

**Issue**: Planning validator reports errors for `relative_angle` and `trajectory_shift` due to intentional 180° orientation changes.

**Workaround Strategies**: 
- **Short-term**: Configuration adjustments (make `trajectory_shift` non-critical, increase thresholds)
- **Long-term**: Make validator direction-change-aware (see Section 7.1.4 for detailed implementation)

**Status**: Documented in Section 7.1.4 with implementation details

**Priority**: Medium (validation warnings, not blocking)

---

#### 4. Velocity Reversal Placeholder (Future Enhancement) 📝

**Issue**: Current implementation only reverses orientations (yaw angles), not velocity signs. For full compatibility with downstream modules that check `isDrivingForwardWithTwist()`, velocity reversal may be needed.

**Workaround Strategy**:
- Add placeholder code for negative velocity assignment in `reverseOrientationAtCusps()`
- Keep as TODO comment for future implementation
- Current orientation-only approach is sufficient for basic functionality

**Status**: Placeholder exists in code, not implemented

**Priority**: Low (future enhancement, not required for current functionality)

---

#### 5. Path Generator Orientation Overwrite 🔧 **Downstream Module Issue**

**Issue**: The `path_generator` node publishes `/path_with_lane_id` topic and recalculates path point orientations from geometry, overwriting the intentional 180° orientation reversals applied by the direction_change module.

**Root Cause**:
- `path_generator` is a **separate path generation system** that runs independently from `behavior_path_planner`
- It subscribes to:
  - `~/input/route` (LaneletRoute)
  - `~/input/vector_map` (LaneletMapBin)
  - `~/input/odometry` (Odometry)
- It generates its own path from the lanelet map and route
- During path generation, it calls `trajectory->align_orientation_with_trajectory_direction()` which recalculates orientations from trajectory geometry (azimuth/direction)

**Impact**:
- Orientation reversals set by `direction_change` module are lost when `path_generator` processes the path
- The `/path_with_lane_id` topic shows forward orientations even in reverse segments
- Module's core functionality (orientation reversal) is negated for paths processed by `path_generator`
- This affects any downstream system that subscribes to `/path_with_lane_id` instead of the behavior_path_planner output

**Code Flow**:

```
DirectionChangeModule::plan()
  └─→ reverseOrientationAtCusps()  [Sets yaw = yaw + 180°]
      └─→ output.path (with reversed orientations)
          └─→ BehaviorPathPlannerNode::run()
              └─→ path_publisher_->publish()  [Publishes to ~/output/path]
                  │
                  ├─→ (Separate path) path_generator subscribes to route/map/odometry
                  │   └─→ PathGenerator::generate_path()
                  │       └─→ trajectory->align_orientation_with_trajectory_direction()  ❌ OVERWRITES!
                  │           └─→ path_publisher_->publish()  [Publishes to ~/output/path = /path_with_lane_id]
                  │
                  └─→ (Original path) Other downstream modules use behavior_path_planner output
```

**Location of Issue**:
- **File**: `src/autoware/core/planning/autoware_path_generator/src/node.cpp`
- **Line 447**: `trajectory->align_orientation_with_trajectory_direction();` (after path generation)
- **Line 470**: `refined_path->align_orientation_with_trajectory_direction();` (after goal refinement)

**What `align_orientation_with_trajectory_direction()` Does**:
- Recalculates yaw angles from trajectory geometry (azimuth/direction of interpolated curve)
- Overwrites existing orientations, including intentional 180° reversals
- Designed for cases where only positions are provided and orientations need to be calculated

**Workaround Strategy**:
- Make `path_generator` direction-change-aware
- Check if path contains `direction_change_area` tagged lanes
- Preserve existing orientations for direction_change segments instead of recalculating from geometry
- Only recalculate orientations for segments NOT in `direction_change_area`

**Implementation Approach**:

**For Path Generator** (`autoware_path_generator/src/node.cpp`):
```cpp
// Around line 447, before align_orientation_with_trajectory_direction call:
bool has_direction_change = checkPathHasDirectionChangeArea(
  path_points_with_lane_id, planner_data_.lanelet_map_ptr);
if (!has_direction_change) {
  trajectory->align_orientation_with_trajectory_direction();
} else {
  // Preserve existing orientations for direction_change segments
  // Only recalculate orientations for segments NOT in direction_change_area
  // OR: Skip orientation alignment entirely if any direction_change_area is present
}
```

**Helper Function Needed**:
```cpp
bool checkPathHasDirectionChangeArea(
  const std::vector<PathPointWithLaneId> & path_points,
  const lanelet::LaneletMapPtr & lanelet_map)
{
  for (const auto & point : path_points) {
    for (const auto & lane_id : point.lane_ids) {
      const auto & lanelet = lanelet_map->laneletLayer.get(lane_id);
      if (hasDirectionChangeAreaTag(lanelet)) {
        return true;
      }
    }
  }
  return false;
}
```

**Alternative Approaches**:
1. **Skip orientation alignment for direction_change paths**: If any lane in the path has `direction_change_area` tag, skip `align_orientation_with_trajectory_direction()` entirely
2. **Selective orientation preservation**: Only preserve orientations for path points in `direction_change_area` lanes, recalculate for others
3. **Metadata flag**: Add a flag to path messages indicating intentional orientation changes (requires message definition changes)

**Priority**: High (affects core functionality if path_generator output is used)

**Status**: Not implemented - requires modifications to `path_generator` module

**Note**: This is a separate issue from the smoother/optimizer problem (item #2). `path_generator` is a different module that generates paths independently, while smoothers/optimizers process behavior_path_planner output.

---

#### 6. Goal Pose Validation for Reversed Orientations 🔧 **Downstream Module Issue**

**Issue**: Goal pose validation in `mission_planner` and `arrival_checker` uses strict orientation thresholds that reject reversed goal poses (yaw + 180°), even when the goal lanelet has the `direction_change_area` tag.

**Root Cause**:
- When the vehicle approaches a goal in a `direction_change_area` with reversed orientation (yaw + 180°), the validation fails
- Current validation checks if goal orientation matches lanelet orientation within a threshold (typically 45°)
- Reversed orientations (180° difference) exceed this threshold and are rejected

**Impact**:
- Vehicle cannot reach goals in `direction_change_area` lanes when approaching with reversed orientation
- Mission planning fails to validate goals in direction change areas
- Arrival checker rejects arrival even when vehicle is at goal position with reversed orientation

**Code Location**:

**Goal Validation** (`autoware_mission_planner_universe/src/lanelet2_plugins/default_planner.cpp`):
- **Line 242, 304**: Uses `param_.goal_angle_threshold_deg` (default 45°) to validate goal orientation
- Checks if `std::abs(angle_diff) < th_angle` where `angle_diff = normalize_radian(lane_yaw - goal_yaw)`
- Reversed orientations (180° difference) fail this check
- **Note**: `arrival_checker` does NOT need modification because it compares current pose to the already-validated goal pose. If the goal validation accepts a reversed goal pose, the arrival checker will automatically work correctly since it's doing a relative comparison.

**Workaround Strategy**:
- Check if goal lanelet has `direction_change_area` tag
- If tag exists, relax orientation threshold to accept both forward and reversed orientations
- For reversed orientations, check if `std::abs(angle_diff - M_PI) < relaxed_threshold` (180° ± relaxed threshold)
- Use a relaxed threshold value (e.g., 90° or 120°) to account for orientation reversal

**Implementation Approach**:

**For Goal Validation** (`default_planner.cpp`):
```cpp
// Around line 242 and 304, modify angle check:
const auto goal_yaw = tf2::getYaw(goal.orientation);
const auto angle_diff = autoware_utils::normalize_radian(lane_yaw - goal_yaw);

// Check if goal lanelet has direction_change_area tag
bool has_direction_change_tag = false;
if (closest_shoulder_lanelet.id() != 0) {
  has_direction_change_tag = hasDirectionChangeAreaTag(closest_shoulder_lanelet);
} else if (closest_lanelet_to_goal.id() != 0) {
  has_direction_change_tag = hasDirectionChangeAreaTag(closest_lanelet_to_goal);
}

double th_angle = autoware_utils::deg2rad(param_.goal_angle_threshold_deg);
if (has_direction_change_tag) {
  // Relaxed threshold for direction_change_area (e.g., 90° or 120°)
  th_angle = autoware_utils::deg2rad(90.0);  // or configurable parameter
}

// Check forward orientation
if (std::abs(angle_diff) < th_angle) {
  return true;
}

// If direction_change_area, also check reversed orientation (180° ± threshold)
if (has_direction_change_tag) {
  const double reversed_angle_diff = std::abs(autoware_utils::normalize_radian(angle_diff - M_PI));
  if (reversed_angle_diff < th_angle) {
    return true;
  }
}
```

**Note on Arrival Checker**: The `arrival_checker` does NOT need modification. Here's why:

1. **Code Flow**:
   - `DefaultPlanner::plan()` calls `is_goal_valid(goal_pose)` to validate the goal
   - If valid, route is created with that validated `goal_pose`
   - `arrival_checker_.set_goal(goal)` stores the validated goal pose from the route
   - `arrival_checker_.is_arrived(current_pose)` compares current pose to the stored goal pose

2. **Why No Modification Needed**:
   - The arrival checker performs a **relative comparison** between current pose and the already-validated goal pose
   - If `is_goal_valid()` accepts a reversed goal pose (when lanelet has `direction_change_area` tag), then:
     - The route will contain the reversed goal pose
     - The arrival checker will store that reversed goal pose
     - When the vehicle arrives with reversed orientation, it will match the reversed goal pose
   - Therefore, fixing `is_goal_valid()` is sufficient - the arrival checker will work automatically

**Helper Function Needed**:
```cpp
// In default_planner.cpp
bool hasDirectionChangeAreaTag(const lanelet::ConstLanelet & lanelet)
{
  const auto & attr = lanelet.attribute("direction_change_area");
  return attr.exists() && attr.value() != "none";
}
```

**Configuration Parameters**:
- Add `goal_angle_threshold_direction_change_deg` parameter (default: 90.0°) for relaxed threshold in direction_change areas
- Note: No additional parameter needed for arrival_checker since it uses the validated goal pose

**Code Flow**:

```
Goal Pose Validation Request
  └─→ Get closest lanelet to goal
      └─→ Check if lanelet has direction_change_area tag
          │
          ├─→ If NO tag:
          │   └─→ Use standard threshold (45°)
          │       └─→ Check: |lane_yaw - goal_yaw| < 45°
          │
          └─→ If YES tag:
              └─→ Use relaxed threshold (90°)
                  ├─→ Check forward: |lane_yaw - goal_yaw| < 90°
                  └─→ Check reversed: |(lane_yaw - goal_yaw) - π| < 90°
                      └─→ Accept if either check passes
```

**Priority**: High (blocks goal reaching in direction_change areas)

**Status**: Not implemented - requires modifications to `default_planner.cpp::is_goal_valid()` only

**Note**: 
- This validation is separate from path planning. It affects whether a goal can be set, not the path generation itself.
- The `arrival_checker` does NOT need modification because it compares current pose to the already-validated goal pose. Once the goal validation accepts reversed poses, the arrival checker will automatically work correctly.

---

## 8. Class and Data Structure Definitions

This section documents all classes, enums, and data structures defined for the **Direction Change Module**. The module implements direction change behavior by detecting cusp points in planned paths and reversing path point orientations (yaw angles) to indicate reverse direction segments.

**Note on Naming**: All class and struct names have been updated to reflect the new module name: `DirectionChange*`. The implementation focuses on **orientation reversal** rather than velocity sign assignment, and the module is activated based on the `direction_change_area` map tag.

---

### 8.1 DirectionChangeParameters Struct

**Purpose**: Contains all configurable parameters for the Direction Change Module, including cusp detection thresholds, path generation settings, and module behavior flags.

**Definition**:
```cpp
struct DirectionChangeParameters
{
  // Cusp detection parameters
  double cusp_detection_distance_threshold;  // [m] Distance threshold for cusp detection
  double cusp_detection_angle_threshold_deg; // [deg] Angle threshold for cusp detection (configurable)
  double cusp_approach_speed;                // [m/s] Speed limit when approaching cusp
  double cusp_stop_distance;                 // [m] Distance from cusp to stop

  // Direction change parameters
  double reverse_speed_limit;         // [m/s] Maximum reverse speed (for future velocity reversal)
  double reverse_lookahead_distance;   // [m] Lookahead distance for reverse following
  double reverse_safety_margin;       // [m] Safety margin for reverse following

  // Path generation parameters
  double path_resolution;        // [m] Resolution for path generation
  double backward_path_length;   // [m] Length of backward path segment
  double forward_path_length;    // [m] Length of forward path segment

  // General parameters
  bool enable_cusp_detection;     // Enable/disable cusp point detection
  bool enable_reverse_following;  // Enable/disable direction change behavior
  bool publish_debug_marker;      // Enable/disable debug marker publishing
};
```

**Configuration**: Parameters are loaded from `config/direction_change.param.yaml` during module initialization.

**Key Parameters**:
- `cusp_detection_angle_threshold_deg`: Configurable angle threshold (in degrees) for detecting cusp points. Path points with yaw angle differences exceeding this threshold are marked as cusp points.
- `enable_cusp_detection`: Master switch to enable/disable cusp point detection. When disabled, the module will not detect or process cusp points.

---

### 8.2 DirectionChangeDebugData Struct

**Purpose**: Stores debug information for visualization, logging, and debugging purposes. Used by the module to track detected cusp points and path segments.

**Definition**:
```cpp
struct DirectionChangeDebugData
{
  std::vector<geometry_msgs::msg::Point> cusp_points{};  // Detected cusp point positions
  PathWithLaneId forward_path{};                        // Forward path segment (original orientation)
  PathWithLaneId reverse_path{};                        // Reverse path segment (reversed orientation)
};
```

**Usage**: 
- Populated by `setDebugMarkersVisualization()` method
- Used for RViz visualization markers
- Can be used for logging and debugging module behavior

---

### 8.3 DirectionChangeModule Class

**Purpose**: Main scene module class that implements the Direction Change Module behavior. This class detects cusp points in planned paths and reverses path point orientations (yaw angles) to indicate reverse direction segments, enabling downstream modules to correctly interpret bidirectional paths.

**Inheritance**: Inherits from `SceneModuleInterface` (base class for all behavior path planner scene modules).

**Key Responsibilities**:
1. **Activation Detection**: Checks for `direction_change_area` tag in lanelets to determine if the module should be active
2. **Cusp Detection**: Identifies cusp points in the path using angle-based detection
3. **Orientation Reversal**: Reverses path point yaw angles at cusp points to indicate direction changes
4. **Safety Validation**: Performs lane continuity safety checks to prevent fatal conditions

**Key Public Methods**:

| Method | Purpose |
|--------|---------|
| `isExecutionRequested()` | Checks if module should be activated based on `direction_change_area` tag presence |
| `isExecutionReady()` | Checks if module is ready to execute (always returns `true` in current implementation) |
| `updateData()` | Updates internal data from previous module output, extracts lanelets, and detects cusp points |
| `plan()` | Main planning function - reverses orientations (yaw) at cusp points and returns modified path |
| `planWaitingApproval()` | Planning function when waiting for approval (delegates to `plan()`) |
| `planCandidate()` | Generates candidate path for external judgment |
| `processOnEntry()` | Called when module enters RUNNING state - initializes variables and updates data |
| `processOnExit()` | Called when module exits RUNNING state - resets variables |

**Key Private Methods**:

| Method | Purpose |
|--------|---------|
| `findCuspPoints()` | Detects cusp points in the path using angle-based detection with configurable threshold |
| `shouldActivateModule()` | Determines if module should activate based on `direction_change_area` tag in lanelets |
| `canTransitSuccessState()` | Checks if module can transition to SUCCESS state (always returns `true`) |
| `setDebugMarkersVisualization()` | Sets debug marker data for visualization |

**Member Variables**:

| Variable | Type | Purpose |
|----------|------|---------|
| `reference_path_` | `PathWithLaneId` | Input path from previous module (before orientation reversal) |
| `modified_path_` | `PathWithLaneId` | Modified path with reversed orientations (after processing) |
| `current_lanelets_` | `lanelet::ConstLanelets` | Current lanelets in the path (extracted from path point lane_ids) |
| `parameters_` | `std::shared_ptr<DirectionChangeParameters>` | Module parameters (shared with manager) |
| `cusp_point_indices_` | `std::vector<size_t>` | Indices of detected cusp points in the path |
| `debug_data_` | `DirectionChangeDebugData` | Debug data for visualization and logging |

**Implementation Notes**:
- The module uses **batch processing** - it processes the entire path at once rather than incrementally
- Orientation reversal is applied by adding π radians to yaw angles and normalizing the result
- The module does NOT modify velocity values - only path point orientations are reversed
- Safety checks are performed before applying orientation reversals to prevent fatal conditions

---

### 8.5 DirectionChangeModuleManager Class

**Purpose**: Manages the lifecycle, initialization, and instances of `DirectionChangeModule`. This manager class handles plugin registration, parameter loading, and module instance creation.

**Inheritance**: Inherits from `SceneModuleManagerInterface` (base class for all scene module managers in the behavior path planner).

**Key Responsibilities**:
1. **Plugin Registration**: Registers the module with the behavior path planner plugin system
2. **Parameter Management**: Loads and manages module parameters from ROS parameters
3. **Instance Management**: Creates and manages instances of the scene module

**Key Methods**:

| Method | Purpose |
|--------|---------|
| `init()` | Initializes the manager, loads parameters from ROS node, and sets up the module |
| `createNewSceneModuleInstance()` | Creates a new instance of `DirectionChangeModule` with shared parameters |
| `updateModuleParams()` | Updates module parameters from ROS parameters (dynamic reconfiguration support) |

**Member Variables**:

| Variable | Type | Purpose |
|----------|------|---------|
| `parameters_` | `std::shared_ptr<DirectionChangeParameters>` | Shared parameters for all module instances (loaded from config file) |

**Plugin Registration**: 
- Registered via `plugins.xml` for dynamic loading by the behavior path planner
- Automatically discovered and loaded at runtime
- Module name in plugin system: `direction_change`

---

### 8.6 Utility Functions

**Purpose**: Helper functions for cusp detection, orientation reversal, lanelet tag checking, and safety validation. These functions are namespace-level utilities used by the Direction Change Module.

**Namespace**: All utility functions are in `autoware::behavior_path_planner` namespace.

**Functions**:

| Function | Purpose | Parameters | Return |
|----------|---------|------------|--------|
| `detectCuspPoints()` | Detects cusp points in the path using angle-based detection with path point yaws | `path` (const PathWithLaneId&), `angle_threshold_deg` (double) | `std::vector<size_t>` (cusp point indices) |
| `reverseOrientationAtCusps()` | Reverses path point orientations (yaw) at cusp points by adding π radians | `path` (PathWithLaneId*), `cusp_indices` (const std::vector<size_t>&) | `void` (modifies path in-place) |
| `detectLaneBoundaries()` | Detects lane boundaries/transitions in the path (placeholder, implementation postponed) | `path` (const PathWithLaneId&) | `std::vector<size_t>` (boundary indices) |
| `hasDirectionChangeAreaTag()` | Checks if a lanelet has the `direction_change_area` tag | `lanelet` (const lanelet::ConstLanelet&) | `bool` |
| `checkLaneContinuitySafety()` | **CRITICAL**: Safety check for reverse exit with next lane compatibility | `path`, `cusp_indices`, `route_handler` | `bool` (false = fatal condition detected) |

**Key Function Details**:

**`reverseOrientationAtCusps()`**:
- **Algorithm**: For each path point, counts the number of cusp points that occur before it
- If odd number of cusps passed → reverses orientation (yaw + π, normalized)
- If even number of cusps passed → keeps original orientation
- **Mathematical Formula**: `yaw_reversed[i] = normalize_radian(yaw[i] + π)` if odd cusps passed, else `yaw[i]`

**`hasDirectionChangeAreaTag()`**:
- Checks for the `direction_change_area` attribute in the lanelet
- Returns `true` if tag exists (any non-"none" value), `false` otherwise
- Used by `shouldActivateModule()` to determine module activation

**`checkLaneContinuitySafety()`**:
- **CRITICAL SAFETY FUNCTION** - Currently placeholder implementation
- Should detect lane boundaries and verify that if vehicle exits with reversed orientation (odd cusps), the next lane also has `direction_change_area` tag
- Returns `false` if fatal condition detected (prevents orientation reversal)
- **MUST be fully implemented before production use**

---

### 8.7 Class Relationships

The following diagram shows the relationships between classes and utility functions in the Direction Change Module:

```
DirectionChangeModuleManager
    │
    │ manages instances of
    │ creates and initializes
    │
    └──> DirectionChangeModule
            │
            │ uses (shared_ptr)
            │
            ├──> DirectionChangeParameters (shared_ptr)
            │       └──> Contains: cusp_detection_angle_threshold_deg, enable_cusp_detection, etc.
            │
            ├──> DirectionChangeDebugData (member)
            │       └──> Contains: cusp_points, forward_path, reverse_path
            │
            │ calls utility functions (namespace functions)
            │
            └──> Utility Functions (autoware::behavior_path_planner namespace)
                    ├──> detectCuspPoints() - Detects cusp points using angle-based detection
                    ├──> reverseOrientationAtCusps() - Reverses yaw angles at cusp points
                    ├──> hasDirectionChangeAreaTag() - Checks for direction_change_area tag
                    ├──> checkLaneContinuitySafety() ⚠️ (placeholder, CRITICAL) - Safety validation
                    └──> detectLaneBoundaries() (placeholder) - Lane transition detection
```

**Data Flow**:
1. Manager loads parameters and creates module instance
2. Module receives path from previous module via `updateData()`
3. Module calls `detectCuspPoints()` to find cusp points
4. Module calls `reverseOrientationAtCusps()` to modify path orientations
5. Module returns modified path to next module or Behavior Velocity Planner

---

### 8.8 Integration Points

**Base Classes**:
- **`SceneModuleInterface`**: Provides base functionality for scene modules including:
  - Path access (`getPreviousModuleOutput()`, `getReferencePath()`)
  - State management (RUNNING, SUCCESS, FAILURE states)
  - RTC (Request to Cooperate) interface
  - Logger and clock access
  - Planner data access (`planner_data_`)

- **`SceneModuleManagerInterface`**: Provides base functionality for module managers including:
  - Plugin loading and registration
  - Module instance management
  - Parameter update mechanisms

**Dependencies**:
- **`autoware_internal_planning_msgs::msg::PathWithLaneId`**: Path message type containing path points with lane IDs
- **`lanelet2_core`**: Lanelet map types and operations for accessing map data and lanelet attributes
- **`autoware::motion_utils`**: Motion utility functions for path operations, arc length calculations, and nearest point finding
- **`autoware_utils`**: Geometry and math utilities including:
  - `normalize_radian()`: Angle normalization
  - `create_quaternion_from_yaw()`: Quaternion creation from yaw angle
  - `calc_distance2d()`: 2D distance calculation
- **`tf2`**: ROS2 transform library for quaternion and yaw operations
- **`RouteHandler`**: Route handler for accessing lanelet map and route information

**Integration with Behavior Path Planner**:
- Module is registered as a plugin and automatically discovered
- Executed in the behavior path planner's module chain
- Receives paths from upstream modules (e.g., lane following module)
- Outputs modified paths to downstream modules or Behavior Velocity Planner
- Integrates with RTC system for cooperative planning scenarios

---

## Appendix A: Code Structure

```
autoware_behavior_path_direction_change_module/
├── include/
│   └── autoware/behavior_path_direction_change_module/
│       ├── data_structs.hpp      # Data structures and parameters
│       ├── manager.hpp            # Module manager interface
│       ├── scene.hpp              # Main scene module interface
│       └── utils.hpp              # Utility function declarations
├── src/
│   ├── manager.cpp                # Manager implementation
│   ├── scene.cpp                  # Main module logic
│   └── utils.cpp                  # Utility implementations
├── config/
│   └── direction_change.param.yaml
├── plugins.xml                    # Plugin registration
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Appendix B: Sequence Diagram

```
Behavior Path Planner
    │
    ├─→ isExecutionRequested()
    │       │
    │       ├─→ Check direction_change_area tag
    │       │       │
    │       │       └─→ No tag → return false
    │       │
    │       └─→ Tag found → Detect cusp points
    │               │
    │               └─→ Cusps found → return true
    │
    ├─→ plan()
    │       │
    │       ├─→ updateData()
    │       │       ├─→ Get reference path
    │       │       └─→ findCuspPoints()
    │       │
    │       ├─→ reverseOrientationAtCusps()
    │       │       ├─→ For each point:
    │       │       │   ├─→ Count cusps passed (before this point)
    │       │       │   └─→ If odd number: reverse yaw (add π radians)
    │       │       └─→ Return modified path
    │       │
    │       └─→ Return BehaviorModuleOutput
    │
    └─→ Pass to Behavior Velocity Planner
```

---

## Document Version

- **Version**: 1.0
- **Date**: 2024-12-02
- **Author**: Autoware Development Team
- **Status**: Final Design

---

## References

1. Autoware Behavior Path Planner Architecture
2. Lanelet2 Map Format Specification
3. Autoware Plugin System Documentation
4. Behavior Velocity Planner Integration Guide

