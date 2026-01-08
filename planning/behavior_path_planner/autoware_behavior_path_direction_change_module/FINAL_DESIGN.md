# Final Design: Cusp and Reverse Lane Following Module

## ✅ All Design Decisions Finalized

### 1. Cusp Detection ✅
- **Method**: Use path point yaws (from `point.pose.orientation`)
- **Threshold**: Configurable parameter `cusp_detection_angle_threshold_deg` (default: 90.0°)
- **Algorithm**: Compare yaw angles between consecutive path points

### 2. Velocity Sign Assignment ✅
- **At cusp point**: Velocity = **0** (zero)
- **After cusp point**: Velocity = **negative** (reverse)
- **Before first cusp**: Velocity = **positive** (forward)
- **Toggle behavior**: Each cusp toggles the sign
  - Before Cusp1: **+**
  - After Cusp1: **-**
  - After Cusp2: **+**
  - After Cusp3: **-**
  - And so on...
- **Transition**: **Instant sign change** (comment added for future smooth transition)

### 3. Centerline Points ✅
- Use existing path points (they already represent centerline)
- No need to extract separately

### 4. Multi-Cusp Handling ✅
- **Confirmed**: Each cusp toggles the sign
- Pattern: + → (Cusp1) → - → (Cusp2) → + → (Cusp3) → - → ...

### 5. Lane Continuity (Tag Propagation) ✅
- **Check tag for each lanelet** in the path
- Use path point `lane_ids` to check each lanelet
- **Lane boundary detection**: Methods written but **implementation postponed**

### 6. Path Point Processing ✅
- **For debug**: Create copy/print for debugging (via `#ifdef DEBUG_MODE`)
- **For final implementation**: **Overwrite** path in-place

### 7. Architecture Simplification ✅
- **No state machine**: Removed complex state management
- **Simple processing**: Check tag → detect cusps → apply signs → return path
- **Velocity handling**: Lower modules (BVP) handle actual velocity values

---

## Implementation Summary

### Core Algorithm

```
1. isExecutionRequested():
   - Check if turn_area tag exists in any lanelet along path
   - If no tag → return false (module inactive)
   - If tag exists → return true (module active, regardless of cusp points)

2. plan():
   a. Get reference path from previous module
   b. Detect cusp points using path point yaws
   c. If cusp points exist:
      - Apply velocity signs:
        - At cusp: velocity = 0
        - Before first cusp: positive
        - After each cusp: toggle sign
   d. If no cusp points:
      - Return path unchanged (forward velocities only)
   e. Overwrite path in-place (or create debug copy)
   f. Return path (always returns a path when tag exists)
```

### Velocity Sign Logic

```cpp
For each path point i:
  if (i is cusp point):
    velocity[i] = 0.0
  else:
    sign = 1.0  // Start positive
    for each cusp_idx in cusp_indices:
      if (i > cusp_idx):
        sign *= -1.0  // Toggle at each cusp
    velocity[i] = abs(original_velocity) * sign
```

### Example Scenarios

#### Single Cusp
```
Path: [P0: +] → [P1: +] → [CUSP: 0] → [P2: -] → [P3: -]
```

#### Two Cusps
```
Path: [P0: +] → [CUSP1: 0] → [P1: -] → [CUSP2: 0] → [P2: +] → [P3: +]
```

#### Three Cusps
```
Path: [P0: +] → [CUSP1: 0] → [P1: -] → [CUSP2: 0] → [P2: +] → [CUSP3: 0] → [P3: -]
```

---

## Code Structure

### Key Functions

1. **`detectCuspPoints()`**: Detects cusp points using path point yaws
2. **`applySignedVelocityToPath()`**: Applies velocity signs based on cusp points
3. **`hasTurnAreaTag()`**: Checks if lanelet has turn_area tag
4. **`detectLaneBoundaries()`**: Placeholder for future lane boundary detection

### Removed Components

- ❌ State machine (IDLE, FORWARD_FOLLOWING, etc.)
- ❌ `planForwardFollowing()`, `planReverseFollowing()`, `planAtCusp()`
- ❌ Complex status tracking
- ❌ Velocity limit management (handled by lower modules)

### Kept Components

- ✅ `turn_area` tag checking
- ✅ Cusp point detection
- ✅ Velocity sign assignment
- ✅ Multi-cusp support
- ✅ Debug mode support

---

## Future Enhancements (Commented in Code)

1. **Smooth Transition**: Add transition zone around cusp points instead of instant sign change
2. **Lane Boundary Detection**: Implement full lane transition detection logic
3. **Advanced Cusp Detection**: Enhance detection algorithm if needed

---

## Testing Checklist

- [ ] Single cusp point scenario
- [ ] Multiple cusp points (2, 3, 4+)
- [ ] Path with turn_area tag
- [ ] Path without turn_area tag (should not activate)
- [ ] Lane transitions with/without tag
- [ ] Velocity signs correctly applied
- [ ] Zero velocity at cusp points

---

## Configuration Parameters

```yaml
direction_change:
  cusp_detection_angle_threshold_deg: 90.0  # Configurable threshold
  enable_cusp_detection: true
  publish_debug_marker: false
```

---

## Integration Notes

- Module activates only when `turn_area` tag is present
- Checks each lanelet in the path for the tag
- Applies signed velocities for downstream modules (BVP)
- Lower modules handle actual velocity planning

