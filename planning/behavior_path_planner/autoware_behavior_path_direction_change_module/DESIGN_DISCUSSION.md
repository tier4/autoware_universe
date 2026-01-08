# Design Discussion: Cusp and Reverse Lane Following Module

## Overview

This module modifies path velocities by applying signed velocities (positive/negative) based on cusp point detection. The module is **activated only when `turn_area` tag is present** in the lanelet map.

## Core Requirements

1. **Activation Condition**: Module activates only if `turn_area` tag exists in lanelets along the path
2. **Cusp Detection**: Detect cusp points based on angle changes in the path
3. **Velocity Sign Assignment**: Apply negative velocity sign after cusp points, positive before
4. **Multi-Cusp Support**: Handle multiple cusp points in sequence
5. **Lane Continuity**: If exiting a lane in reverse mode, continue negative velocities in next lane (if it has the tag)

## Simplified Design (No State Machine)

### Key Principle
- **This module only sets velocity signs, not actual velocity values**
- Lower modules (Behavior Velocity Planner) handle actual velocity planning
- Module operates on path points and modifies `longitudinal_velocity_mps` sign

---

## Design Questions & Decisions Needed

### 1. Cusp Point Detection

**Question**: How should we detect cusp points?

**Proposed Approach**:
- Calculate angle difference between consecutive path segments
- If angle change exceeds threshold (e.g., > 90° or > 135°), mark as cusp point
- Consider both:
  - **Yaw angle change** (orientation of path points)
  - **Geometric angle** (direction vector between points)

**Decision**:
- ✅ **Angle threshold**: Configurable parameter (default: 90° or 135°)
- ✅ **Use path point yaws** (not geometric angle)
- [ ] Minimum distance between cusp points to avoid false positives?

---

### 2. Velocity Sign Assignment Logic

**Question**: How should velocity signs be assigned?

**Proposed Approach**:
```
1. Start with positive velocity (forward) for all points
2. For each cusp point detected:
   - All points BEFORE cusp: positive velocity (V > 0)
   - All points AT and AFTER cusp: negative velocity (V < 0)
3. For multiple cusp points:
   - Each cusp toggles the sign
   - Cusp 1: + → - (at cusp 1)
   - Cusp 2: - → + (at cusp 2)
   - Cusp 3: + → - (at cusp 3)
   - etc.
```

**Decision Needed**:
- [ ] Should velocity at cusp point itself be 0 or negative?
- [ ] How many path points should be affected at the cusp transition?
- [ ] Should we apply smoothing/transition zone at cusp points?

---

### 3. Centerline Points

**Question**: What does "find centerline points" mean?

**Proposed Approach**:
- Use the path points directly (they should already be on centerline)
- OR extract centerline from lanelet geometry if needed
- Path points from previous modules should already represent centerline

**Decision Needed**:
- [ ] Do we need to extract centerline separately, or use existing path points?
- [ ] Should we resample path points for better cusp detection?

---

### 4. Multi-Cusp Handling

**Question**: How to handle multiple cusp points?

**Proposed Approach**:
```
Path: [P0] --[+]--> [Cusp1] --[-]--> [Cusp2] --[+]--> [Cusp3] --[-]--> [P_end]

Velocity signs toggle at each cusp:
- Before Cusp1: +
- After Cusp1: -
- After Cusp2: +
- After Cusp3: -
```

**Decision Needed**:
- [ ] Confirmation: Each cusp toggles the sign?
- [ ] What if there are an even/odd number of cusp points?

---

### 5. Lane Continuity (Tag Propagation)

**Question**: How to handle velocity sign when transitioning between lanes?

**Proposed Approach**:
```
Scenario:
- Lane A (has turn_area tag): Exits in reverse mode (V < 0)
- Lane B (has turn_area tag): Should continue with V < 0
- Lane C (no turn_area tag): Module not active, velocity handled by other modules

Logic:
1. Track current velocity sign at end of current lane
2. If next lane has turn_area tag:
   - Continue with same velocity sign
   - Check for cusp points in next lane
   - Apply sign changes based on cusps
3. If next lane doesn't have turn_area tag:
   - Module stops modifying velocities
   - Let other modules handle it
```

**Decision Needed**:
- [ ] Should we check lane boundaries or use path point lane_ids?
- [ ] How to detect lane transitions?
- [ ] What if a lane has the tag but no cusp points? (Keep previous sign or reset to positive?)

---

### 6. Path Point Processing

**Question**: How to process path points efficiently?

**Proposed Approach**:
```cpp
1. Check if turn_area tag exists → if not, return early
2. Detect all cusp points in path (single pass)
3. Create sign map: point_index → velocity_sign
4. Apply signs to path points
```

**Decision Needed**:
- [ ] Process all points at once or incrementally?
- [ ] Should we modify path in-place or create new path?

---

## Proposed Algorithm Flow

```
1. isExecutionRequested():
   - Check if turn_area tag exists in any lanelet along path
   - If no tag → return false (module inactive)
   - If tag exists → return true

2. plan():
   a. Get reference path from previous module
   b. Extract centerline points (or use path points directly)
   c. Detect cusp points:
      - For each path segment, calculate angle change
      - If angle change > threshold → mark as cusp
   d. Assign velocity signs:
      - Initialize all points with positive sign
      - For each cusp point:
        - Toggle sign for all points after this cusp
   e. Apply signs to path:
      - For each point: velocity_mps *= sign
   f. Return modified path

3. Handle lane transitions:
   - Track velocity sign at path end
   - If next lane has turn_area tag:
     - Continue with current sign
     - Detect cusps in next lane
     - Apply sign changes
```

---

## Implementation Simplifications

### Remove:
- ❌ State machine (IDLE, FORWARD_FOLLOWING, etc.)
- ❌ Status tracking
- ❌ Complex state transitions
- ❌ Velocity limit management (handled by lower modules)
- ❌ Stop distance calculations

### Keep:
- ✅ turn_area tag checking
- ✅ Cusp point detection
- ✅ Velocity sign assignment
- ✅ Multi-cusp support
- ✅ Lane continuity logic

---

## Open Questions for Discussion

1. **Cusp Detection**: ✅ **DECIDED**
   - ✅ Angle threshold: **Configurable parameter** (default value TBD)
   - ✅ Use **path point yaws** (not geometric direction vectors)

2. **Velocity at Cusp Point**:
   - Should velocity be exactly 0 at cusp point?
   - Or should it be negative (reverse) immediately after?

3. **Transition Smoothing**:
   - Do we need a transition zone around cusp points?
   - Or instant sign change?

4. **Lane Boundary Detection**:
   - How to detect when we transition to a new lane?
   - Use lane_ids in path points?

5. **Tag Checking**:
   - Should we check tag for each lanelet in path?
   - Or just check once at activation?

6. **Multiple Cusp Points**:
   - Confirmation: Each cusp toggles the sign?
   - What's the expected behavior for 2, 3, 4+ cusp points?

7. **Path Modification**:
   - Modify path in-place or create copy?
   - Should we preserve original velocity magnitudes?

---

## Proposed Data Structures

```cpp
// Simplified - no complex state
struct CuspReverseLaneFollowingParameters {
  double cusp_detection_angle_threshold_deg;  // ✅ Configurable (uses path point yaws)
  bool enable_cusp_detection;
  bool publish_debug_marker;
};

// No status enum needed - just process and return
```

---

## Next Steps

1. **Clarify requirements** based on questions above
2. **Finalize algorithm** for cusp detection and sign assignment
3. **Define lane transition logic** for tag propagation
4. **Implement simplified version** without state machine
5. **Test with multiple cusp scenarios**

---

## Example Scenarios

### Scenario 1: Single Cusp
```
Path: [A] --[+]--> [CUSP] --[-]--> [B]
Result: Positive before cusp, negative after
```

### Scenario 2: Two Cusps
```
Path: [A] --[+]--> [CUSP1] --[-]--> [CUSP2] --[+]--> [B]
Result: Toggle at each cusp
```

### Scenario 3: Lane Transition with Tag
```
Lane1 (tag): [A] --[+]--> [CUSP] --[-]--> [end]
Lane2 (tag): [start] --[-]--> [B]  (continues negative)
Result: Negative velocity continues into next lane
```

### Scenario 4: Lane Transition without Tag
```
Lane1 (tag): [A] --[+]--> [CUSP] --[-]--> [end]
Lane2 (no tag): [start] --[?]--> [B]  (module inactive)
Result: Module stops, other modules handle Lane2
```

---

Please review and provide answers to the questions marked with [ ] above.

