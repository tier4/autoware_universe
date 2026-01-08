# Design Decisions: Cusp and Reverse Lane Following Module

## ✅ Decided

### 1. Cusp Detection
- **Angle threshold**: Configurable parameter (`cusp_detection_angle_threshold_deg`)
- **Default value**: 90.0 degrees (in config file)
- **Method**: Use **path point yaws** (not geometric direction vectors)
- **Implementation**: Compare yaw angles between consecutive path points

### 2. Module Activation
- **Condition**: Module activates only when `turn_area` tag is present in lanelets
- **Check locations**: 
  - Reference lanelets (`current_lanelets_`)
  - Lanelets from path points (via `lane_ids`)

### 3. Architecture Simplification
- **No state machine**: Remove complex state management
- **Simple processing**: Check tag → detect cusps → apply signs → return path
- **Velocity handling**: Lower modules (BVP) handle actual velocity values

---

## ✅ All Decisions Finalized

### 2. Velocity Sign Assignment Logic ✅
- ✅ **Velocity at cusp point**: **0** (zero)
- ✅ **Velocity after cusp**: **Negative** (reverse)
- ✅ **Transition**: **Instant sign change** (comment added for future smooth transition)
- ✅ **Toggle behavior**: 
  - Before first cusp: **Positive** (+)
  - After first cusp: **Negative** (-)
  - After second cusp: **Positive** (+) (toggles)
  - After third cusp: **Negative** (-) (toggles)
  - And so on...

### 3. Centerline Points ✅
- ✅ Use existing path points (they already represent centerline)
- ✅ No need to extract separately

### 4. Multi-Cusp Handling ✅
- ✅ **Confirmed**: Each cusp toggles the sign
- ✅ Pattern: + → (Cusp1) → - → (Cusp2) → + → (Cusp3) → - → ...

### 5. Lane Continuity (Tag Propagation) ✅
- ✅ **Check tag for each lanelet** in the path
- ✅ **Lane boundary detection**: Methods written but **implementation postponed**
- ✅ Use path point `lane_ids` to check each lanelet

### 6. Path Point Processing ✅
- ✅ **For debug**: Create copy/print for debugging
- ✅ **For final implementation**: **Overwrite** path in-place

---

## Current Implementation Status

### ✅ Implemented
- Cusp detection using path point yaws with configurable threshold
- `turn_area` tag checking in activation logic
- Basic velocity sign assignment (toggles at each cusp)

### 🔄 Needs Refinement
- Velocity sign assignment logic (pending decisions above)
- Lane continuity handling (pending decisions above)
- Multi-cusp handling verification

---

## Next Steps

1. Answer remaining questions in "Pending Decisions" section
2. Refine implementation based on decisions
3. Test with multiple cusp scenarios
4. Verify lane transition behavior

