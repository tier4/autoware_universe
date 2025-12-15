# CudaOcclusionPredictor Implementation TODOs

## Current Status: Core Pipeline Incomplete ⚠️

The CUDA blackboard migration is complete, but the **actual occlusion detection algorithm** has 7 major TODOs that need implementation.

---

## TODOs in `predict()` function

### ✅ COMPLETE: Steps 1-3
- [x] Step 1: Extract XYZ from point cloud
- [x] Step 2: Transform points to camera frame  
- [x] Step 3: Filter points by spatial bounds

### ❌ INCOMPLETE: Steps 4-9

#### Step 4: Compact Filtered Points (Thrust)
**Location**: Line 159
**Task**: Use Thrust to compact filtered points based on mask
**Complexity**: Medium
**Dependencies**: Thrust library
**Output**: Compacted array of valid points

#### Step 5: Convert to Spherical Coordinates
**Location**: Line 162
**Task**: Convert filtered 3D points to spherical (azimuth, elevation, distance)
**Complexity**: Medium
**Dependencies**: Spherical conversion kernel
**Output**: Array of Ray structures

#### Step 6: Sort Rays
**Location**: Line 165
**Task**: Sort rays by azimuth/elevation for efficient lookup
**Complexity**: Medium  
**Dependencies**: Thrust sorting
**Output**: Sorted ray array with spatial index

#### Step 7: Sample ROI Points
**Location**: Line 168
**Task**: Generate sample points within each ROI for occlusion testing
**Complexity**: High - needs careful geometric sampling
**Dependencies**: ROI 3D points, sampling parameters
**Output**: Array of sample ray directions per ROI

#### Step 8: Detect Occlusion
**Location**: Line 171
**Task**: For each ROI sample, check if ray hits obstacle before traffic light
**Complexity**: High - needs spatial search in sorted rays
**Dependencies**: Sorted rays, ROI samples, distance thresholds
**Output**: Per-sample occlusion flags

#### Step 9: Calculate Occlusion Ratios
**Location**: Line 174
**Task**: Aggregate per-sample results to per-ROI occlusion percentage
**Complexity**: Low
**Dependencies**: Occlusion flags
**Output**: Integer occlusion ratio (0-100) per ROI

---

## Implementation Strategy

### Phase 1: Point Compaction (Step 4)
**Goal**: Remove filtered-out points to reduce data size
**Approach**:
1. Use Thrust `copy_if` or manual scan+compact
2. Count valid points from mask
3. Compact points array

**Estimated Effort**: 1-2 hours

### Phase 2: Spherical Conversion (Step 5)
**Goal**: Convert 3D Cartesian to spherical coordinates
**Approach**:
1. Implement `convertToSphericalKernel`
2. Calculate: azimuth = atan2(y, x), elevation = atan2(z, sqrt(x²+y²)), distance = sqrt(x²+y²+z²)
3. Store in Ray structure

**Estimated Effort**: 1-2 hours

### Phase 3: Ray Sorting & Indexing (Step 6)
**Goal**: Create spatial index for efficient occlusion queries
**Approach**:
1. Sort rays by azimuth, then elevation (lexicographic)
2. Consider using angular bins for faster lookup
3. Store sorted indices

**Estimated Effort**: 2-3 hours

### Phase 4: ROI Sampling (Step 7) ⭐ MOST COMPLEX
**Goal**: Generate evenly-distributed sample rays for each ROI
**Approach**:
1. For each ROI (top_left, bottom_right):
   - Generate grid of sample points (horizontal_sample_num × vertical_sample_num)
   - Convert image coordinates to 3D rays in camera frame
   - Calculate expected azimuth/elevation for each sample
2. Challenge: Proper geometric sampling considering perspective projection

**Estimated Effort**: 3-4 hours

### Phase 5: Occlusion Detection (Step 8) ⭐ MOST COMPLEX
**Goal**: Check if obstacles occlude traffic light for each sample ray
**Approach**:
1. For each ROI sample ray:
   - Find rays in sorted array within angular tolerance
   - Check if any ray has distance < (traffic_light_distance - threshold)
   - Mark as occluded if obstacle found
2. Challenge: Efficient spatial search in sorted rays

**Estimated Effort**: 3-4 hours

### Phase 6: Ratio Calculation (Step 9)
**Goal**: Aggregate results to final occlusion percentage
**Approach**:
1. Count occluded samples per ROI
2. Calculate percentage: (occluded / total) × 100
3. Copy results back to host

**Estimated Effort**: 1 hour

---

## Mathematical Details

### Spherical Conversion
```
azimuth = atan2(y, x)                    // Horizontal angle [-π, π]
elevation = atan2(z, sqrt(x² + y²))     // Vertical angle [-π/2, π/2]
distance = sqrt(x² + y² + z²)           // Radial distance
```

### Angular Resolution
- Azimuth bins: 360° / azimuth_occlusion_resolution_deg
- Elevation bins: 180° / elevation_occlusion_resolution_deg
- Default: 0.5° → 720 × 360 angular grid

### Occlusion Criterion
```
For sample ray (az_sample, el_sample, dist_tl):
  Find all rays where:
    |azimuth - az_sample| < angular_tolerance
    |elevation - el_sample| < angular_tolerance
  
  If any ray satisfies:
    distance < (dist_tl - min_dist_from_occlusion_to_tl)
  Then:
    Sample is OCCLUDED
```

---

## Testing Strategy

### Unit Tests Needed
1. ✅ Empty point cloud (already done)
2. ✅ No ROIs (already done)
3. ✅ Null pointers (already done)
4. ❌ Point compaction correctness
5. ❌ Spherical conversion accuracy
6. ❌ Ray sorting order
7. ❌ ROI sampling coverage
8. ❌ Occlusion detection logic
9. ❌ Full pipeline with known occlusion

### Integration Tests Needed
1. ❌ Real point cloud data
2. ❌ Multiple ROIs
3. ❌ Partial occlusion scenarios
4. ❌ Performance benchmarks

---

## Performance Targets

### Expected Throughput
- Input: 100K points, 5 ROIs
- Target: < 5ms end-to-end latency
- Breakdown:
  - Transform + Filter: ~1ms
  - Compact + Convert: ~1ms
  - Sort: ~1ms
  - Sample + Detect: ~2ms

### Memory Usage
- Point cloud: ~1.6MB (100K × 16 bytes)
- Filtered points: ~400KB (25K after filtering)
- Rays: ~200KB (25K × 8 bytes)
- ROI samples: ~4KB (5 ROIs × 400 samples)
- **Total**: ~2.2MB peak

---

## Current Blockers

1. **No implementation** for Steps 4-9
2. **Disabled test** `OneROI` cannot pass without full pipeline
3. **Integration test** only checks node launch, not actual prediction

---

## Next Actions

### Immediate (Today)
1. Implement Step 4: Point compaction with Thrust
2. Implement Step 5: Spherical conversion kernel
3. Add unit tests for Steps 4-5

### Short Term (This Week)
4. Implement Step 6: Ray sorting
5. Implement Step 7: ROI sampling
6. Implement Steps 8-9: Occlusion detection and ratio calculation
7. Enable and fix `OneROI` test
8. Add comprehensive integration tests

### Long Term
9. Performance optimization
10. Multi-stream parallelization
11. Kernel fusion opportunities

---

## Estimated Total Effort

- **Remaining Implementation**: 12-16 hours
- **Testing & Validation**: 4-6 hours
- **Performance Tuning**: 2-4 hours
- **Documentation**: 2 hours

**Total**: ~20-28 hours of focused development work

---

**Status**: Ready to begin Step 4 implementation
**Priority**: HIGH - Core functionality incomplete
**Risk**: Medium - Mathematical complexity in Steps 7-8



