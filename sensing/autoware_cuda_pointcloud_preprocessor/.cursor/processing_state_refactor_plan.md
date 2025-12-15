# Processing State Refactor Plan

## Goal
Use `PointcloudProcessingState` (lightweight metadata + raw GPU pointer) between filters instead of `shared_ptr<CudaPointCloud2>` to achieve true zero-copy processing.

---

## Architecture

### Data Flow
```
External Input: CudaPointCloud2
         ↓
    [OrganizeFilter]
         ↓
PointcloudProcessingState (device_data → device_organized_points_)
         ↓
    [TransformFilter]
         ↓
PointcloudProcessingState (device_data → device_transformed_points_)
         ↓
    [CropBoxFilter] (masks only, device_data unchanged)
         ↓
    [DistortionFilter] (in-place on device_data)
         ↓
    [RingOutlierFilter] (masks only, device_data unchanged)
         ↓
    [FinalizeFilter]
         ↓
External Output: CudaPointCloud2
```

###key Insight
- `state.device_data` ALWAYS points to `CudaPointcloudPreprocessor`'s internal buffers
- NEVER points to CudaPointCloud2's managed memory
- This is safe because preprocessor instance lives for entire node lifetime

---

## Implementation Checklist

### Phase 1: Core Preprocessor Methods ✅
- [x] `transformPointcloudPublic(PointcloudProcessingState &)`
- [x] `correctDistortionPublic(PointcloudProcessingState &)`
- [ ] `applyCropBoxPublic(PointcloudProcessingState &)`
- [ ] `applyRingOutlierFilterPublic(PointcloudProcessingState &)`

### Phase 2: Entry/Exit Filters
- [ ] `OrganizeFilter`: CudaPointCloud2 → PointcloudProcessingState
  - Copy input to `device_organized_points_`
  - Create state with `device_data → device_organized_points_`
  
- [ ] `FinalizeFilter`: PointcloudProcessingState → CudaPointCloud2
  - Apply all masks
  - Compact points
  - Allocate new CudaPointCloud2
  - Copy compacted data

### Phase 3: Update TypedInputs
Change from:
```cpp
struct TypedInputs {
  std::map<std::string, shared_ptr<CudaPointCloud2>> pointclouds;
};
```

To:
```cpp
struct TypedInputs {
  std::map<std::string, shared_ptr<PointcloudProcessingState>> processing_states;
  std::vector<std::string> special_inputs;
};
```

### Phase 4: Update DAG Executor
- [ ] `prepareTypedInputs`: Cast to `PointcloudProcessingState` instead of `CudaPointCloud2`
- [ ] Input/output handling: Wrap entry points, unwrap exit points

### Phase 5: Update All Filters
- [ ] `TransformFilter` ✅ (already updated preprocessor method)
- [ ] `CropBoxFilter`
- [ ] `DistortionFilter` ✅ (already updated preprocessor method)
- [ ] `RingOutlierFilter`
- [ ] `DownsampleFilter`

### Phase 6: Update DAG Node
- [ ] Entry: CudaPointCloud2 from subscriber → OrganizeFilter → PointcloudProcessingState
- [ ] Exit: PointcloudProcessingState → FinalizeFilter → CudaPointCloud2 → publisher

---

## Memory Ownership Rules

### Rule 1: Processing State is Non-Owning
`PointcloudProcessingState::device_data` is a **raw pointer** - it does NOT own memory.

### Rule 2: Preprocessor Owns Internal Buffers
`device_organized_points_`, `device_transformed_points_`, etc. are owned by `CudaPointcloudPreprocessor` (via thrust::device_vector).

### Rule 3: CudaPointCloud2 Owns Its Memory
Entry (from ROS) and Exit (to ROS) use `CudaPointCloud2` which owns its GPU memory via `cuda_blackboard::CudaUniquePtr`.

### Rule 4: State Lifetime
`PointcloudProcessingState` objects live only during DAG execution (stack or short-lived heap). They become invalid after preprocessor internal buffers are reused.

---

## Performance Benefits

### Before (Current - shared_ptr<CudaPointCloud2>)
```
Organize: Allocate + Copy                    [1 alloc, 1 copy]
Transform: Allocate + Copy                   [1 alloc, 1 copy]
Crop: Update masks only                      [0 alloc, 0 copy]
Distortion: Allocate + Copy                  [1 alloc, 1 copy]
Outlier: Update masks only                   [0 alloc, 0 copy]
Finalize: Allocate + Compact + Copy          [1 alloc, 1 copy]
---------------------------------------------------
TOTAL:                                       [4 alloc, 4 copy]
```

### After (Processing State)
```
Organize: Copy to internal buffer            [0 alloc, 1 copy]
Transform: Work on internal buffer           [0 alloc, 0 copy] ← pointer update
Crop: Update masks only                      [0 alloc, 0 copy]
Distortion: In-place on internal buffer      [0 alloc, 0 copy] ← true in-place
Outlier: Update masks only                   [0 alloc, 0 copy]
Finalize: Allocate + Compact + Copy          [1 alloc, 1 copy]
---------------------------------------------------
TOTAL:                                       [1 alloc, 2 copy]
```

**Improvement**: 75% fewer allocations, 50% fewer copies! 🚀

---

## Implementation Strategy

1. **Start small**: Update one filter at a time
2. **Test incrementally**: Ensure each change compiles and works
3. **Maintain compatibility**: Keep both interfaces during transition if needed
4. **Document carefully**: This is a fundamental architecture change

---

## Current Status

- [x] `PointcloudProcessingState` struct defined
- [x] `transformPointcloudPublic` updated (works on state)
- [x] `correctDistortionPublic` updated (works on state)
- [ ] Other filters need updating
- [ ] DAG executor needs updating
- [ ] Entry/exit filters need implementing

**Next Step**: Update `applyCropBoxPublic` and `applyRingOutlierFilterPublic` to work with `PointcloudProcessingState`.

