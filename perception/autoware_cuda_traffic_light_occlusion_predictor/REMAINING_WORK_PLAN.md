# Remaining Work Plan - Simplified Approach

## Current Situation

I've added the high-level pipeline structure, but there are signature mismatches and the implementation is too ambitious for immediate completion. Let me take a **step-by-step incremental approach**.

## Revised Strategy: Incremental Implementation

### Phase 1: Get Basic Pipeline Working (TODAY - 2-3 hours)
**Goal**: Implement minimal working version that compiles and passes basic tests

1. **Step 4**: Point compaction ✅ (kernel added)
2. **Step 5**: Spherical conversion ✅ (kernel added)  
3. **Step 6**: Sort rays ✅ (Thrust call added)
4. **Steps 7-9**: Simplify detectOcclusionLaunch to work with current API

### Phase 2: Optimize & Complete (LATER - 4-6 hours)
5. Improve ROI sampling accuracy
6. Optimize occlusion detection with spatial indexing
7. Add comprehensive tests
8. Performance tuning

---

## Immediate Actions

### Action 1: Fix detectOcclusionLaunch Call ⚠️

**Current signature in header**:
```cpp
void detectOcclusionLaunch(
  const PointXYZ * sample_points,      // Sample points (not rays!)
  const Ray * lidar_rays,
  const int * ray_azimuth_indices,      // Spatial indices
  const int * ray_elevation_indices,
  uint32_t * occlusion_mask,
  float azimuth_tolerance,
  ...);
```

**What I called**:
```cpp
kernels::detectOcclusionLaunch(
  d_sample_rays,                        // Wrong: Ray*, not PointXYZ*
  d_rays, 
  static_cast<int>(num_filtered), 
  d_occlusion_flags,                    // Wrong order!
  ...);
```

**Fix needed**: Update the call in predict() to match the existing signature

### Action 2: Understand Existing detect OcclusionKernel

The current kernel expects:
- `sample_points` as `PointXYZ*` (not converted to Ray yet)
- `ray_azimuth_indices` and `ray_elevation_indices` for spatial lookup
- Kernel will convert samples to rays internally

**This means**: I don't need to pre-convert ROI samples to rays!

---

## Corrected Implementation Steps

### Immediate Fix (30 min)

1. Keep compactPointsKernel and convertToSphericalKernel ✅
2. Update predict() to use correct detectOcclusionLaunch signature
3. Create simple spatial indices (azimuth/elevation bins)
4. Build and test

### Code Changes Needed

**In predict(), replace Step 7-9 with**:
```cpp
// For each ROI
for (size_t roi_idx = 0; roi_idx < num_rois; ++roi_idx) {
  const PointXYZ & top_left = roi_3d_points[roi_idx * 2];
  const PointXYZ & bottom_right = roi_3d_points[roi_idx * 2 + 1];
  
  const int num_samples = params_.horizontal_sample_num * params_.vertical_sample_num;
  PointXYZ * d_sample_points = allocateBufferFromPool<PointXYZ>(num_samples);
  uint32_t * d_occlusion_mask = allocateBufferFromPool<uint32_t>(num_samples);
  
  // Generate sample POINTS (not rays) for ROI
  const int sample_blocks = (num_samples + threads_per_block_ - 1) / threads_per_block_;
  kernels::sampleRoiLaunch(
    top_left, bottom_right, d_sample_points,  // Output: PointXYZ*
    params_.horizontal_sample_num, params_.vertical_sample_num,
    threads_per_block_, sample_blocks, stream_);
  
  // TODO: Create spatial indices for rays (azimuth/elevation bins)
  // For now, use nullptr and let kernel do linear search
  
  // Detect occlusion
  kernels::detectOcclusionLaunch(
    d_sample_points,          // PointXYZ* (not Ray*)
    d_rays,                   // LiDAR rays (sorted)
    nullptr,                  // TODO: azimuth indices
    nullptr,                  // TODO: elevation indices
    d_occlusion_mask,         // Output mask
    params_.azimuth_occlusion_resolution_deg,
    params_.elevation_occlusion_resolution_deg,
    params_.min_dist_from_occlusion_to_tl,
    num_samples,
    static_cast<int>(num_filtered),
    threads_per_block_,
    sample_blocks,
    stream_);
  
  // Calculate ratio
  uint32_t occluded_count = thrust::reduce(
    thrust::cuda::par_nosync.on(stream_),
    thrust::device_ptr<uint32_t>(d_occlusion_mask),
    thrust::device_ptr<uint32_t>(d_occlusion_mask + num_samples),
    0U);
  
  occlusion_ratios[roi_idx] = (100 * occluded_count) / num_samples;
  
  returnBufferToPool(d_sample_points);
  returnBufferToPool(d_occlusion_mask);
}
```

### sampleRoiLaunch Signature

**Expected**:
```cpp
void sampleRoiLaunch(
  const PointXYZ & top_left,
  const PointXYZ & bottom_right,
  PointXYZ * output_points,              // Not Ray*!
  int horizontal_sample_num,
  int vertical_sample_num,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream);
```

---

## Testing Plan

### Step 1: Build Test
```bash
colcon build --packages-select autoware_cuda_traffic_light_occlusion_predictor
```
**Expected**: Compilation succeeds

### Step 2: Run Unit Tests
```bash
colcon test --packages-select autoware_cuda_traffic_light_occlusion_predictor
```
**Expected**: Basic tests still pass

### Step 3: Enable OneROI Test
- Un-DISABLE the test
- Run with small point cloud
- **Expected**: May fail initially, but should show progress

---

## Timeline

**Today (Remaining ~2 hours)**:
- [x] Understand current API
- [ ] Fix predict() implementation (30 min)
- [ ] Build and fix compilation errors (30 min)
- [ ] Run tests (15 min)
- [ ] Debug and iterate (45 min)

**Tomorrow**:
- [ ] Add spatial indexing for rays
- [ ] Optimize detection kernel
- [ ] Comprehensive testing
- [ ] Performance benchmarks

---

**Current Status**: Ready to fix predict() implementation
**Next Action**: Update cuda_occlusion_predictor.cu with corrected API calls



