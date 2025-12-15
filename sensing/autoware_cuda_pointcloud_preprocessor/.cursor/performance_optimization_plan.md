# DAG Performance Optimization Implementation

**Date**: 2025-12-10  
**Goal**: Achieve 2-3x speedup by eliminating unnecessary allocations, copies, and synchronizations

---

## Changes Being Implemented

### 1. Internalize Organize/Finalize
- **Organize**: Automatically called on external input in node callback
- **Finalize**: Automatically called when preparing output for publishing
- **YAML**: Users no longer configure OrganizeFilter or FinalizeFilter

### 2. BFS Execution with Smart Copy-on-Write
- **Execution Order**: Breadth-First Search (level by level)
- **Copy Logic**: Only the last child gets direct pointer, others get copies
- **Tracking**: Analyze DAG to identify branching points

### 3. Remove Unnecessary Synchronization
- **Public Methods**: No cudaStreamSynchronize in filter methods
- **Sync Points**: Only before publishing and at DAG completion

### 4. Shared Pointer Throughout Pipeline
- **Internal**: Use `shared_ptr<CudaPointCloud2>` in DAG execution
- **Publishing**: Convert to `unique_ptr` only when publishing

---

## Implementation Status

- [x] Plan documented
- [ ] Filter interface updated
- [ ] Organize internalized
- [ ] Finalize internalized
- [ ] BFS execution implemented
- [ ] Copy-on-write implemented
- [ ] Sync removed from methods
- [ ] All filters updated
- [ ] Tests updated
- [ ] Build successful

---

## Breaking Changes

1. **Filter Interface**: `process()` now works with shared_ptr
2. **YAML Format**: OrganizeFilter and FinalizeFilter removed from user configs
3. **Performance**: Filters must not sync internally (breaking for custom filters)

---

## Expected Results

**Before**:
- 6 allocations per frame
- 6 copies per frame
- 3+ synchronizations per frame

**After**:
- 1 allocation per frame (for publishing)
- 1 copy per frame (for publishing)
- 1 synchronization per frame (before publish)

**Speedup**: 2-3x faster ⚡

