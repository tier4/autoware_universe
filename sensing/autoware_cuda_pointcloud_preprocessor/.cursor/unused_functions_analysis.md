# Unused Functions Analysis - DAG Implementation

**Date**: 2025-12-10  
**Analysis Scope**: DAG executor, filters, registry

---

## Summary

Found **5 unused functions/fields** that can be safely removed:

1. ✂️ `DagExecutor::getExecutionOrder()` - Only used in tests
2. ✂️ `FilterRegistry::listFilters()` - Never called
3. ✂️ `FilterRegistry::getMetadata()` - Only used in tests  
4. ✂️ `ConsumerInfo::is_published` - Field never accessed
5. ✂️ `DagNode::initialized` - Field set but never read

---

## Detailed Analysis

### 1. ❌ `DagExecutor::getExecutionOrder()` (UNUSED IN PRODUCTION)

**Location**: `dag_executor.hpp` line 79, `dag_executor.cpp` line 159

**Usage Count**: 3 occurrences
- 1x declaration (header)
- 1x implementation (source)
- 1x test call (`test_dag_executor.cpp`)

**Purpose**: Returns topologically sorted node indices

**Analysis**: 
- Never called in production code
- Only used in test `TEST_F(DagExecutorTest, ExecutionOrder)`
- `execute()` calls `topologicalSort()` directly (private method)

**Recommendation**: ✂️ **REMOVE** - Can be added back if needed for debugging/introspection

```cpp
// REMOVE FROM dag_executor.hpp (line 79)
std::vector<std::size_t> getExecutionOrder() const;

// REMOVE FROM dag_executor.cpp (line 159)
std::vector<std::size_t> DagExecutor::getExecutionOrder() const { 
  return topologicalSort(); 
}
```

---

### 2. ❌ `FilterRegistry::listFilters()` (NEVER CALLED)

**Location**: `filter_registry.hpp` line 66, `filter_registry.cpp` line 50

**Usage Count**: 2 occurrences (declaration + implementation only)

**Purpose**: Returns list of all registered filter names

**Analysis**:
- No calls found in entire codebase
- Not used in tests
- Not used for validation or introspection
- Registry validation uses `isRegistered()` instead

**Recommendation**: ✂️ **REMOVE** - No evidence of use

```cpp
// REMOVE FROM filter_registry.hpp (line 66)
std::vector<std::string> listFilters() const;

// REMOVE FROM filter_registry.cpp (lines 50-57)
std::vector<std::string> FilterRegistry::listFilters() const
{
  std::vector<std::string> filters;
  filters.reserve(factories_.size());
  for (const auto & [name, _] : factories_) {
    filters.push_back(name);
  }
  return filters;
}
```

---

### 3. ⚠️ `FilterRegistry::getMetadata()` (TEST-ONLY)

**Location**: `filter_registry.hpp` line 60, `filter_registry.cpp` line 41

**Usage Count**: 18 files match, but analysis shows:
- All filter implementations define `getMetadata()` (required by interface)
- Registry's `getMetadata()` only used in tests (`test_filter_registry.cpp`)
- **NOT used during DAG execution** - filters validate themselves directly

**Analysis**:
- Part of public API but not used in production
- Tests verify metadata correctness
- Could be useful for debugging/CLI tools

**Recommendation**: ⚠️ **KEEP** (but mark as test/debug utility)

Alternative: If strictly production-only, could be removed.

---

### 4. ❌ `ConsumerInfo::is_published` (FIELD NEVER ACCESSED)

**Location**: `dag_executor.hpp` line 96

**Usage Count**: 
- 1x declaration
- 0x writes
- 0x reads

**Purpose**: Intended to track if output is published externally (not just consumed by other nodes)

**Analysis**:
- Field is declared but never set or checked
- Consumer tracking only looks at `consumer_node_indices` and `remaining_consumers`
- Original design may have intended different behavior for published outputs

**Current Code**:
```cpp
struct ConsumerInfo {
  std::vector<std::size_t> consumer_node_indices;
  int remaining_consumers{0};
  bool is_published{false};  // ❌ Never used
};
```

**Recommendation**: ✂️ **REMOVE** 

```cpp
struct ConsumerInfo {
  std::vector<std::size_t> consumer_node_indices;
  int remaining_consumers{0};
};
```

---

### 5. ❌ `DagNode::initialized` (WRITE-ONLY FIELD)

**Location**: `dag_executor.hpp` line 86, `dag_executor.cpp` line 43

**Usage Count**:
- 1x declaration
- 1x write (`node.initialized = true;`)
- 0x reads

**Purpose**: Track if node's filter has been initialized

**Analysis**:
- Set to `true` after `filter->initialize()` call
- Never checked anywhere
- Error handling happens via exceptions, not flag checking

**Current Code**:
```cpp
struct DagNode {
  DagNodeConfig config;
  std::unique_ptr<IFilter> filter;
  bool initialized{false};  // ❌ Set but never read
};

// In buildDag():
filter->initialize(config.parameters);
node.filter = std::move(filter);
node.initialized = true;  // ❌ Useless
```

**Recommendation**: ✂️ **REMOVE**

```cpp
struct DagNode {
  DagNodeConfig config;
  std::unique_ptr<IFilter> filter;
};
```

---

## Fields That Look Unused But Are NOT

### ✅ `TypedInputs::pointclouds` - **LEGACY FIELD**

**Status**: Currently replaced by `processing_states`

**Analysis**:
- Old field: `std::map<std::string, std::shared_ptr<cuda_blackboard::CudaPointCloud2>> pointclouds;`
- New field: `std::map<std::string, std::shared_ptr<PointcloudProcessingState>> processing_states;`
- All filters now use `processing_states`
- Search shows 0 references to `.pointclouds` in filter implementations

**Recommendation**: ⚠️ **VERIFY AND REMOVE** if confirmed unused

**Check needed**:
```bash
grep -r "inputs\.pointclouds" src/dag/filters/
grep -r "TypedInputs.*pointclouds" src/dag/
```

If no matches, remove from `filter_interface.hpp`:
```cpp
struct TypedInputs
{
  // REMOVE this line if confirmed unused:
  // std::map<std::string, std::shared_ptr<cuda_blackboard::CudaPointCloud2>> pointclouds;
  
  std::map<std::string, std::shared_ptr<PointcloudProcessingState>> processing_states;
  std::vector<std::string> special_inputs;
};
```

---

## Impact Analysis

### Files to Modify

1. **include/autoware/cuda_pointcloud_preprocessor/dag/dag_executor.hpp**
   - Remove `getExecutionOrder()` declaration (line 79)
   - Remove `ConsumerInfo::is_published` field (line 96)
   - Remove `DagNode::initialized` field (line 86)

2. **src/dag/dag_executor.cpp**
   - Remove `getExecutionOrder()` implementation (line 159)
   - Remove `node.initialized = true;` assignment (line 43)

3. **include/autoware/cuda_pointcloud_preprocessor/dag/filter_registry.hpp**
   - Remove `listFilters()` declaration (line 66)

4. **src/dag/filter_registry.cpp**
   - Remove `listFilters()` implementation (lines 50-57)

5. **test/test_dag_executor.cpp** (IF removing getExecutionOrder)
   - Update test to use alternative approach or remove test

### Build Impact

- ✅ No ABI breakage (private methods/fields only)
- ✅ No API breakage for external users
- ⚠️ Tests may need updates if `getExecutionOrder()` removed

### Lines Saved

- **Header files**: ~6 lines
- **Source files**: ~10 lines
- **Total**: ~16 lines + improved clarity

---

## Recommendations Priority

### High Priority (Remove Immediately)
1. ✂️ `ConsumerInfo::is_published` - Dead field, confusing
2. ✂️ `DagNode::initialized` - Write-only, no value
3. ✂️ `FilterRegistry::listFilters()` - Completely unused

### Medium Priority (Consider Removing)
4. ✂️ `DagExecutor::getExecutionOrder()` - Test-only, but harmless

### Low Priority (Keep for Now)
5. ⚠️ `FilterRegistry::getMetadata()` - Useful for debugging
6. ⚠️ `TypedInputs::pointclouds` - Need verification first

---

## Action Plan

```bash
# Step 1: Remove confirmed unused items
# Edit dag_executor.hpp - remove 3 items
# Edit dag_executor.cpp - remove 2 items  
# Edit filter_registry.hpp - remove 1 item
# Edit filter_registry.cpp - remove 1 function

# Step 2: Verify TypedInputs::pointclouds is unused
grep -r "inputs\.pointclouds" src/dag/filters/
# If no matches, remove from filter_interface.hpp

# Step 3: Build and test
colcon build --packages-select autoware_cuda_pointcloud_preprocessor
colcon test --packages-select autoware_cuda_pointcloud_preprocessor

# Step 4: Update tests if needed
# Modify test_dag_executor.cpp to remove getExecutionOrder() test
```

---

## Conclusion

**Total Removable**: 5 functions/fields (16+ lines of code)

**Safety**: High - all removals are internal implementation details

**Benefit**: 
- Reduced maintenance burden
- Clearer code (no misleading unused fields)
- Faster compilation (marginally)

**Risk**: Low - no API changes for external users

---

*Generated by Code Quality Analysis Tool*



