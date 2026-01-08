# PlannerManager Call Chain Explanation

## Overview

This document explains why `plan()` is called multiple times per planning cycle in the Behavior Path Planner architecture. Understanding this call chain is crucial for debugging and optimizing module performance.

---

## Table of Contents

1. [Why Multiple Calls?](#why-multiple-calls)
2. [Call Chain Architecture](#call-chain-architecture)
3. [Detailed Call Flow](#detailed-call-flow)
4. [Code Locations](#code-locations)
5. [Example Execution Trace](#example-execution-trace)
6. [Implications for Module Development](#implications-for-module-development)

---

## Why Multiple Calls?

The Behavior Path Planner uses an **iterative refinement** approach:

1. **Approved Modules**: Modules that are already approved and running
2. **Candidate Modules**: New modules that want to execute
3. **Iterative Processing**: The system may need multiple iterations to:
   - Process approved modules
   - Evaluate candidate modules
   - Handle module transitions (candidate → approved)
   - Refine paths based on module outputs

This design allows modules to:
- Request new modules based on their output
- Handle priority conflicts
- Support simultaneous execution of compatible modules

---

## Call Chain Architecture

### High-Level Flow

```
PlannerManager::run()
  └─→ For each slot: SubPlannerManager::propagateFull()
        └─→ Iterative Loop (up to max_iteration_num)
              ├─→ runApprovedModules()  → plan() call #1
              └─→ runRequestModules()   → plan() call #2
```

### Detailed Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│ PlannerManager::run()                                       │
│ File: planner_manager.cpp:103                               │
│                                                              │
│  for (auto & slot : planner_manager_slots_) {              │
│    result = slot.propagateFull(data, result);               │
│  }                                                           │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│ SubPlannerManager::propagateFull()                          │
│ File: planner_manager.cpp:912                               │
│                                                              │
│  max_iteration_num = module_size * (module_size + 1) / 2    │
│                                                              │
│  for (itr = 0; itr < max_iteration_num; ++itr) {            │
│    ┌──────────────────────────────────────────┐            │
│    │ approved_result = runApprovedModules()   │            │
│    │   └─→ For each approved module:           │            │
│    │       └─→ run(module)                     │            │
│    │           └─→ module->run()              │            │
│    │               └─→ plan()  ← CALL #1     │            │
│    └──────────────────────────────────────────┘            │
│                                                              │
│    request_modules = getRequestModules()                    │
│                                                              │
│    ┌──────────────────────────────────────────┐            │
│    │ [module, output] = runRequestModules()   │            │
│    │   └─→ For each candidate module:          │            │
│    │       └─→ run(module)                     │            │
│    │           └─→ module->run()              │            │
│    │               └─→ plan()  ← CALL #2     │            │
│    └──────────────────────────────────────────┘            │
│                                                              │
│    if (module approved) {                                   │
│      addApprovedModule(module);                            │
│      continue;  // Next iteration                           │
│    }                                                        │
│  }                                                           │
└─────────────────────────────────────────────────────────────┘
```

---

## Detailed Call Flow

### Level 1: Entry Point - PlannerManager::run()

**File**: `planner_manager.cpp`  
**Line**: 103

```cpp
BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
  // Setup and initialization
  resetProcessingTime();
  // ... status checks ...
  
  // Line 166-180: Process each slot
  for (auto & planner_manager_slot : planner_manager_slots_) {
    if (result_output.is_upstream_failed_approved) {
      planner_manager_slot.propagateWithFailedApproved();
    } else if (result_output.is_upstream_waiting_approved) {
      result_output = planner_manager_slot.propagateWithWaitingApproved(data, result_output);
    } else if (result_output.is_upstream_candidate_exclusive) {
      result_output = planner_manager_slot.propagateWithExclusiveCandidate(data, result_output);
    } else {
      // Line 178: Normal case - calls propagateFull()
      result_output = planner_manager_slot.propagateFull(data, result_output);
    }
  }
  
  return result_output.valid_output;
}
```

**Purpose**: Main entry point called by `BehaviorPathPlannerNode` on each planning cycle.

---

### Level 2: Slot Propagation - SubPlannerManager::propagateFull()

**File**: `planner_manager.cpp`  
**Line**: 912

```cpp
SlotOutput SubPlannerManager::propagateFull(
  const std::shared_ptr<PlannerData> & data, 
  const SlotOutput & previous_slot_output)
{
  // Line 916: Calculate maximum iterations
  const size_t module_size = manager_ptrs_.size();
  const size_t max_iteration_num = static_cast<int>(module_size * (module_size + 1) / 2);
  
  bool is_waiting_approved_slot = previous_slot_output.is_upstream_waiting_approved;
  bool is_failed_approved_slot = false;
  auto output_path = previous_slot_output.valid_output;
  
  std::vector<SceneModulePtr> deleted_modules;
  
  // Line 923: ITERATIVE LOOP - This causes multiple plan() calls!
  for (size_t itr_num = 0; itr_num < max_iteration_num; ++itr_num) {
    
    // FIRST CALL PATH: Approved Modules
    // Line 925: Run all approved modules
    const auto approved_module_result =
      runApprovedModules(data, previous_slot_output.valid_output, deleted_modules);
    const auto & approved_module_output = approved_module_result.valid_output;
    
    // Update status flags
    is_waiting_approved_slot = 
      is_waiting_approved_slot || approved_module_result.is_upstream_waiting_approved;
    is_failed_approved_slot = 
      is_failed_approved_slot || approved_module_result.is_upstream_failed_approved;
    
    // Line 935: Get modules that want to execute
    const auto request_modules = getRequestModules(approved_module_output, deleted_modules);
    if (request_modules.empty()) {
      // No more modules to process, exit loop
      return SlotOutput{approved_module_output, ...};
    }
    
    // SECOND CALL PATH: Candidate/Request Modules
    // Line 943-944: Run candidate modules
    const auto [highest_priority_module, candidate_module_output] =
      runRequestModules(request_modules, data, approved_module_output);
    
    if (!highest_priority_module) {
      // No valid candidate, exit loop
      return SlotOutput{approved_module_output, ...};
    }
    
    if (highest_priority_module->isWaitingApproval()) {
      // Module waiting for approval, exit loop
      return SlotOutput{candidate_module_output, ...};
    }
    
    // Line 960-962: Module approved, add it and continue loop
    output_path = candidate_module_output;
    addApprovedModule(highest_priority_module);
    clearCandidateModules();
    // Loop continues → plan() may be called again in next iteration!
  }
  
  return SlotOutput{output_path, ...};
}
```

**Key Points**:
- **Iterative Loop**: Can iterate up to `module_size * (module_size + 1) / 2` times
- **Two Call Paths**: Approved modules and candidate modules are processed separately
- **Module Promotion**: Candidate modules can become approved, triggering new iterations

---

### Level 3A: Approved Modules Execution - SubPlannerManager::runApprovedModules()

**File**: `planner_manager.cpp`  
**Line**: 783

```cpp
SlotOutput SubPlannerManager::runApprovedModules(
  const std::shared_ptr<PlannerData> & data, 
  const BehaviorModuleOutput & upstream_slot_output,
  std::vector<SceneModulePtr> & deleted_modules)
{
  std::unordered_map<std::string, BehaviorModuleOutput> results;
  BehaviorModuleOutput output = upstream_slot_output;
  results.emplace("root", output);
  
  if (approved_module_ptrs_.empty()) {
    return SlotOutput{output, ...};
  }
  
  // Line 808-810: Loop through ALL approved modules
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), 
    [&](const auto & m) {
      // Line 809: Call run() for each approved module
      output = run(m, data, output);
      //         ↑
      //         └─→ This calls SubPlannerManager::run() → module->run() → plan()
      results.emplace(m->name(), output);
    });
  
  // ... handle waiting approval, failures, success ...
  
  return SlotOutput{approved_modules_output, ...};
}
```

**Purpose**: Execute all modules that are already approved and running. These modules have priority and their output feeds into the next modules.

---

### Level 3B: Candidate Modules Execution - SubPlannerManager::runRequestModules()

**File**: `planner_manager.cpp`  
**Line**: 628

```cpp
std::pair<SceneModulePtr, BehaviorModuleOutput> SubPlannerManager::runRequestModules(
  const std::vector<SceneModulePtr> & request_modules, 
  const std::shared_ptr<PlannerData> & data,
  const BehaviorModuleOutput & previous_module_output)
{
  std::vector<SceneModulePtr> executable_modules;
  std::vector<SceneModulePtr> waiting_approved_modules;
  std::vector<SceneModulePtr> already_approved_modules;
  std::unordered_map<std::string, BehaviorModuleOutput> results;
  
  // Line 647-648: Sort by priority
  auto sorted_request_modules = request_modules;
  sortByPriority(sorted_request_modules);
  
  // Line 653-672: Filter executable modules (handle simultaneous execution)
  for (const auto & module_ptr : sorted_request_modules) {
    // Check if module can execute simultaneously with others
    if (can_execute_simultaneously) {
      executable_modules.push_back(module_ptr);
    }
  }
  
  // Line 677-685: Execute all executable candidate modules
  for (const auto & module_ptr : executable_modules) {
    const auto & manager_ptr = getManager(module_ptr);
    
    if (!manager_ptr->exist(module_ptr)) {
      manager_ptr->registerNewModule(
        std::weak_ptr<SceneModuleInterface>(module_ptr), previous_module_output);
    }
    
    // Line 685: Call run() for each candidate module
    results.emplace(module_ptr->name(), run(module_ptr, data, previous_module_output));
    //                                            ↑
    //                                            └─→ This calls SubPlannerManager::run() → module->run() → plan()
  }
  
  // ... separate by approval status, select highest priority ...
  
  return std::make_pair(highest_priority_module, results.at(highest_priority_module->name()));
}
```

**Purpose**: Execute candidate modules that want to run. These are evaluated, and the highest priority one may become approved.

---

### Level 4: Module Execution Wrapper - SubPlannerManager::run()

**File**: `planner_manager.cpp`  
**Line**: 759

```cpp
BehaviorModuleOutput SubPlannerManager::run(
  const SceneModulePtr & module_ptr, 
  const std::shared_ptr<PlannerData> & planner_data,
  const BehaviorModuleOutput & previous_module_output) const
{
  StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic(module_ptr->name());
  
  // Line 766-767: Setup module with data
  module_ptr->setData(planner_data);
  module_ptr->setPreviousModuleOutput(previous_module_output);
  
  // Line 769-771: THE KEY CALL - This invokes module->run()
  module_ptr->lockRTCCommand();
  const auto result = module_ptr->run();  // ← Calls SceneModuleInterface::run()
  module_ptr->unlockRTCCommand();
  
  // Line 773-777: Post-processing
  module_ptr->postProcess();
  module_ptr->updateCurrentState();
  module_ptr->publishObjectsOfInterestMarker();
  
  processing_time_.at(module_ptr->name()) += stop_watch.toc(module_ptr->name(), true);
  return result;
}
```

**Purpose**: Wrapper that sets up the module and calls its `run()` method. Also handles timing and post-processing.

---

### Level 5: Base Class Run - SceneModuleInterface::run()

**File**: `scene_module_interface.hpp`  
**Line**: 140

```cpp
virtual BehaviorModuleOutput run()
{
  updateData();  // ← Line 142: Calls YOUR updateData() method
  const auto output = isWaitingApproval() ? planWaitingApproval() : plan();
  //                                          ↑ Line 143: Calls YOUR plan() method
  try {
    autoware::motion_utils::validateNonEmpty(output.path.points);
  } catch (const std::exception & ex) {
    throw std::invalid_argument("[" + name_ + "]" + ex.what());
  }
  return output;
}
```

**Purpose**: Base class implementation that:
1. Calls `updateData()` to refresh module data
2. Calls either `planWaitingApproval()` or `plan()` based on module status
3. Validates the output

**Note**: This is where `updateData()` and `plan()` are actually invoked!

---

## Code Locations Summary

| Level | File | Line | Function | Purpose |
|-------|------|------|----------|---------|
| 1 | `planner_manager.cpp` | 103 | `PlannerManager::run()` | Main entry point, processes all slots |
| 2 | `planner_manager.cpp` | 912 | `SubPlannerManager::propagateFull()` | Iterative loop, orchestrates module execution |
| 3A | `planner_manager.cpp` | 783 | `SubPlannerManager::runApprovedModules()` | Executes approved modules |
| 3B | `planner_manager.cpp` | 628 | `SubPlannerManager::runRequestModules()` | Executes candidate modules |
| 4 | `planner_manager.cpp` | 759 | `SubPlannerManager::run()` | Wrapper that calls `module->run()` |
| 5 | `scene_module_interface.hpp` | 140 | `SceneModuleInterface::run()` | Base class that calls `updateData()` and `plan()` |
| 6 | `scene.cpp` | 118 | `DirectionChangeModule::updateData()` | **YOUR CODE** - Lightweight data update |
| 6 | `scene.cpp` | 200 | `DirectionChangeModule::plan()` | **YOUR CODE** - Expensive planning work |

---

## Example Execution Trace

### Scenario: Direction Change Module as New Candidate

```
Time: Planning Cycle Start
─────────────────────────────────────────────────────────────

[1] PlannerManager::run() [line 103]
    └─→ Slot 1: propagateFull()
          │
          ├─→ ITERATION 1
          │     │
          │     ├─→ runApprovedModules() [line 925]
          │     │     └─→ (No approved modules yet)
          │     │
          │     └─→ runRequestModules() [line 944]
          │           │
          │           └─→ direction_change module detected
          │                 │
          │                 └─→ run(direction_change_module) [line 685]
          │                       │
          │                       └─→ direction_change_module->run() [line 770]
          │                             │
          │                             └─→ SceneModuleInterface::run() [line 140]
          │                                   │
          │                                   ├─→ updateData() [line 142] ✅
          │                                   │   └─→ DirectionChangeModule::updateData()
          │                                   │       └─→ reference_path_ = previous_output.path
          │                                   │
          │                                   └─→ plan() [line 143] ✅ CALL #1
          │                                       └─→ DirectionChangeModule::plan()
          │                                           └─→ cusp_point_indices_ = findCuspPoints()
          │                                           └─→ reverseOrientationAtCusps()
          │
          ├─→ Module approved, added to approved list
          │
          └─→ ITERATION 2
                │
                ├─→ runApprovedModules() [line 925]
                │     │
                │     └─→ direction_change module (now approved)
                │           │
                │           └─→ run(direction_change_module) [line 809]
                │                 │
                │                 └─→ direction_change_module->run() [line 770]
                │                       │
                │                       └─→ SceneModuleInterface::run() [line 140]
                │                             │
                │                             ├─→ updateData() [line 142] ✅
                │                             │
                │                             └─→ plan() [line 143] ✅ CALL #2
                │
                └─→ No more request modules, exit loop

─────────────────────────────────────────────────────────────
Time: Planning Cycle End
```

### Why Two Calls in This Example?

1. **First Call (Iteration 1, Candidate)**: Module is detected as a candidate and executed to evaluate if it should be approved
2. **Second Call (Iteration 2, Approved)**: Module is now approved and executed again as part of the approved module chain

---

## Implications for Module Development

### 1. **`updateData()` Should Be Lightweight**

Since `updateData()` is called every time `run()` is called (which happens multiple times), it should:
- ✅ Only update reference data (e.g., `reference_path_ = previous_output.path`)
- ❌ NOT do expensive computations (e.g., cusp detection, path processing)

**Correct Pattern**:
```cpp
void DirectionChangeModule::updateData()
{
  const auto previous_output = getPreviousModuleOutput();
  reference_path_ = previous_output.path;  // Lightweight copy only
  // No expensive computations here!
}
```

### 2. **`plan()` Does the Expensive Work**

Since `plan()` is called when actually planning, it should:
- ✅ Do expensive computations (cusp detection, path modification)
- ✅ Use cached data from `updateData()`

**Correct Pattern**:
```cpp
BehaviorModuleOutput DirectionChangeModule::plan()
{
  // Expensive work happens here
  cusp_point_indices_ = findCuspPoints();  // Compute when needed
  // ... process path ...
  return output;
}
```

### 3. **Multiple Calls Are Expected**

- `plan()` will be called **multiple times per planning cycle**
- This is **normal behavior** due to the iterative refinement architecture
- Modules should be designed to handle this efficiently

### 4. **Performance Considerations**

If your module has expensive computations:
- **Option A**: Accept the overhead (standard pattern)
- **Option B**: Cache results and only recompute when inputs change
- **Option C**: Use state machines to avoid unnecessary recomputation

---

## Debugging Tips

### Add Debug Output to Track Calls

```cpp
void DirectionChangeModule::plan()
{
  static int call_count = 0;
  call_count++;
  std::cout << "[MY_DEBUG] plan() call #" << call_count << std::endl;
  // ... rest of plan() ...
}
```

### Check Module Status

```cpp
void DirectionChangeModule::plan()
{
  std::cout << "[MY_DEBUG] Module status: " 
            << magic_enum::enum_name(getCurrentStatus()) << std::endl;
  // ... rest of plan() ...
}
```

### Verify Path Changes

```cpp
void DirectionChangeModule::updateData()
{
  static size_t last_path_size = 0;
  reference_path_ = previous_output.path;
  
  if (reference_path_.points.size() != last_path_size) {
    std::cout << "[MY_DEBUG] Path changed: " << last_path_size 
              << " -> " << reference_path_.points.size() << std::endl;
    last_path_size = reference_path_.points.size();
  }
}
```

---

## Summary

- **Multiple `plan()` calls are normal** - The PlannerManager uses iterative refinement
- **Two main call paths**: Approved modules and candidate modules
- **`updateData()` = lightweight**, **`plan()` = expensive work**
- **This architecture allows**: Module priority handling, simultaneous execution, iterative refinement

Understanding this call chain helps with:
- Debugging module behavior
- Optimizing performance
- Following Autoware BPP patterns correctly

