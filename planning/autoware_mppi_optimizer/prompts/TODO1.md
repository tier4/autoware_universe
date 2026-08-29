The texture coordinate math, surface writes, stream ordering, and out-of-bounds fallback look correct. I found these remaining issues.

1. High — drivable-area distance is not actually signed

The static kernel stores unsigned distance to boundary segments ([generation](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu:228)). Subtracting the ego radius only makes it negative while crossing the boundary ([sampling](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu:2027)).

Once the ego is completely outside, the distance becomes positive again and the barrier disappears. The trajectory validator also does not check the drivable-area polygon, and the checked-in configuration sets its barrier weight to zero.

Fix: generate a signed field from a closed polygon—positive inside, negative outside—and restore a hard polygon/footprint validation check.

1. High — 3D generation has excessive computational complexity

The obstacle kernel evaluates every obstacle for every voxel ([kernel](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu:244)):

- `512 × 512 × 80 = 20,971,520` voxels per cycle
- With 64 obstacles: approximately 1.34 billion oriented-box distance evaluations
- Static worst case: `1024 × 1024 × 512 ≈ 537 million` segment evaluations

This is unlikely to meet a 100 Hz control budget. Static obstacles are also recomputed independently for all 80 slices.

Fix: rasterize primitives and run a GPU EDT/JFA, split static and dynamic obstacles, or compute only a bounded distance band. This needs profiling before relying on the estimated 1 ms generation time.

1. High — debug visualization still has a lifetime race

The visualizer is a raw pointer ([member](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh:388)). Enable/disable can delete it while the control thread is inside `render()` ([lifecycle](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu:867)).

Context unbinding solves sequential context ownership, but not concurrent deletion/use-after-free.

Fix: protect creation, rendering, and destruction with one mutex and use `std::unique_ptr`; ideally keep all GLFW/OpenGL operations on a dedicated visualization thread.

1. Medium — configured `overlimit_coeff` is ignored

`make_cost_params()` copies `steer_rate_coeff` and then skips directly to noise parameters ([parameter conversion](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/src/trajectory_mppi_optimizer.cpp:75)). Therefore the configured value `100.0` is replaced by the struct default `10000.0`.

Fix:

```cpp
output.overlimit_coeff = static_cast<float>(params.overlimit_coeff);
```

1. DONE — host and GPU evaluate different objectives

GPU texture queries approximate the ego footprint with its circumscribed circle, while host replay uses exact oriented-box geometry. This can be substantially more conservative beside a long vehicle and makes reported cost breakdowns differ from the objective MPPI optimized.

Fix:
A. The GPU Optimizer (Fast, Close Approximation)
Do not use a single massive bounding circle. Upgrade the GPU to use the Multi-Circle Spine approximation discussed earlier. By using 3 or 4 overlapping circles, the GPU's cost landscape will very closely match the exact rectangle, allowing it to easily find paths through narrow corridors and parallel parking spots.

    B. The Host Cost Breakdown (Must match GPU)
    The functions on the host that report the cost scores (computeRunningCostBreakdown and computeTerminalCostBreakdown) must use the exact same math as the GPU (e.g., the multi-circle ESDF lookup, or multi-circle math).

    C. The Final Trajectory Validator (Exact OBB)
    Once the optimizer outputs its single best trajectory, pass it to a separate, dedicated "Hard Constraint Validator" component.

6. Medium — existing test contradicts the new dynamic-obstacle behavior

The implementation and header now include moving objects in the gradual 3D obstacle cost, but [the test](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/test/test_trajectory_validator.cu:328) still expects their obstacle cost to be zero. That test should now fail.

1. Medium — incorrect input stride for long obstacle trajectories

`num_timesteps` is clamped to the 80-step horizon and the clamped value is then used as the obstacle-major source stride ([trajectory setter](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu:1425)). Inputs containing more than 80 steps are indexed incorrectly for every obstacle after the first.

1. Lower-priority API bugs

- `setDrivableAreaPolygon()` and `clearDrivableArea()` are declared but have no definitions ([declarations](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh:194)).
- The path-reference bridge dereferences `ref[0]` even when `ref` is empty ([bridge](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost_bridge.hpp:103)).
- When reference yaw is omitted, the first point receives the final segment’s yaw rather than the first segment’s yaw ([reference setter](/home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/universe/planning/autoware_mppi_optimizer/include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu:1303)).

I made no changes and did not build or run tests.
