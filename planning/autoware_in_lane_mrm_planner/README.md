# autoware_in_lane_mrm_planner

MRM in-lane stop trajectory planner (Phase1).

## Design

See `docs/design_phase1.md` and `docs/implementation_plan_phase1.md`.

## Build

From `pilot-auto.x2.v4.3.2` repository root (after workspace dependencies are installed):

```bash
./cmd_helper.sh --build_ccache --packages-select autoware_in_lane_mrm_planner
```

Do not run package builds in parallel with a full workspace build.

## Launch

```bash
ros2 launch autoware_in_lane_mrm_planner in_lane_mrm_planner.launch.xml
```

## Dependencies

- Path / obstacle-stop logic: vendored from `pilot-auto.x2.v4.3.e2e` (see `docs/*_sync.md`)
- EB smoothing: `autoware_path_smoother` (v4.3.2 workspace)
