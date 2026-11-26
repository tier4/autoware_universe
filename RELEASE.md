# Release 2026-11-27: four jpntaxi models, one build (pilot-auto.xx1 v0.53)

Hand-over for the agent deploying on the Japan taxi. One build of the two branches below runs all four 2026-11-27
models; switching models is one launch argument, `e2e_model`, with no rebuild.

The same file is on both branches:

| Repository | Branch | Code at (only this RELEASE.md follows) | Base |
|---|---|---|---|
| tier4/autoware_universe | `e2e-release/1127/jpntaxi-unified` | `6cb4cec182` | `e2e-release/1127/resworld-1.4.4.4` (the ResWorld node unchanged) |
| tier4/autoware_launch.xx1 | `e2e-release/1127/jpntaxi-unified` | `fa209c9051` | `dev/v0.53/e2e-resworld` at `fbff3706` (#2206-#2209 included) |

Every other repository stays at the car's stack, pilot-auto.xx1 `beta/v0.53` (`autoware.repos`; tested at `7f39680f`).
No OTA, no other version change.

## The four models

All four are WebAuto package releases in project `prd_jt`, ML package `OnePlanner`.

| `e2e_model` | Model | WebAuto package release (id) | Files in the directory |
|---|---|---|---|
| `OnePlanner` (default) | ResWorld / OnePlanner, whichever version is in `<data_path>/OnePlanner` (now 1.4.4.4) | `20261125_resworld__oneplanner-1.4.4.4-jpntaxi` (`5db45bcf`) | `resworld_planner.simplified.onnx`, `bevfusion_lidar_feature.onnx`, `ml_package_resworld.param.yaml`, `deployment_manifest.json` |
| `oneplanner-mlplanner-1p5stage-b-jpntaxi` | ML-Planner 1.5-stage B | `20261125_mlplanner__oneplanner-mlplanner-1p5stage-b-jpntaxi` (`33ed57d8`) | `ml_planner_bev.onnx`, `metadata.json`, `bevfusion_lidar_feature.onnx` |
| `oneplanner-mlplanner-2stage-a-jpntaxi` | ML-Planner 2-stage A | `20261125_mlplanner__oneplanner-mlplanner-2stage-a-jpntaxi` (`6c701579`) | `ml_planner_sampler.onnx`, `metadata.json` |
| `oneplanner-mlplanner-2stage-b-jpntaxi` | ML-Planner 2-stage B | `20261125_mlplanner__oneplanner-mlplanner-2stage-b-jpntaxi` (`87efa0a9`) | `ml_planner_sampler.onnx`, `metadata.json` |

sha256 of the files (check them after copying):

| File | sha256 |
|---|---|
| OnePlanner 1.4.4.4 `resworld_planner.simplified.onnx` | `4b5e9f4f64c5aa5b156ae10c31ca0d75169ac6e033e15e7d5dbff1703d8dbb92` |
| OnePlanner 1.4.4.4 `bevfusion_lidar_feature.onnx` (the same file in 1.5-stage B) | `d701f6c37b44af6a04deab43535c0ff7ed81142def56126bd0df4690f4e57e8e` |
| OnePlanner 1.4.4.4 `ml_package_resworld.param.yaml` | `df55e869bed3ea3387653a3806cd6a0e56f6b78f5e9e5b39c20cd4d6c0b87432` |
| OnePlanner 1.4.4.4 `deployment_manifest.json` | `50054e7014f9e3a425e51615de788b893771652b4e41df52ac19429c2fb5f4bf` |
| 1.5-stage B `ml_planner_bev.onnx` | `aba79716c35603f96e430da558920661bf759016ea5cff4ec7e55c20fac17a37` |
| 1.5-stage B `metadata.json` | `4bf246ba529ffd149c43de15ae1bad4692219ead5424c6737535008ccbc258c3` |
| 2-stage A `ml_planner_sampler.onnx` | `d42dbdf0195cddfe5329abcc5c4fe28e7e4f47d108f2975bf54335c8ac2892ee` |
| 2-stage A `metadata.json` | `202f013acf8f96191ca8c5cd82a3e7cf6ccb55300105d9234607a3af3822a1aa` |
| 2-stage B `ml_planner_sampler.onnx` | `8b33591904a1d6a5958ff7cfa1cd434d900af291dda4ee3b58aa3f28b56dbe86` |
| 2-stage B `metadata.json` | `b5e95a6bed8353ae8fd6add1bc92c35b23d316b099b05498fba642f517456016` |

Per-model details (training run, offline metrics, how the graphs were exported) are in each model's own release
branch, `e2e-release/1127/<model>`, in this repository and in tier4/OnePlanner.

## Step 1: build once

1. Take the car's workspace (pilot-auto.xx1 `beta/v0.53`). Point two entries at the branches above: `universe` to
   tier4/autoware_universe and `launcher` to tier4/autoware_launch.xx1, both `e2e-release/1127/jpntaxi-unified`.
   Leave every other entry alone.
2. Build as usual (Release). New packages that get built: `autoware_ml_planner`, plus the 1.5-stage node inside
   `autoware_tensorrt_e2e`.
3. **Check that acados is found for ML-Planner.** `autoware_ml_planner` generates its trajectory-optimizer solver at
   build time from `$ACADOS_SOURCE_DIR` (default `/opt/acados`). It needs `lib/libacados.so`, `lib/libblasfeo.so`,
   `lib/libhpipm.so` and `.venv/bin/python3` with `interfaces/acados_template` there. If one is missing, the build
   still **succeeds**, prints `acados was not found under ...; trajectory optimization support will be disabled`, and
   the ML-Planner models then run **without** their internal optimizer. Search the build log for that line. If it is
   there, fix `ACADOS_SOURCE_DIR` and rebuild `autoware_ml_planner` and `autoware_tensorrt_e2e`. The v0.53 chain's
   own `autoware_trajectory_optimizer` already needs acados (`find_package(acados REQUIRED)`), so the libraries are
   on the car; the Python template is what to check.
4. If an ML-Planner node fails at start with `libacados.so: cannot open shared object file`, add the acados `lib`
   directory to `LD_LIBRARY_PATH` for the Autoware process.

## Step 2: put the models in place

Under the launch's `data_path` (autoware.launch.xml defaults to `$HOME/autoware_data`; use whatever the car passes):

```
<data_path>/OnePlanner/                                  ResWorld, as today (replace its four files to change version)
<data_path>/oneplanner-mlplanner-1p5stage-b-jpntaxi/     1.5-stage B
<data_path>/oneplanner-mlplanner-2stage-a-jpntaxi/       2-stage A
<data_path>/oneplanner-mlplanner-2stage-b-jpntaxi/       2-stage B
```

Copy the package releases as they are; their file names are what the nodes read.

- **`<data_path>/OnePlanner` must exist even when testing a 2-stage model.** The 2-stage models' detections come from
  a ResWorld node that reads the ResWorld model there (see "What runs").
- **Keep every model directory writable.** The first start of each model builds its TensorRT engines beside the ONNX
  files (`*.engine`, `*.trt.engine` and an `.identity` file) and takes a few minutes. Later starts load them in
  seconds. An engine whose graph, precision or bytes changed is rebuilt by itself.

## Step 3: launch, switch with `e2e_model`

Same command as the current ResWorld test (`planning_setting:=resworld`), plus `e2e_model`:

```
ros2 launch autoware_launch autoware.launch.xml ... planning_setting:=resworld e2e_model:=<directory name>
```

Leaving `e2e_model` out is the current ResWorld behaviour exactly. To change model, change `e2e_model`; to change the
ResWorld version, replace the files in `<data_path>/OnePlanner`. Nothing else, and no rebuild.

## What runs

Perception's own detection stage, including the original BEVFusion node, is **never** launched under
`planning_setting:=resworld`, for any `e2e_model`. `launch_detection` defaults to `auto`, which is false for
resworld. That cuts the only include path to `autoware_bevfusion`:
`perception.launch.xml` `<group if="$(var launch_detection)">` → `detection.launch.xml` →
`lidar_dnn_detector.launch.xml`. Tracking, prediction and traffic lights run as before. The detections come from the
OnePlanner BEV backbone and its detection head in the E2E node, published to
`/perception/object_recognition/detection/bevfusion/objects`, the topic the tracker reads.

BEVFusion starts only if someone passes `launch_detection:=true` explicitly or uses another `planning_setting`.

All nodes are under `/planning/trajectory_generator/neural_network_based_planner/`:

| `e2e_model` | Drives | Detections from | Nodes |
|---|---|---|---|
| `OnePlanner` | ResWorld | the ResWorld node | `tensorrt_e2e` |
| `...-1p5stage-...` | 1.5-stage (BEVFusion-L extractor and detection head in its own node) | that node | `ml_planner_node` (`autoware_tensorrt_e2e` `MLPlannerBev`) |
| `...-2stage-...` | 2-stage (`autoware_ml_planner`, on the tracked objects) | a ResWorld node beside it, reading `<data_path>/OnePlanner` | `ml_planner_node` + `tensorrt_e2e` |

Every node with a BEV extractor (`tensorrt_e2e`, and `ml_planner_node` for the 1.5-stage) is composed into the
pointcloud container, the only place the CUDA concatenator's zero-copy cloud reaches, as ResWorld is today.

The drive path is the same chain for every model, unchanged from `dev/v0.53/e2e-resworld` (`fbff3706`):

1. the model's candidates on `/planning/generator/resworld/candidate_trajectories`;
2. `trajectory_modifier`, then `trajectory_optimizer`, with the same parameter files, not one value changed;
3. the result on `/planning/trajectory_generator/trajectory`.

Every model drives `/planning/turn_indicators_cmd`. In the 2-stage setting, the ResWorld node's own trajectory and
turn indicator are sidelined to `/planning/generator/resworld/detection_only/...` and drive nothing.

The ML-Planner models also run their own internal optimizer, as on X2. Its parameters are
`autoware_launch/config/planning/neural_net_planner/ml_planner.param.yaml`: the jpntaxi ML-Planner tuning of
pilot-auto.xx1 v0.54.0, in weight major version 7's format. The parameters version 7 adds are at the package defaults.

## Check on the car after each start

1. `ros2 node list | grep -i bevfusion`: empty.
2. `ros2 node list | grep neural_network_based_planner`: the nodes of the table above, plus `trajectory_modifier`
   and `trajectory_optimizer_node`.
3. Rates, each about 10 Hz:
   - `/planning/generator/resworld/candidate_trajectories`
   - `/planning/trajectory_generator/trajectory`
   - `/perception/object_recognition/detection/bevfusion/objects`
   - `/planning/turn_indicators_cmd`
4. The candidates' generator name: `TensorrtE2eResworld_batch_0` for ResWorld, `MLPlanner_batch_0` for ML-Planner.
5. ResWorld logs `BEV feature extractor detection head enabled` once its extractor engine is up. The ML-Planner nodes
   log `Loaded model_path=...`. ResWorld waits for BEV history after a start or a localization jump; repeated
   `Waiting for the first input from 'bev_feature'` while clouds flow means the cloud is not reaching the node: check
   the container composition and `/sensing/lidar/concatenated/pointcloud`.

## How it was verified (2026-11-26)

- **Build.** The pilot-auto.xx1 `beta/v0.53` repositories with these two branches built: every planner and
  post-processing package, on CUDA 13 / TensorRT 10.13 with sm_90 added for the test GPUs. The car is
  CUDA 12.8 / TensorRT 10.8, and the packages' own architecture lists cover its GPU.
- **Replay.** Golden-val taxi scene `prd_jt/0ed0b1e4/2025-11-19/11_08_34` (60 s, map 1423) on sim time.
  - Node level: the planner alone.
  - Launcher level: this branch's `resworld.launch.xml` with the car's arguments and configuration, through the
    modifier and optimizer.

| `e2e_model` | Node level: FDE 2 s / 4 s (m), rate | Launcher level: candidates → modified → final, detections |
|---|---|---|
| OnePlanner (1.4.4.4) | 0.213 / 0.560, 8.7 Hz (offline on the same scene: 0.202 / 0.512) | 523 → 523 → 523, 534 detection frames |
| 1.5-stage B | 0.180 / 0.510, 9.2 Hz | 543 → 452 → 429, 543 detection frames |
| 2-stage B | 0.179 / 0.471, 10.0 Hz | 597 → 591 → 591, 496 detection frames from the ResWorld node |
| 2-stage A | 0.210 / 0.591, 10.0 Hz | 597 → 351 → 336, 474 detection frames from the ResWorld node |

No BEVFusion node ran in any of them.

## Known points

- **One replay run of OnePlanner received no cloud at all.** That was the first of three; the other two were normal.
  A replay feeds the cloud from a bag through the node's standalone fallback subscription. On the car the node sits
  in the pointcloud container on the zero-copy path that ran on 2026-11-13. If it ever happens on the car, see item 5
  of the checks.
- **The replay ran the 2-stage A chain at 5.6 Hz.** One replay job had 16 CPUs for two planner nodes, the modifier and
  the optimizer; the other models stayed near 10 Hz. Check its rate on the car.
- **One source change outside the new code.** `autoware_ml_planner` is feat/v0.64/e2e's `ef8d906c16` except for one
  compile-time guard. v0.53 pins `autoware_internal_msgs` `1856e5b`, whose `CandidateTrajectory` has no
  `turn_indicators_command`, so the field is set only where it exists. The turn indicator still goes out on its own
  topic, which is what this stack reads.
- **Rollback.** Leave `e2e_model` out (or set it to `OnePlanner`): that is today's ResWorld test, with the same node
  code, chain and parameters.
