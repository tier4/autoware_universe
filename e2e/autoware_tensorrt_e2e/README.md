# autoware_tensorrt_e2e

An **abstract E2E trajectory planner node**: one node that runs end-to-end planning models with
different sensing inputs (front camera / surround cameras / concatenated LiDAR) and an
adjustable, fully optional set of diffusion-planner-style context inputs. The node adapts to
the model by introspecting the TensorRT engine's IO manifest — switching models is a config
change, not a code change.

See [docs/design.md](docs/design.md) for the architecture and design rationale.

## Supported models

| Model | Launch file | Sensing input |
| ----------------------- | ----------------------------------------- | ------------------------------------------------------------ |
| **ResWorld** (OnePlanner) | `e2e_planner_resworld.launch.xml` | Concatenated point cloud → frozen BEVFusion-L features, 3-frame temporal history |

```bash
ros2 launch autoware_tensorrt_e2e e2e_planner_resworld.launch.xml
```

`$(data_path)/$(model_name)` — by default
`$HOME/autoware_data/ml_models/tensorrt_e2e/resworld` — holds
`resworld_planner.simplified.onnx`, `resworld_deployment_contract.json` (the cache/tensor
contract, read by the node), the frozen BEVFusion-L feature extractor ONNX
(`bevfusion_lidar_feature.onnx`: the production BEVFusion lidar branch exported with
`bev_feature` `[1, 512, 180, 180]` as an output), and `ml_package_resworld.param.yaml`
describing all of it (see [Configuration layout](#configuration-layout); the copy under
`config/` is a reference, the node reads the one beside the artifacts).

The extractor ONNX carries `autoware::GetIndicePairsImplicitGemm` / `autoware::ImplicitGemm`
custom nodes (sparse convolutions), so building and running its engine requires
`autoware_tensorrt_plugins` (`bev_feature.extractor.plugins_path`) and an `spconv_cpp`
build that includes this GPU's architecture. Its 32 points per voxel and 5 point features
(`use_intensity: true`) match the frozen BEVFusion-L training voxelizer and are baked into
the graph — which is why the ml_package generator reads them back out of the graph instead
of trusting a hand-written value.

The planner runs in **fp32**: its graph embeds normalization statistics that overflow fp16
(TensorRT clips them silently), and TensorRT 10.16's fp16 builder path aborts on this graph
outright. `ml_package_resworld.param.yaml` records that, overriding the package default.

## Sensor prototypes

| Prototype | Launch file | Sensing input |
| --------------- | -------------------------------------- | ------------------------------------------------------- |
| Front camera | `e2e_planner_front_camera.launch.xml` | `CAM_FRONT_WIDE` |
| Surround camera | `e2e_planner_surround_cameras.launch.xml` | Front wide + 4 corner wide cameras |
| LiDAR (raw points) | `e2e_planner_lidar.launch.xml` | `/sensing/lidar/concatenated/pointcloud` (as BEVFusion) |
| LiDAR (BEV features) | `e2e_planner_resworld.launch.xml` | Concatenated point cloud → temporal BEV feature history |

Add `build_only:=true` to any launch to build the TensorRT engine(s) and exit.

## Model contract

The node accepts any single ONNX/TensorRT engine whose inputs are covered by the enabled input
providers (matched **by tensor name**, validated **by tensor shape** at startup) and whose
primary output is an ego trajectory tensor:

- Output `prediction`: `[B, A, T, 4]` (agent 0 = ego, others = neighbor predictions) or
  `[B, T, 4]` (ego only), with `(x, y, cos(yaw), sin(yaw))` per 0.1 s step in the ego frame.
  The current models use `T = 40` (4 s horizon); the horizon is validated against
  `postprocess.horizon_seconds`. The tensor name is configurable
  (`postprocess.prediction_tensor`, e.g. ResWorld's `trajectory`), and additional ego-only
  trajectory outputs (`postprocess.extra_trajectory_tensors`, e.g. ResWorld's
  `prior_trajectory`) are published as extra candidate trajectories.

### Claimable input tensors

| Provider | Tensor (default name) | Shape | Notes |
| -------- | --------------------------------- | ---------------- | ------------------------------------------ |
| camera | `camera_images` | `[1, N, 3, H, W]` | Normalized RGB; `H`, `W` from the engine |
| camera | `camera_intrinsics` | `[1, N, 3, 3]` | Rescaled to the model resolution |
| camera | `camera2ego` | `[1, N, 4, 4]` | From TF (`camera frame -> base_link`) |
| lidar | `points` | `[1, P, D]` | `D` in 3–5; padded/truncated to `P` |
| lidar | `num_points` | `[1, 1]` | Valid point count |
| bev_feature | `bev_feature_history` | `[1, K, C, H, W]` | Temporal BEVFusion-L features, current-to-past, SE(2)-warped |
| context | `ego_current_state` | `[1, 10]` | Same feature pipeline as the |
| context | `ego_agent_past` | `[1, T, 4]` | diffusion planner (its library |
| context | `neighbor_agents_past` | `[1, N, 31, 11]` | is reused, so the features are |
| context | `static_objects` | any | bit-identical) |
| context | `lanes(_speed_limit)(_has_speed_limit)` | `[1, S, 20, D]` / `[1, S, 1]` | |
| context | `route_lanes(_speed_limit)(_has_speed_limit)` | `[1, S, 20, D]` / `[1, S, 1]` | |
| context | `polygons`, `line_strings` | fixed | |
| context | `goal_pose`, `ego_shape`, `turn_indicators` | `[1, 4]` / `[1, 3]` / `[1, T]` | |

Context tensors are all optional: only the ones present in the engine manifest are produced,
and only the subscriptions they need are created. An engine input no provider can produce is a
**startup error** that names the missing tensor.

## Interface

### Inputs (subscribed on demand)

| Topic | Type | Used by |
| ------------------------------ | ------------------------------------------------- | ------------------ |
| `~/input/odometry` | `nav_msgs/msg/Odometry` | always |
| `~/input/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | `ego_current_state` |
| `~/input/camera{i}/image` | `sensor_msgs/msg/Image` (raw or compressed) | camera provider |
| `~/input/camera{i}/camera_info`| `sensor_msgs/msg/CameraInfo` | `camera_intrinsics` |
| `~/input/pointcloud` | `sensor_msgs/msg/PointCloud2` | lidar provider |
| `~/input/tracked_objects` | `autoware_perception_msgs/msg/TrackedObjects` | `neighbor_agents_past` |
| `~/input/traffic_signals` | `autoware_perception_msgs/msg/TrafficLightGroupArray` | `lanes`, `route_lanes` |
| `~/input/route` | `autoware_planning_msgs/msg/LaneletRoute` | `route_lanes`, `goal_pose` |
| `~/input/vector_map` | `autoware_map_msgs/msg/LaneletMapBin` | map tensors |
| `~/input/turn_indicators` | `autoware_vehicle_msgs/msg/TurnIndicatorsReport` | `turn_indicators` |

### Outputs

| Topic | Type | Description |
| ----------------------------- | ---------------------------------------------------- | ----------------------------------- |
| `~/output/trajectory` | `autoware_planning_msgs/msg/Trajectory` | Ego trajectory (40 pts / 4 s) |
| `~/output/trajectories` | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | One candidate per batch |
| `~/output/predicted_objects` | `autoware_perception_msgs/msg/PredictedObjects` | Only for multi-agent models |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs/msg/Float64Stamped` | Per-tick processing time |
| `/diagnostics` | — | `inference_status` (readiness, input drops, budget overruns) |

## 10 Hz operation

The node is timer-driven at `planning_frequency_hz` (default 10 Hz); sensor callbacks only
latch messages. Each tick's processing time is published, and exceeding the planning period
raises a `WARN` diagnostic (`Processing time exceeded the planning period`), making rate
violations observable. Model + preprocessing must fit the 100 ms budget on target hardware.

## Visualization

Every launch file takes `rviz:=true`, which opens `rviz/e2e_planner.rviz` — grid, vehicle
model, lanelet2 map, the published trajectory, and the input pointcloud (off by default: it
is the expensive display).

```bash
ros2 launch autoware_tensorrt_e2e <launch file> rviz:=true use_sim_time:=true
```

That also starts `vector_map_marker_relay.py`, which is not optional if you want to see the
map. Two things stop RViz from drawing it, neither of which reports an error:

- `autoware_lanelet2_map_visualizer` emits duplicate `(ns, id)` marker keys — thousands of
  them on a city map — and RViz rejects the whole array on its duplicate check;
- a looping `ros2 bag play --clock` moves simulated time backwards once per cycle, RViz
  resets and discards everything it holds, and a latched message is delivered once per
  subscription and never again.

The relay renumbers the ids and re-latches on a slow heartbeat, so the view heals itself at
every loop point. `use_sim_time:=true` is required whenever inputs come from a bag: without
it the node measures input staleness against wall time and drops every message.

## Configuration layout

Parameters are split the way `autoware_bevfusion` splits them, so that switching
models never edits this package:

| File | Lives in | Holds |
| ---- | -------- | ----- |
| `config/e2e_planner.param.yaml` | the package | deployment defaults, **model-agnostic**: artifact paths (from launch arguments), TensorRT workspace, planning rate, staleness tolerances, context and postprocess behaviour |
| `ml_package_<model>.param.yaml` | **the model directory, beside the ONNX files** | the network itself: tensor names, voxelization geometry, temporal contract, horizon, and the precision the graph is validated in |

The launch file loads the package defaults first and the ml_package second, so a
model's values override the defaults. Deploying another model or a re-trained
variant is a launch argument, not an edit:

```bash
ros2 launch autoware_tensorrt_e2e e2e_planner_resworld.launch.xml \
  data_path:=$HOME/autoware_data/ml_models/tensorrt_e2e model_name:=resworld
```

`ml_package_<model>.param.yaml` is **generated from the artifacts**, never
hand-written, so a new checkpoint cannot silently disagree with a stale config:

```bash
python3 scripts/make_ml_package_param.py <model_dir> --model-name <model>
```

It reads points-per-voxel and the point-feature count out of the extractor's
`voxels` input shape, the cache semantics out of the deployment contract JSON,
and the tensor names and horizon out of the planner graph. Re-run it whenever
the artifacts in that directory change.

### TensorRT workspace

`trt_workspace_mib` (default 4096) is a property of the deployment host, not of
the model. Sizing it below what a graph needs does not merely cost performance:
the builder segfaults (1 GiB is not enough for the ResWorld planner).

## Extending

- **New model variant (same modality)**: point `model_name`/`model_path` at the
  new artifacts and regenerate their ml_package file; no code change.
- **New sensing modality**: implement `InputProviderInterface` (`claim_inputs` + `collect`),
  register it in `TensorrtE2eNode::create_providers()`, and add its name to `sensor_inputs`.
- **New context feature**: extend `ContextInputProvider` with the new tensor claim.

## Related packages

- [`autoware_diffusion_planner`](../../planning/autoware_diffusion_planner/README.md) — context
  feature pipeline and trajectory postprocessing are reused from its exported library.
- [`autoware_tensorrt_vad`](../autoware_tensorrt_vad/README.md) — camera synchronization and
  ROS/CUDA domain separation follow its design.
