# autoware_tensorrt_e2e

One node that runs end-to-end trajectory planners with TensorRT. The model decides what
the node does: at startup the node reads the engine's input and output manifest, asks the
configured input providers to claim the tensors they can produce, validates every claimed
shape, and refuses to start if any model input is left unclaimed. Deploying another model,
or a re-trained variant of the same one, is a launch argument and a generated
configuration file, never an edit to this package.

The node does not reimplement what Autoware already has. Point clouds are taken and
voxelized the way `autoware_bevfusion` takes and voxelizes them; the map, route, ego and
traffic-light tensors are built by `autoware_diffusion_planner`'s own functions; the
output trajectory, its candidate list and its planning factors are produced by
`autoware_diffusion_planner`'s postprocessing. Where the two reference nodes report
something, this node reports the same thing under the same name.

## Supported models

| Model | Launch file | Sensing input |
| --- | --- | --- |
| **ResWorld** (OnePlanner) | `e2e_planner_resworld.launch.xml` | Concatenated point cloud, frozen BEVFusion-L features, 3-frame temporal history |

```bash
ros2 launch autoware_tensorrt_e2e e2e_planner_resworld.launch.xml
```

The model directory, `$(var data_path)/$(var model_name)` (by default
`$HOME/autoware_data/ml_models/tensorrt_e2e/resworld`), holds:

| File | Role |
| --- | --- |
| `resworld_planner.simplified.onnx` | The planner graph. Batch is frozen to 1 after simplification (`scripts/freeze_onnx_batch.py`): onnxsim miscompiles this graph when simplified with a static batch, and TensorRT will not build a dynamic one without a profile. |
| `resworld_deployment_contract.json` | The temporal-cache contract (frames, interval, warp geometry) and the tensor names. The node reads it; it is the source of truth over the yaml. |
| `bevfusion_lidar_feature.onnx` | The production BEVFusion lidar branch exported with its `bev_feature` map `[1, 512, 180, 180]` as the output. Carries the sparse-convolution custom nodes, so its engine needs `autoware_tensorrt_plugins` and an `spconv` build for this GPU. |
| `ml_package_resworld.param.yaml` | The network description, generated from the three files above (see [Configuration layout](#configuration-layout)). The copy under `config/` is a reference; the node reads the one beside the artifacts. |

The planner runs in fp32: its graph embeds normalization statistics that overflow fp16, and
TensorRT clips silently rather than failing. The extractor runs in fp16. The ResWorld graph
declares a `turn_indicators` input it never reads (perturbing it changes no output), so its
ml_package file sets `context.turn_indicators.enabled: false` and the node subscribes to
nothing for it.

## Architecture

```
timer (planning_frequency_hz)
  └─ ego frame            odometry, acceleration, ego→map transform
  └─ input providers      each claims tensors by name, fills them per tick
  │    camera             images + intrinsics + extrinsics (from TF)
  │    lidar              raw points, for a model that voxelizes internally
  │    bev_feature        frozen BEVFusion-L features, temporal history, SE(2)-warped
  │    context            the diffusion planner's ego / map / route / traffic-light tensors
  └─ inference            one TensorRT engine, built in-node with TrtCommon
  └─ postprocess          diffusion planner's trajectory conversion, 4 s at 0.1 s
  └─ publish              Trajectory, CandidateTrajectories, planning factors, diagnostics
```

Sensor providers are looked up in a registry by the names in `sensor_inputs`; each provider
registers itself from its own source file, so a model line adds a provider by adding a file,
and the node is identical on every branch that carries it.

## Sensor prototypes

| Prototype | Launch file | Sensing input |
| --- | --- | --- |
| Front camera | `e2e_planner_front_camera.launch.xml` | `CAM_FRONT_WIDE` |
| Surround cameras | `e2e_planner_surround_cameras.launch.xml` | Front wide plus four corner wide cameras |
| LiDAR, raw points | `e2e_planner_lidar.launch.xml` | `/sensing/lidar/concatenated/pointcloud`, for a model that voxelizes in-graph |
| LiDAR, BEV features | `e2e_planner_resworld.launch.xml` | Concatenated point cloud, temporal BEV feature history |

Add `build_only:=true` to any launch to build the TensorRT engines and exit.

## Model contract

The node accepts any single ONNX graph whose inputs are covered by the enabled providers,
matched by tensor name and validated by shape at startup, and whose primary output is an
ego trajectory tensor.

**Output.** `[B, A, T, 4]` (agent 0 is the ego, the rest are neighbour predictions) or
`[B, T, 4]` (ego only): `(x, y, cos yaw, sin yaw)` per 0.1 s step, in the ego frame. `T`
is validated against `postprocess.horizon_seconds` (40 steps, 4 s, for the current models).
The tensor name is `postprocess.prediction_tensor`; further ego-only trajectory outputs
listed in `postprocess.extra_trajectory_tensors` are published as extra candidates.

**Claimable inputs.**

| Provider | Tensor (default name) | Shape | Built by |
| --- | --- | --- | --- |
| camera | `camera_images` | `[1, N, 3, H, W]` | this package, normalized RGB, `H` and `W` from the engine |
| camera | `camera_intrinsics` | `[1, N, 3, 3]` | rescaled to the model resolution |
| camera | `camera2ego` | `[1, N, 4, 4]` | TF, camera frame to `base_link` |
| lidar | `points` | `[1, P, D]` | `D` in 3 to 5, padded or truncated to `P` |
| lidar | `num_points` | `[1, 1]` | valid point count |
| bev_feature | `bev_feature_history` | `[1, K, C, H, W]` | `autoware_bevfusion` voxelizer, the frozen extractor engine, this package's temporal cache and SE(2) warp |
| context | `ego_current_state` | `[1, 10]` | `autoware_diffusion_planner` |
| context | `ego_agent_past` | `[1, T, 4]` | `autoware_diffusion_planner` |
| context | `neighbor_agents_past` | `[1, N, 31, 11]` | `autoware_diffusion_planner` |
| context | `lanes`, `lanes_speed_limit`, `lanes_has_speed_limit` | `[1, S, 20, 33]`, `[1, S, 1]` | `autoware_diffusion_planner`; traffic-light state in channels 8 to 12 |
| context | `route_lanes` and its two speed-limit tensors | as above | `autoware_diffusion_planner` |
| context | `polygons`, `line_strings` | `[1, 10, 40, 3]`, `[1, 60, 20, 4]` | `autoware_diffusion_planner` |
| context | `goal_pose`, `ego_shape` | `[1, 4]`, `[1, 3]` | route goal in the ego frame; vehicle info |
| context | `turn_indicators` | `[1, T]` | report history, or a constant when disabled |
| context | `static_objects` | any | zero-filled, as in `autoware_diffusion_planner` |

Only the context tensors named in the engine manifest are produced, and only the
subscriptions they need are created. A model input that no provider can produce is a
startup error naming the tensor.

## Interface

### Inputs

| Topic | Type | Used by |
| --- | --- | --- |
| `~/input/odometry` | `nav_msgs/msg/Odometry` | always |
| `~/input/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | `ego_current_state` |
| `~/input/camera{i}/image` | `sensor_msgs/msg/Image`, raw or compressed | camera provider |
| `~/input/camera{i}/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_intrinsics` |
| `~/input/pointcloud` | `sensor_msgs/msg/PointCloud2` through `cuda_blackboard`: a GPU-resident cloud is negotiated on `~/input/pointcloud/cuda`, a plain publisher is accepted as the fallback | lidar providers, as `autoware_bevfusion` |
| `~/input/tracked_objects` | `autoware_perception_msgs/msg/TrackedObjects` | `neighbor_agents_past` |
| `~/input/traffic_signals` | `autoware_perception_msgs/msg/TrafficLightGroupArray` | `lanes`, `route_lanes` |
| `~/input/route` | `autoware_planning_msgs/msg/LaneletRoute` | `route_lanes`, `goal_pose` |
| `~/input/vector_map` | `autoware_map_msgs/msg/LaneletMapBin` | map tensors |
| `~/input/turn_indicators` | `autoware_vehicle_msgs/msg/TurnIndicatorsReport` | `turn_indicators`; not subscribed when `context.turn_indicators.enabled` is false |

A missing input stops planning for that tick with a throttled warning and a `WARN`
diagnostic naming the input. The one exception mirrors `autoware_diffusion_planner`: a
missing traffic-signal message leaves lanes marked as having no signal.

### Outputs

| Topic | Type | Content |
| --- | --- | --- |
| `~/output/trajectory` | `autoware_planning_msgs/msg/Trajectory` | Ego trajectory in `map`, 40 points at 0.1 s, stamped with the odometry it was planned from |
| `~/output/trajectories` | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | One candidate per batch and per extra trajectory tensor, with `GeneratorInfo` |
| `~/output/predicted_objects` | `autoware_perception_msgs/msg/PredictedObjects` | Multi-agent models only |
| `/planning/planning_factors/tensorrt_e2e` | `autoware_internal_planning_msgs/msg/PlanningFactorArray` | Stop and slow-down factors read off the trajectory, as `autoware_diffusion_planner` reports them |
| `~/debug/processing_time_ms` | `autoware_internal_debug_msgs/msg/Float64Stamped` | Per-tick processing time |
| `~/debug/cyclic_time_ms`, `~/debug/pipeline_latency_ms`, `~/debug/processing_time/{total,collect,inference,postprocess}_ms` | `autoware_internal_debug_msgs/msg/Float64Stamped` | The `autoware_bevfusion` debug set |
| `/diagnostics` | | `inference_status` |

The `inference_status` diagnostic carries the readiness state, the reason a tick was skipped,
a `WARN` when processing exceeded the planning period, and the keys the reference nodes
publish: `is_num_voxels_within_range` from `autoware_bevfusion`, and `valid_lane_count`,
`valid_route_count`, `valid_polygon_count`, `valid_line_string_count`,
`valid_neighbor_count` from `autoware_diffusion_planner`, each counted the same way.

### Trajectory semantics

Every field of the output is derived by `autoware_diffusion_planner`'s
`create_ego_trajectory`, so a consumer sees the same message shape from either planner:
velocity is the chord length between consecutive points over 0.1 s, seeded from the
current ego position; it is smoothed with a trailing window of `velocity_smoothing_window`
points; a drop below `stopping_threshold` while the ego is moving latches a stop, after
which velocity is zero and the pose is held; acceleration is the forward difference of the
smoothed velocity; the first point is at 0.1 s and the current pose is not prepended; `z`
is the ego's current `z`; lateral velocity and heading rate are left at zero. The only
difference from `autoware_diffusion_planner` is the horizon, 4 s instead of 8 s.

## Operation

The node is timer-driven at `planning_frequency_hz` (default 10 Hz); subscriptions only
latch messages. Each sensor input has a staleness bound (`*.max_delay_ms`), measured
against the node clock, so replaying a bag needs `use_sim_time:=true`. Processing time is
published per stage, and exceeding the planning period raises a `WARN` diagnostic
(`Processing time exceeded the planning period`). Model and preprocessing must fit the
100 ms budget on the target hardware.

TensorRT engines are built in-node by `TrtCommon` and cached beside the ONNX files. Both
engines are built with the `trt_workspace_mib` workspace (default 16 GiB): below a graph's
need the builder segfaults rather than failing, and the threshold moves with whatever else
holds GPU memory.

## Configuration layout

Parameters are split the way `autoware_bevfusion` splits them, so that deploying a model
never edits this package:

| File | Lives in | Holds |
| --- | --- | --- |
| `config/e2e_planner.param.yaml` | the package | Deployment defaults, model-agnostic: artifact paths (from launch arguments), TensorRT workspace, planning rate, staleness bounds, context, postprocess and planning-factor behaviour |
| `ml_package_<model>.param.yaml` | the model directory, beside the ONNX files | The network: which providers it needs, tensor names, voxelization geometry, temporal contract, horizon, validated precision, and which declared inputs it actually reads |

The launch loads the package defaults first and the ml_package second, so the model's
values win. Fields that describe the network are declared without defaults, as
`autoware_bevfusion` declares its ml_package fields: a model directory that lacks one fails
at startup instead of running with another network's geometry.

```bash
ros2 launch autoware_tensorrt_e2e <launch file> \
  data_path:=$HOME/autoware_data/ml_models/tensorrt_e2e model_name:=<model>
```

One file describes the network, and the node reads no other. Everything the runtime needs
about a model is in its ml_package file: which providers it needs, its tensor names, its
voxelization geometry, its history length and cadence, its horizon and its validated
precision.

That file is generated from the artifacts, never hand-written, so a new checkpoint cannot
disagree with a stale configuration:

```bash
ros2 run autoware_tensorrt_e2e make_ml_package_param.py <model_dir> --model-name <model> \
  --contract <export_dir>/<model>_deployment_contract.json [--turn-indicators-optional]
```

It reads points-per-voxel and the point-feature count out of the extractor's `voxels`
input, and the tensor names and horizon out of the planner graph. The exporter's deployment
contract supplies the one value no graph carries, the history cadence the model was trained
on; it is an input to the generator at deploy time, not an artifact the node reads, so the
deployed model directory keeps only the generated file. `--turn-indicators-optional`
records that the network never reads its `turn_indicators` input; verify that by perturbing
the input before setting it. Re-run the generator whenever the artifacts change.

Every parameter is described in `schema/tensorrt_e2e.schema.json`.

## Visualization

Every launch file takes `rviz:=true`, which opens the standard Autoware layout
(`autoware_launch/rviz/autoware.rviz`). That layout draws `/planning/trajectory`, so remap
the output there to see it:

```bash
ros2 launch autoware_tensorrt_e2e <launch file> rviz:=true use_sim_time:=true \
  output_trajectory:=/planning/trajectory
```

In a full Autoware stack the vehicle launch publishes `/robot_description`. A standalone
replay has nothing drawing the ego; add `vehicle_model_publisher:=true
vehicle_model:=<name>` to publish the body from `<name>_description`.

## Extending

- **Another checkpoint of a supported model.** Point `model_name` or `model_path` at the new
  artifacts and regenerate their ml_package file. No code change.
- **Another sensing modality.** Implement `InputProviderInterface` (`claim_inputs`,
  `collect`, and optionally `add_diagnostics` and `latest_input_stamp`) in its own source
  file, register it with `TENSORRT_E2E_REGISTER_INPUT_PROVIDER("<name>", factory)`, and name
  it in `sensor_inputs`. The node is not edited.
- **Another context feature.** Extend `ContextInputProvider` with the new claim; keep it
  optional, driven by the engine manifest.

## Reference nodes

| Package | What this node takes from it |
| --- | --- |
| [`autoware_bevfusion`](../../perception/autoware_bevfusion/README.md) | `cuda_blackboard` point cloud input, `PreprocessCuda` voxelization, the engine build through `TrtCommon`, the ml_package convention, the debug topic set and the voxel-range diagnostic |
| [`autoware_diffusion_planner`](../../planning/autoware_diffusion_planner/README.md) | Every context tensor, the trajectory conversion, candidate trajectories, planning factors and the valid-count diagnostics |
| [`autoware_tensorrt_vad`](../autoware_tensorrt_vad/README.md) | Camera synchronization and the separation of ROS and CUDA domains |
