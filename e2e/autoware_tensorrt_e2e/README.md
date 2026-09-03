# autoware_tensorrt_e2e

`autoware_tensorrt_e2e` is the model-agnostic foundation for running a single TensorRT
end-to-end planning model in Autoware. The model is selected by configuration; the node
introspects its TensorRT IO manifest and connects the requested tensors to enabled input
providers.

The foundation deliberately contains no model-vendor feature extractor or deployment contract.
Those belong in a downstream model branch/provider that depends on this package.

## Architecture

The runtime pipeline is:

```text
ROS topics -> InputProviderInterface implementations -> TensorMap
           -> InferenceEngine (named TensorRT IO) -> TensorMap
           -> TrajectoryPostprocessor -> Autoware planning topics
```

`InferenceEngine` is independent of tensor names. It loads or builds one TensorRT engine,
resolves a dynamic batch dimension to one, rejects unsupported dynamic dimensions, validates
input element counts, and returns host output tensors.

`InputProviderInterface` is the extension point for model inputs. Providers claim tensors from
the engine manifest in `claim_inputs()` and produce them on every planning tick in `collect()`.
The node rejects missing or multiply-claimed inputs at startup, so a model/deployment mismatch
is reported before inference starts.

The included providers are:

| Provider | Typical model input | Description |
| --- | --- | --- |
| `camera` | `camera_images`, `camera_intrinsics`, `camera2ego` | One or more synchronized cameras with GPU preprocessing |
| `lidar` | `points`, `num_points` | Padded/truncated point-cloud tensors |
| `latentdrive` | `video`, `status` | A front-camera clip and an ego status vector (see [LatentDrive](#latentdrive)) |
| context | diffusion-planner-style tensor names | Optional ego, object, map, route, and turn-indicator features |

The standard output contract is an ego trajectory tensor named `prediction` by default:
`[B, T, P]` or `[B, A, T, P]` in the model reference frame, where a pose is
`(x, y, cos(yaw), sin(yaw))` when `P` is 4 and `(x, y, yaw)` when `P` is 3.
The output tensor name, horizon, smoothing, and optional additional trajectory tensors are
configuration parameters. Outputs are published as `Trajectory`, `CandidateTrajectories`, and,
when neighbor predictions are present, `PredictedObjects`.

## Example deployments

The checked-in launch/config pairs exercise the foundation with different sensor layouts:

| Deployment | Launch file | Sensing input |
| --- | --- | --- |
| Front camera | `e2e_planner_front_camera.launch.xml` | One camera |
| Surround camera | `e2e_planner_surround_cameras.launch.xml` | Five cameras |
| Raw LiDAR | `e2e_planner_lidar.launch.xml` | Concatenated point cloud |
| LatentDrive | `e2e_planner_latentdrive.launch.xml` | Front-camera clip + ego status |

For example:

```bash
ros2 launch autoware_tensorrt_e2e e2e_planner_front_camera.launch.xml \
  data_path:=$HOME/autoware_data/ml_models/tensorrt_e2e
```

Use `build_only:=true` to build the TensorRT engine and exit.

## Model contract

At startup, all engine input tensors must be produced by an enabled provider. Names and shapes
are validated against the provider contract. A model can add or remove optional context tensors
without changing the node, provided those tensors are part of the context provider contract.

The foundation supports the following common tensor shapes:

| Provider | Tensor | Shape |
| --- | --- | --- |
| camera | `camera_images` | `[1, N, 3, H, W]` |
| camera | `camera_intrinsics` | `[1, N, 3, 3]` |
| camera | `camera2ego` | `[1, N, 4, 4]` |
| lidar | `points` | `[1, P, D]`, `D` in 3–5 |
| lidar | `num_points` | `[1, 1]` |
| latentdrive | `video` | `[1, 3, T, H, W]` |
| latentdrive | `status` | `[1, 6]` |

Tensor names used by the camera, LiDAR and LatentDrive providers are parameters, so retraining
with different names does not require a code change. A new modality or feature pipeline should
implement `InputProviderInterface` in a downstream branch and register it in the node's provider
factory.

### LatentDrive

LatentDrive is a V-JEPA2 ViT-L encoder over a short front-camera clip feeding a planner
transformer. Its inference framework lives in the `LatentDrive-TRT` repository; this package
carries only the input contract, in `LatentDriveInputProvider`, and runs the graph through the
common `InferenceEngine`.

- `video` `[1, 3, T, H, W]`: `T` frames `latentdrive.frame_interval_seconds` apart (0.5 s),
  oldest first, channel-major. Each frame is preprocessed once on arrival exactly as in the
  training pipeline: RGB, centre-crop the height to width / 2, bilinear resize to `W x H`,
  scale to [0, 1], ImageNet-normalize. Frames are chosen by stamp, so a dropped camera frame
  shifts one slot to its neighbour rather than compressing the clip.
- `status` `[1, 6]`: `(subgoal_x / 10, subgoal_y / 10, v_x, v_y, a_x, a_y)` in the ego frame.
  The subgoal is the point `latentdrive.subgoal_ahead_m` (50 m) of arc length ahead on the
  reference trajectory subscribed at `~/input/reference_trajectory`. In an open-loop replay the
  recorded planner trajectory plays that role; a closed-loop deployment needs a route-based
  source. The subgoal actually used is published on `~/debug/latentdrive/subgoal`.
- `traj` `[1, 40, 3]`: `(x, y, yaw)` at 0.1 s over 4 s, decoded by the common postprocessor
  through `LatentDrivePostprocessor`, which adds an optional temporal smoothing
  (`latentdrive.smoothing.*`, off by default). Consecutive plans disagree by about a metre on
  how far they reach, which reads as jitter in RViz and as a restless reference for a
  controller. The filter, ported from LatentDrive-TRT's display smoother, carries the previous
  plan forward by the ego's measured motion and blends the fresh plan into it; a large end-point
  jump or a time gap resets it. With it on, the unfiltered plan is still published as the
  candidate trajectory whose generator name ends in `_raw`, so open-loop accuracy is measured
  on the model's output and the filter is declared as part of the system rather than hidden.
  `smoothing:=true` on the launch file turns it on for one run.

The offline validation builds the engine fp16 with the planner stage pinned to fp32, a layer
precision recipe `autoware_tensorrt_common` cannot express; `precision` here is therefore the
pure fp16 or fp32 build. To run the validated engine instead, place it beside the ONNX under the
same stem (`<planner>.engine`): `autoware_tensorrt_common` loads an existing engine file before
it builds one, provided it was serialized by the same TensorRT version. Its `ml_package` file
comes from
`scripts/make_latentdrive_ml_package_param.py`, which reads the frame count, input resolution
and horizon out of the planner graph and refuses the 8-waypoint (2 Hz) export, whose 0.5 s
step the postprocessor does not support.

```bash
python3 scripts/make_latentdrive_ml_package_param.py <model_dir> --planner-onnx <planner>.onnx
ros2 launch autoware_tensorrt_e2e e2e_planner_latentdrive.launch.xml \
  model_path:=<model_dir> planner_onnx:=<planner>.onnx use_sim_time:=true \
  rviz:=true output_trajectory:=/planning/trajectory \
  vehicle_model_publisher:=true vehicle_model:=sample_vehicle
ros2 bag play <bag> --clock --topics /sensing/camera/camera1/image_raw/compressed \
  /localization/kinematic_state /localization/acceleration \
  /planning/scenario_planning/lane_driving/trajectory /tf /tf_static
```

Checked on a 60 s recording of a decelerate-to-red-light scene: the 4 s plan reaches
43 m at 11.5 m/s and shrinks to nothing as the ego stops, heading within a degree of the
ego's, at 10 Hz with about 28 ms per tick on a laptop RTX 4060.

## Visualization

Every launch file takes `rviz:=true`, which opens the standard Autoware layout
(`autoware_launch/rviz/autoware.rviz`). That layout draws `/planning/trajectory`, so remap
the output there to see it:

```bash
ros2 launch autoware_tensorrt_e2e <launch file> rviz:=true use_sim_time:=true \
  output_trajectory:=/planning/trajectory
```

`use_sim_time:=true` is required whenever inputs come from a bag: without it the node
measures input staleness against wall time and drops every message.

In a full Autoware stack the vehicle launch publishes `/robot_description`. For a standalone
replay nothing does, and the ego is absent from the scene; add
`vehicle_model_publisher:=true vehicle_model:=<name>` to publish the body from
`<name>_description`.

## Configuration layout

Parameters are split the way `autoware_bevfusion` splits them, so that deploying a model
never edits this package:

| File | Lives in | Holds |
| ---- | -------- | ----- |
| `config/e2e_planner.param.yaml` | the package | deployment defaults, **model-agnostic**: artifact paths (from launch arguments), TensorRT workspace, planning rate, staleness tolerances, context and postprocess behaviour |
| `ml_package_<model>.param.yaml` | **the model directory, beside the ONNX files** | the network itself: tensor names, input geometry, horizon, and the precision the graph is validated in |

A launch file loads the package defaults first and the ml_package second, so a model's values
override the defaults and switching models is a launch argument:

```bash
ros2 launch autoware_tensorrt_e2e <launch file> \
  data_path:=$HOME/autoware_data/ml_models/tensorrt_e2e model_name:=<model>
```

`ml_package_<model>.param.yaml` is **generated from the artifacts**, never hand-written, so a
new checkpoint cannot silently disagree with a stale config:

```bash
python3 scripts/make_ml_package_param.py <model_dir> --model-name <model>
```

It reads the tensor names and horizon out of the planner graph, and — for models that ship a
feature extractor and a deployment contract — the input geometry out of the extractor graph and
the cache semantics out of the contract JSON. Re-run it whenever those artifacts change.

### TensorRT workspace

`trt_workspace_mib` (default 4096) is a property of the deployment host, not of the model.
Sizing it below what a graph needs does not merely cost performance: the builder segfaults.

## Runtime behavior

The node is timer-driven at `planning_frequency_hz` (10 Hz by default). Sensor callbacks only
cache messages; expensive preprocessing and inference happen in the timer callback. Processing
time is published on `~/debug/processing_time_ms` and budget overruns are reported through
diagnostics.

## Dependencies

The foundation links only the generic TensorRT/CUDA, camera, LiDAR, context, and postprocessing
dependencies. Model-specific feature extractors and their vendor libraries should be added by
the downstream model branch that implements the corresponding provider.
