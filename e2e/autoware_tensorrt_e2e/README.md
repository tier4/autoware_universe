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
| context | diffusion-planner-style tensor names | Optional ego, object, map, route, and turn-indicator features |

The standard output contract is an ego trajectory tensor named `prediction` by default:
`[B, T, 4]` or `[B, A, T, 4]`, with `(x, y, cos(yaw), sin(yaw))` in the model reference frame.
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

Tensor names used by camera and LiDAR providers are parameters, so retraining with different
names does not require a code change. A new modality or feature pipeline should implement
`InputProviderInterface` in a downstream branch and register it in the node's provider factory.

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
