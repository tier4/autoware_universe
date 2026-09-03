#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Generate `ml_package_<model>.param.yaml` for a LatentDrive planner export.

Same contract as make_ml_package_param.py: the node reads every
network-describing value from a param file that ships WITH the model, and that
file is derived from the artifacts so it cannot disagree with them. LatentDrive
has a single graph and no feature extractor, so everything comes from the
planner ONNX:

- `video` [1, 3, T, H, W] fixes the frame count and input resolution (the node
  reads them from the engine; they are recorded here as comments);
- `status` [1, 6] is checked against the contract;
- `traj` [1, N, 3] supplies the horizon: N steps over 4 s, so the step is 4/N.
  The postprocessor accepts only 0.1 s steps, so the 40-waypoint (10 Hz)
  checkpoint is the deployable one and the 8-waypoint export is refused here.

What the ONNX cannot tell -- the frame spacing, the subgoal distance and the
status scaling -- is the training contract and is taken from the arguments.

Usage:
    make_latentdrive_ml_package_param.py MODEL_DIR --planner-onnx e2e-idx8-hist4-wp10hz-e3.onnx
"""

from __future__ import annotations

import argparse
from pathlib import Path

import onnx

HORIZON_SECONDS = 4.0
SUPPORTED_TIME_STEP = 0.1


def _tensor_shape(model: onnx.ModelProto, name: str) -> list:
    for value in list(model.graph.input) + list(model.graph.output):
        if value.name == name:
            return [d.dim_value if d.dim_value else (d.dim_param or -1)
                    for d in value.type.tensor_type.shape.dim]
    raise KeyError(f"{name} is not an input or output of the graph")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("model_dir", type=Path)
    parser.add_argument("--model-name", default="latentdrive")
    parser.add_argument("--planner-onnx", default=None,
                        help="planner ONNX file name (default: the only .onnx in MODEL_DIR)")
    parser.add_argument("--video-tensor", default="video")
    parser.add_argument("--status-tensor", default="status")
    parser.add_argument("--trajectory-tensor", default="traj")
    parser.add_argument(
        "--precision", default="fp16",
        help="TensorRT precision this graph is validated in. The ViT encoder carries the "
             "compute and runs fp16 on the vehicle; the offline validation pins the small "
             "planner stage to fp32, which autoware_tensorrt_common cannot express, so "
             "fp16 here is the pure-fp16 build")
    parser.add_argument("--frame-interval", type=float, default=0.5,
                        help="seconds between the clip's frames (training contract)")
    parser.add_argument("--subgoal-ahead-m", type=float, default=50.0,
                        help="arc length ahead on the route at which the subgoal is taken")
    parser.add_argument("--status-subgoal-divisor", type=float, default=10.0,
                        help="the subgoal is divided by this in the status vector")
    args = parser.parse_args()

    model_dir: Path = args.model_dir.expanduser().resolve()
    if args.planner_onnx:
        planner_path = model_dir / args.planner_onnx
    else:
        candidates = sorted(model_dir.glob("*.onnx"))
        if len(candidates) != 1:
            raise SystemExit(
                f"{model_dir} holds {len(candidates)} .onnx files; pass --planner-onnx")
        planner_path = candidates[0]
    if not planner_path.is_file():
        raise SystemExit(f"missing artifact: {planner_path}")

    planner = onnx.load(str(planner_path), load_external_data=False)
    video_shape = _tensor_shape(planner, args.video_tensor)
    status_shape = _tensor_shape(planner, args.status_tensor)
    traj_shape = _tensor_shape(planner, args.trajectory_tensor)

    if len(video_shape) != 5 or video_shape[1] != 3:
        raise SystemExit(f"{args.video_tensor} has shape {video_shape}; want [1, 3, T, H, W]")
    if [d for d in status_shape if d != 1] != [6]:
        raise SystemExit(f"{args.status_tensor} has shape {status_shape}; want [1, 6]")
    if len(traj_shape) != 3 or traj_shape[2] != 3:
        raise SystemExit(f"{args.trajectory_tensor} has shape {traj_shape}; want [1, N, 3]")

    _, _, num_frames, height, width = video_shape
    num_waypoints = int(traj_shape[1])
    time_step = HORIZON_SECONDS / num_waypoints
    if abs(time_step - SUPPORTED_TIME_STEP) > 1e-9:
        raise SystemExit(
            f"{planner_path.name} emits {num_waypoints} waypoints over {HORIZON_SECONDS} s "
            f"({time_step:.3g} s apart); the node's postprocessing accepts only "
            f"{SUPPORTED_TIME_STEP} s steps, so use the 10 Hz (40-waypoint) export")

    lines = [
        "# GENERATED by autoware_tensorrt_e2e/scripts/make_latentdrive_ml_package_param.py -- do",
        "# not hand-edit: regenerate it whenever the artifacts in this directory change.",
        f"# planner: {planner_path.name}",
        f"#   {args.video_tensor}: {video_shape}  ({num_frames} frames, {width}x{height})",
        f"#   {args.status_tensor}: {status_shape}",
        f"#   {args.trajectory_tensor}: {traj_shape}",
        "#",
        "# This file describes the NETWORK and overrides the package defaults for it.",
        "# Host and deployment behaviour (workspace, rates, tolerances, paths)",
        "# stays in the package's model-agnostic e2e_planner.param.yaml.",
        "/**:",
        "  ros__parameters:",
        "    # Which input providers this model needs, matched by tensor name.",
        '    sensor_inputs: ["latentdrive"]',
        "    enable_context_inputs: false",
        "",
        "    # Precision this graph is validated in; overrides the package default.",
        f"    precision: \"{args.precision}\"",
        "",
        "    latentdrive:",
        f"      video_tensor: \"{args.video_tensor}\"",
        f"      status_tensor: \"{args.status_tensor}\"",
        f"      frame_interval_seconds: {args.frame_interval}"
        f"  # {num_frames} frames span {(num_frames - 1) * args.frame_interval:g} s",
        f"      subgoal_ahead_m: {args.subgoal_ahead_m}",
        f"      status_subgoal_divisor: {args.status_subgoal_divisor}",
        "",
        "    postprocess:",
        f"      prediction_tensor: \"{args.trajectory_tensor}\"",
        # No extra_trajectory_tensors line: rclcpp cannot type an empty YAML list, and the
        # single-output planner has none. The node's default is the empty list.
        f"      horizon_seconds: {HORIZON_SECONDS}  # {num_waypoints} steps",
        f"      time_step: {SUPPORTED_TIME_STEP}",
        f"      generator_name: \"TensorrtE2e{args.model_name.capitalize()}\"",
        "",
    ]
    out_path = model_dir / f"ml_package_{args.model_name}.param.yaml"
    out_path.write_text("\n".join(lines))
    print(f"wrote {out_path}")
    print(f"  frames={num_frames} interval={args.frame_interval}s input={width}x{height} "
          f"waypoints={num_waypoints}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
