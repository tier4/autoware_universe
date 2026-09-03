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

"""Freeze a planner graph's symbolic batch dimension to a fixed size.

Why this is a separate step rather than an exporter flag: onnxsim miscompiles
this graph when it is simplified with a static batch (a 22.7 m trajectory error
was traced to it), so the export must simplify with dynamic shapes. TensorRT,
in turn, refuses to build a network with dynamic inputs unless an optimization
profile is defined, and deployment only ever runs batch 1. Freezing after
simplification satisfies both, and gives the builder static shapes to
specialize on.

Rewrites the batch dimension of every graph input and output, drops the stale
intermediate shapes, and re-runs shape inference so the saved graph is
self-consistent.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import onnx
from onnx import shape_inference


def freeze(model: onnx.ModelProto, batch: int) -> list[str]:
    """Set dim 0 of every graph input/output to ``batch``; report what changed."""

    changed: list[str] = []
    for tensor in list(model.graph.input) + list(model.graph.output):
        dims = tensor.type.tensor_type.shape.dim
        if not dims:
            continue
        first = dims[0]
        if first.HasField("dim_param"):
            changed.append(f"{tensor.name}: {first.dim_param!r} -> {batch}")
            first.ClearField("dim_param")
            first.dim_value = batch
        elif first.dim_value != batch:
            changed.append(f"{tensor.name}: {first.dim_value} -> {batch}")
            first.dim_value = batch
    # Intermediate shapes still carry the symbolic batch. Inference cannot
    # narrow them while they are present, so clear and rebuild them.
    del model.graph.value_info[:]
    return changed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("onnx_path", type=Path, help="graph to rewrite")
    parser.add_argument("-o", "--output", type=Path, help="write here instead of in place")
    parser.add_argument("--batch", type=int, default=1, help="fixed batch size")
    args = parser.parse_args(argv)

    if args.batch < 1:
        parser.error(f"--batch must be positive, got {args.batch}")
    model = onnx.load(str(args.onnx_path))
    changed = freeze(model, args.batch)
    if not changed:
        print(f"{args.onnx_path.name}: batch already fixed at {args.batch}, nothing to do")
        return 0
    model = shape_inference.infer_shapes(model)
    onnx.checker.check_model(model)
    destination = args.output or args.onnx_path
    onnx.save(model, str(destination))
    print(f"wrote {destination}")
    for line in changed:
        print(f"  {line}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
