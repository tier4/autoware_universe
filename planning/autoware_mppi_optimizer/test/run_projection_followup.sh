#!/usr/bin/env bash
# Copyright 2026 TIER IV, Inc.
# SPDX-License-Identifier: Apache-2.0

# Explicitly invoked only; never registered with CTest or the build.
set -euo pipefail

if [[ $# != 1 || ! $1 =~ ^(repeat|nsys|ncu)$ ]]; then
    echo "Usage: bash $0 {repeat|nsys|ncu}" >&2
    echo "Run from the workspace root after sourcing its runtime environment." >&2
    exit 2
fi
mppi_mode=$1
mppi_workspace=$PWD
mppi_binary="$mppi_workspace/build/autoware_mppi_optimizer/test_performance_benchmarks"
if [[ ! -x $mppi_binary ]]; then
    echo "Missing benchmark executable: $mppi_binary" >&2
    exit 1
fi
command -v python3 >/dev/null

mppi_tool=""
if [[ $mppi_mode == nsys ]]; then
    mppi_tool=${MPPI_NSYS:-$(command -v nsys || true)}
elif [[ $mppi_mode == ncu ]]; then
    mppi_tool=${MPPI_NCU:-$(command -v ncu || true)}
    if [[ -z $mppi_tool ]]; then
        # NVIDIA's standalone Linux installer does not always add ncu to PATH.
        shopt -s nullglob
        mppi_candidates=(/opt/nvidia/nsight-compute/*/ncu /usr/local/cuda/nsight-compute*/ncu)
        shopt -u nullglob
        if ((${#mppi_candidates[@]})); then
            mppi_tool=$(printf '%s\n' "${mppi_candidates[@]}" | sort -V | tail -n 1)
        fi
    fi
fi
if [[ $mppi_mode != repeat ]]; then
    mppi_tool=$(command -v "$mppi_tool" || true)
    if [[ ! -x $mppi_tool ]]; then
        echo "Profiler not found. Set MPPI_NCU or MPPI_NSYS to its executable path." >&2
        exit 1
    fi
fi

mkdir -p "$mppi_workspace/mppi_benchmark_results"
mppi_output=$(mktemp -d "$mppi_workspace/mppi_benchmark_results/projection_${mppi_mode}_$(date -u +%Y%m%d_%H%M%S)_XXXXXX")
echo "Results: $mppi_output"
exec > >(tee "$mppi_output/run.log") 2>&1

{
    date -u --iso-8601=seconds
    printf 'mode=%s\nworkspace=%s\nbinary=%s\n' "$mppi_mode" "$mppi_workspace" "$mppi_binary"
    sha256sum "$mppi_binary"
    # Checkout state supplements the configure-time revisions in benchmark JSON;
    # it does not establish which shared libraries were compiled from this state.
    for mppi_repo in src/autoware/universe src/vendor/mppi_generic_vendor; do
        printf '\n%s\n' "$mppi_repo"
        git -C "$mppi_repo" rev-parse HEAD
        git -C "$mppi_repo" status --short
    done
    printf '\nLinked libraries:\n'
    ldd "$mppi_binary"
    if [[ -n $mppi_tool ]]; then
        printf '\nProfiler: %s\n' "$mppi_tool"
        "$mppi_tool" --version
    fi
} >"$mppi_output/manifest.txt" 2>&1
if grep -q 'not found' "$mppi_output/manifest.txt"; then
    echo "A shared library is missing; source the workspace environment. See manifest.txt." >&2
    exit 1
fi

# Hash the optimizer library actually resolved by the loader, not just the small harness.
python3 - "$mppi_output/manifest.txt" <<'PY'
import hashlib
import pathlib
import sys

manifest = pathlib.Path(sys.argv[1])
for line in manifest.read_text().splitlines():
    if line.strip().startswith("libfirst_order_dubins_mppi.so => "):
        library = line.split("=> ", 1)[1].rsplit(" (", 1)[0]
        digest = hashlib.sha256()
        with open(library, "rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        with manifest.open("a") as stream:
            stream.write(f"\nOptimizer library SHA-256:\n{digest.hexdigest()}  {library}\n")
        break
else:
    sys.exit("Optimizer shared library not found in loader manifest")
PY

# Pass these assignments after sudo too: sudo normally strips LD_LIBRARY_PATH.
# Profile paths and GPU remapping retain the caller's settings in every mode.
mppi_environment=(MPPI_BENCHMARK_WARMUP=10 MPPI_BENCHMARK_ITERATIONS=10)
for mppi_variable in LD_LIBRARY_PATH CUDA_VISIBLE_DEVICES CUDA_DEVICE_ORDER \
    MPPI_BENCHMARK_DEVICE MPPI_BENCHMARK_COST_PROFILE MPPI_BENCHMARK_VEHICLE_PROFILE; do
    if [[ -v $mppi_variable ]]; then
        mppi_environment+=("$mppi_variable=${!mppi_variable}")
    fi
done
# Prevent inherited GTest sharding/repetition from silently changing the matrix.
unset GTEST_TOTAL_SHARDS GTEST_SHARD_INDEX GTEST_SHARD_STATUS_FILE
mppi_gtest=(--gtest_repeat=1 --gtest_shuffle=0)
mppi_scenes=(FreeSpaceCached Hairpin256Cached StaticObstacles64Cached
    StaticObstacleGeometryUpdates GridBoundaryCrossings ReferenceGeometryUpdates
    ShortProjectionPath OverlappingProjectionGroups)
mppi_filter=$(printf 'MppiPerformanceBenchmark.%s:' "${mppi_scenes[@]}")
mppi_filter=${mppi_filter%:}

snapshot_gpu() {
    if command -v nvidia-smi >/dev/null; then
        nvidia-smi -q >"$1" 2>&1 || true
    else
        echo "nvidia-smi unavailable" >"$1"
    fi
}

check_reports() {
    python3 - "$1" "$2" "$3" "$4" "$5" <<'PY'
import json
import pathlib
import sys

directory, count, pruning, phase, samples = sys.argv[1:]
reports = list(pathlib.Path(directory).glob("*.json"))
if len(reports) != int(count):
    sys.exit(f"Expected {count} benchmark reports in {directory}, got {len(reports)}")
for path in reports:
    report = json.loads(path.read_text())
    expected = {
        "measurements_valid": True,
        "unexpected_outcomes_including_warmup": 0,
        "warmup_calls_including_first": 10,
        "measured_calls": int(samples),
        "iterations": 10,
        "rollout_kernel": "split",
        "projection_pruning_enabled": bool(int(pruning)),
        "projection_pruning_math_supported": True,
        "projection_bounds_valid": True,
        "mode": "instrumented" if int(phase) else "latency_only",
    }
    for key, value in expected.items():
        if report.get(key) != value:
            sys.exit(f"{path}: {key}={report.get(key)!r}; expected {value!r}")
print(f"Validated {len(reports)} reports in {directory}")
PY
}

if [[ $mppi_mode == repeat ]]; then
    # Four independent off/on pairs per timing mode, with balanced order.
    # Start with pruning enabled, reversing the first supplied comparison.
    for mppi_pair in 1 2 3 4; do
        mppi_order=(1 0)
        if ((mppi_pair % 2 == 0)); then
            mppi_order=(0 1)
        fi
        for mppi_phase in 0 1; do
            for mppi_pruning in "${mppi_order[@]}"; do
                mppi_label="pair_${mppi_pair}_phase_${mppi_phase}_pruning_${mppi_pruning}"
                mppi_run="$mppi_output/$mppi_label"
                mkdir "$mppi_run"
                snapshot_gpu "$mppi_run/gpu_before.txt"
                env "${mppi_environment[@]}" MPPI_BENCHMARK_SAMPLES=100 \
                    MPPI_BENCHMARK_PHASE_TIMING="$mppi_phase" \
                    MPPI_BENCHMARK_PROJECTION_PRUNING="$mppi_pruning" \
                    MPPI_BENCHMARK_OUTPUT_DIR="$mppi_run" \
                    MPPI_BENCHMARK_ENVIRONMENT="${MPPI_BENCHMARK_ENVIRONMENT:-}; $mppi_label; no profiler" \
                    "$mppi_binary" "${mppi_gtest[@]}" --gtest_filter="$mppi_filter" \
                    2>&1 | tee "$mppi_run/console.txt"
                snapshot_gpu "$mppi_run/gpu_after.txt"
                check_reports "$mppi_run" 8 "$mppi_pruning" "$mppi_phase" 100
            done
        done
    done
else
    mppi_privilege=()
    if [[ $mppi_mode == ncu && $EUID != 0 ]]; then
        mppi_privilege=(sudo)
    fi
    for mppi_scene in FreeSpaceCached Hairpin256Cached StaticObstacles64Cached; do
        mppi_run="$mppi_output/$mppi_scene"
        mkdir "$mppi_run"
        snapshot_gpu "$mppi_run/gpu_before.txt"
        mppi_profile_environment=("${mppi_environment[@]}" MPPI_BENCHMARK_PHASE_TIMING=0
            MPPI_BENCHMARK_PROJECTION_PRUNING=1
            "MPPI_BENCHMARK_ENVIRONMENT=${MPPI_BENCHMARK_ENVIRONMENT:-}; $mppi_mode profiler; not a latency baseline")
        if [[ $mppi_mode == nsys ]]; then
            env "${mppi_profile_environment[@]}" MPPI_BENCHMARK_SAMPLES=20 \
                MPPI_BENCHMARK_OUTPUT_DIR="$mppi_run/benchmark_json" \
                "$mppi_tool" profile --trace=cuda,nvtx --sample=none --cpuctxsw=none \
                --output="$mppi_run/timeline" \
                "$mppi_binary" "${mppi_gtest[@]}" --gtest_filter="MppiPerformanceBenchmark.$mppi_scene"
            check_reports "$mppi_run/benchmark_json" 1 1 0 20
            "$mppi_tool" stats --report cuda_gpu_kern_sum,cuda_api_sum,nvtx_sum \
                "$mppi_run/timeline.nsys-rep" >"$mppi_run/timeline.txt" 2>&1
        else
            for mppi_kernel in dynamics cost; do
                mppi_kernel_name=rolloutDynamicsKernel
                if [[ $mppi_kernel == cost ]]; then
                    mppi_kernel_name=rolloutCostKernel
                fi
                mppi_prefix="$mppi_run/$mppi_kernel"
                "${mppi_privilege[@]}" env "${mppi_profile_environment[@]}" MPPI_BENCHMARK_SAMPLES=3 \
                    MPPI_BENCHMARK_OUTPUT_DIR="${mppi_prefix}_benchmark_json" \
                    "$mppi_tool" --config-file off --set full --replay-mode kernel \
                    --clock-control none --cache-control all --import-source yes \
                    --nvtx --nvtx-include "MPPI\\/rollout_${mppi_kernel}/" \
                    --kernel-name "regex:$mppi_kernel_name" --launch-skip 100 --launch-count 3 \
                    --export "$mppi_prefix" \
                    "$mppi_binary" "${mppi_gtest[@]}" --gtest_filter="MppiPerformanceBenchmark.$mppi_scene"
                check_reports "${mppi_prefix}_benchmark_json" 1 1 0 3
                "$mppi_tool" --config-file off --import "$mppi_prefix.ncu-rep" \
                    --page details >"$mppi_prefix.txt"
            done
        fi
        snapshot_gpu "$mppi_run/gpu_after.txt"
    done
fi
echo "Completed $mppi_mode collection: $mppi_output"
