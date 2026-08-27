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

"""Plot MPPI delay-bicycle straight-line plant logs from mppi_open_loop_line_sim.

Example:
  ros2 run autoware_mppi_optimizer mppi_open_loop_line_sim_plot.py -- \
    --csv "$HOME/.cache/autoware/mppi_open_loop_line_sim/plant.csv"
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Dict
from typing import List

import matplotlib

if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_csv(path: Path) -> Dict[str, np.ndarray]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"empty csv: {path}")
        columns: Dict[str, List[float]] = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name in reader.fieldnames:
                columns[name].append(float(row[name]))
    return {name: np.asarray(values, dtype=float) for name, values in columns.items()}


def load_horizon_xy(path: Path) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
    if not path.is_file():
        return None, None
    data = load_csv(path)
    if "x" not in data or "y" not in data:
        return None, None
    return data["x"], data["y"]


def plot(plant: Dict[str, np.ndarray], horizon_x, horizon_y, out_path: Path, show: bool) -> None:
    t = plant["t"]
    fig = plt.figure(figsize=(12.0, 10.5), layout="constrained")
    grid = fig.add_gridspec(4, 2, height_ratios=[1.2, 1.0, 1.0, 1.0])

    ax_xy = fig.add_subplot(grid[0, 0])
    x_line = np.linspace(min(float(plant["x"].min()), 0.0), float(plant["x"].max()) + 5.0, 50)
    ax_xy.plot(x_line, np.zeros_like(x_line), color="0.65", linestyle="--", label="reference")
    if horizon_x is not None and horizon_y is not None:
        ax_xy.plot(horizon_x, horizon_y, color="tab:cyan", linewidth=1.5, label="first horizon")
    ax_xy.plot(plant["x"], plant["y"], color="tab:red", linewidth=2.0, label="plant")
    ax_xy.scatter(plant["x"][0], plant["y"][0], color="tab:red", zorder=3)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("XY")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best")

    ax_cte = fig.add_subplot(grid[0, 1])
    ax_cte.axhline(0.0, color="0.65", linestyle="--")
    ax_cte.plot(t, plant["cross_track_m"], color="tab:red", linewidth=2.0)
    ax_cte.set_xlabel("t [s]")
    ax_cte.set_ylabel("cross-track [m]")
    ax_cte.set_title("Cross-track error")
    ax_cte.grid(True, alpha=0.3)

    ax_v = fig.add_subplot(grid[1, 0])
    ax_v.plot(t, plant["v"], color="tab:blue", linewidth=1.8)
    ax_v.set_ylabel("v [m/s]")
    ax_v.set_title("Speed")
    ax_v.grid(True, alpha=0.3)

    ax_yaw = fig.add_subplot(grid[1, 1], sharex=ax_v)
    ax_yaw.plot(t, np.rad2deg(plant["yaw"]), color="tab:purple", linewidth=1.8)
    ax_yaw.set_ylabel("yaw [deg]")
    ax_yaw.set_title("Heading")
    ax_yaw.grid(True, alpha=0.3)

    ax_a = fig.add_subplot(grid[2, 0], sharex=ax_v)
    ax_a.plot(t, plant["accel"], color="tab:green", linewidth=1.8)
    ax_a.set_ylabel("a [m/s²]")
    ax_a.set_title("Longitudinal accel (plant)")
    ax_a.grid(True, alpha=0.3)

    ax_steer = fig.add_subplot(grid[2, 1], sharex=ax_v)
    ax_steer.plot(t, np.rad2deg(plant["steer"]), color="tab:orange", linewidth=1.8)
    ax_steer.set_ylabel("steer [deg]")
    ax_steer.set_title("Tire angle (plant)")
    ax_steer.grid(True, alpha=0.3)

    ax_ua = fig.add_subplot(grid[3, 0], sharex=ax_v)
    ax_ua.plot(t, plant["accel_cmd"], color="tab:green", linewidth=1.8)
    ax_ua.set_xlabel("t [s]")
    ax_ua.set_ylabel("accel cmd [m/s²]")
    ax_ua.set_title("Acceleration command")
    ax_ua.grid(True, alpha=0.3)

    ax_ud = fig.add_subplot(grid[3, 1], sharex=ax_v)
    ax_ud.plot(t, np.rad2deg(plant["steer_cmd"]), color="tab:orange", linewidth=1.8)
    ax_ud.set_xlabel("t [s]")
    ax_ud.set_ylabel("steer cmd [deg]")
    ax_ud.set_title("Steer command")
    ax_ud.grid(True, alpha=0.3)

    fig.suptitle("MPPI delay-bicycle open-loop straight-line tracking")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    print(f"Wrote {out_path}")
    if show:
        plt.show()
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path, help="plant.csv from the sim")
    parser.add_argument(
        "--horizon-csv",
        type=Path,
        default=None,
        help="optional first-horizon trajectory CSV (debug columns)",
    )
    parser.add_argument("--out", type=Path, default=None, help="output PNG (default: next to csv)")
    parser.add_argument("--show", action="store_true", help="open an interactive window")
    args = parser.parse_args()

    csv_path = args.csv.expanduser().resolve()
    plant = load_csv(csv_path)
    required = {
        "t",
        "x",
        "y",
        "yaw",
        "v",
        "accel",
        "steer",
        "accel_cmd",
        "steer_cmd",
        "cross_track_m",
    }
    missing = required.difference(plant)
    if missing:
        raise SystemExit(f"csv missing columns: {sorted(missing)}")

    horizon_csv = args.horizon_csv
    if horizon_csv is None:
        candidate = csv_path.with_name("horizon0.csv")
        horizon_csv = candidate if candidate.is_file() else None
    horizon_x = horizon_y = None
    if horizon_csv is not None:
        horizon_x, horizon_y = load_horizon_xy(horizon_csv.expanduser().resolve())

    out_path = args.out.expanduser().resolve() if args.out else csv_path.with_suffix(".png")
    plot(plant, horizon_x, horizon_y, out_path, args.show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
