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

r"""Offline temporal-MPT weight retune (MPPI debug-log replay).

Replays logged diffusion/MPPI references from ``enable_debug_trajectory_log`` CSVs
through the Python acados temporal MPC with optionally overridden LINEAR_LS weights,
and writes retuned optimized trajectories (same CSV layout as MPPI logs) for
``mppi_debug_visualizer.py --log-dir``.

Example::

  ros2 run autoware_trajectory_processor temporal_mpt_offline_retune.py -- \\
    --log-dir "$HOME/.cache/autoware/mppi_debug_log" \\
    --out-dir "$HOME/.cache/autoware/temporal_mpt_retune" \\
    --frame 12 --set qpsi=2.0 --set rdelta=0.5 --copy-reference
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import shutil
import sys
from typing import Any


def _find_acados_mpt_root() -> Path:
    env = os.environ.get("AUTOWARE_ACADOS_MPT_ROOT")
    if env:
        p = Path(env)
        if (p / "generators" / "path_tracking_mpc_temporal.py").is_file():
            return p.resolve()
    here = Path(__file__).resolve().parent
    pkg_root = here.parent
    cand = pkg_root / "src" / "acados_mpt"
    if (cand / "generators" / "path_tracking_mpc_temporal.py").is_file():
        return cand.resolve()
    walk = here
    for _ in range(12):
        cand = walk / "src" / "acados_mpt"
        if (cand / "generators" / "path_tracking_mpc_temporal.py").is_file():
            return cand.resolve()
        if walk.parent == walk:
            break
        walk = walk.parent
    raise RuntimeError(
        "Set AUTOWARE_ACADOS_MPT_ROOT to the acados_mpt directory (contains generators/). "
        "Required when the package is used from install space without a nearby source tree."
    )


def _setup_paths(acados_mpt_root: Path) -> None:
    acados = os.environ.get("ACADOS_SOURCE_DIR", "/opt/acados")
    tpl = Path(acados) / "interfaces" / "acados_template"
    if not tpl.is_dir():
        raise RuntimeError(f"ACADOS_SOURCE_DIR invalid: missing {tpl}")
    for p in (str(tpl.resolve()), str(acados_mpt_root.resolve())):
        if p not in sys.path:
            sys.path.insert(0, p)


def _frame_tag(frame_id: int) -> str:
    return f"{frame_id:06d}"


def discover_log_frames(log_dir: Path) -> list[int]:
    index_path = log_dir / "index.csv"
    frame_ids: list[int] = []
    if index_path.is_file():
        lines = index_path.read_text().splitlines()
        start = 1 if lines and lines[0].strip().startswith("frame_id") else 0
        for line in lines[start:]:
            line = line.strip()
            if not line:
                continue
            cell = line.split(",", 1)[0]
            try:
                frame_ids.append(int(cell))
            except ValueError:
                continue
        if frame_ids:
            # index.csv can append the same frame_id more than once across sessions
            return sorted(dict.fromkeys(frame_ids))
    for ref in sorted(log_dir.glob("*_reference.csv")):
        stem = ref.name[: -len("_reference.csv")]
        if stem.isdigit():
            frame_ids.append(int(stem))
    return sorted(dict.fromkeys(frame_ids))


def load_key_value_csv(path: Path) -> dict[str, float]:
    if not path.is_file():
        return {}
    out: dict[str, float] = {}
    with path.open(newline="") as f:
        reader = csv.reader(f)
        rows = list(reader)
    if not rows:
        return out
    start = 1 if rows[0] and rows[0][0].strip().lower() in {"key", "name"} else 0
    for row in rows[start:]:
        if len(row) < 2:
            continue
        try:
            out[row[0].strip()] = float(row[1].strip())
        except ValueError:
            continue
    return out


def bicycle_lf_lr_from_vehicle_params(vehicle: dict[str, float]) -> tuple[float, float]:
    """Derive ``lf``, ``lr`` with ``L = lf + lr = wheel_base``.

    ``lr = rear_axle → box_center``, ``lf = wheel_base - lr``.
    Explicit ``lf``/``lr`` keys in the CSV win when present.
    """
    wb = float(vehicle.get("wheel_base", 4.76))
    wb = max(wb, 1.0e-3)
    if "lf" in vehicle and "lr" in vehicle:
        lf = float(vehicle["lf"])
        lr = float(vehicle["lr"])
        return lf, lr
    lr = float(vehicle.get("ego_axle_to_box_center", 0.5 * wb))
    lr = min(max(lr, 1.0e-3), wb - 1.0e-3)
    lf = wb - lr
    return lf, lr


def actuator_params_from_vehicle(vehicle: dict[str, float]) -> tuple[float, float, float]:
    """Return ``(tau_a, tau_d, max_steer_rate)`` from logged vehicle_params."""
    tau_a = float(vehicle.get("acc_time_constant", vehicle.get("accel_time_constant", 0.15)))
    tau_d = float(vehicle.get("steer_time_constant", 0.08))
    rate = float(vehicle.get("steer_rate_lim", vehicle.get("max_steer_rate", 3.0)))
    return max(1.0e-4, tau_a), max(1.0e-4, tau_d), max(1.0e-6, rate)


def load_trajectory_csv(path: Path) -> tuple[Any, Any, Any, Any, Any, Any, Any]:
    """Return t, x, y, z, yaw, v, a from an MPPI-style trajectory CSV."""
    import numpy as np

    rows: list[list[float]] = []
    with path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"Empty trajectory CSV: {path}")
        for row in reader:
            if not row or all(not c.strip() for c in row):
                continue
            vals = [float(c) for c in row]
            if len(vals) < 6:
                continue
            rows.append(vals)
    if len(rows) < 2:
        raise ValueError(f"Need >=2 points in {path}")
    arr = np.asarray(rows, dtype=float)
    t = arr[:, 0]
    x = arr[:, 1]
    y = arr[:, 2]
    z = arr[:, 3] if arr.shape[1] > 3 else np.zeros(len(rows))
    yaw = arr[:, 4] if arr.shape[1] > 4 else np.zeros(len(rows))
    v = arr[:, 5] if arr.shape[1] > 5 else np.zeros(len(rows))
    a = arr[:, 6] if arr.shape[1] > 6 else np.zeros(len(rows))
    return t, x, y, z, yaw, v, a


def load_ego_csv(path: Path) -> dict[str, float] | None:
    if not path.is_file():
        return None
    with path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        row = next(reader, None)
    if header is None or row is None:
        return None
    keys = [h.strip() for h in header]
    vals = [float(c) for c in row]
    return {k: vals[i] for i, k in enumerate(keys) if i < len(vals)}


def write_trajectory_csv(
    path: Path,
    *,
    t: Any,
    x: Any,
    y: Any,
    z: Any,
    yaw: Any,
    v: Any,
    a: Any,
    steer: Any,
) -> None:
    n = len(x)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_from_start_s", "x", "y", "z", "yaw", "v", "a", "steer", "steer_rate"])
        for i in range(n):
            ti = float(t[i]) if i < len(t) else float(i) * 0.1
            ai = float(a[i]) if i < len(a) else 0.0
            di = float(steer[i]) if i < len(steer) else 0.0
            rate = 0.0
            if i + 1 < len(steer) and i + 1 < len(t):
                dt = float(t[i + 1]) - float(t[i])
                if abs(dt) > 1.0e-9:
                    rate = (float(steer[i + 1]) - di) / dt
            w.writerow(
                [
                    f"{ti:.9f}",
                    f"{float(x[i]):.9f}",
                    f"{float(y[i]):.9f}",
                    f"{float(z[i]) if i < len(z) else 0.0:.9f}",
                    f"{float(yaw[i]):.9f}",
                    f"{float(v[i]):.9f}",
                    f"{ai:.9f}",
                    f"{di:.9f}",
                    f"{rate:.9f}",
                ]
            )


def write_key_value_csv(path: Path, kv: dict[str, float]) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["key", "value"])
        for k, v in kv.items():
            w.writerow([k, f"{float(v):.9f}"])


def load_weights_yaml(path: Path) -> dict[str, float]:
    text = path.read_text()
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(text) or {}
    except Exception:
        # Minimal fallback: key: value lines
        data = {}
        for line in text.splitlines():
            s = line.strip()
            if not s or s.startswith("#") or ":" not in s:
                continue
            k, v = s.split(":", 1)
            k = k.strip()
            v = v.strip().split("#", 1)[0].strip()
            try:
                data[k] = float(v)
            except ValueError:
                continue
    if not isinstance(data, dict):
        raise ValueError(f"Weights yaml must be a mapping: {path}")
    # Allow nested ros-style {"/**": {"ros__parameters": {...}}} or flat
    if "ros__parameters" in data and isinstance(data["ros__parameters"], dict):
        data = data["ros__parameters"]
    out: dict[str, float] = {}
    for k, v in data.items():
        if isinstance(v, (int, float)):
            out[str(k)] = float(v)
    return out


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def tracking_metrics(
    sol_x: Any,
    x_r: Any,
    y_r: Any,
    psi_r: Any,
    v_r: Any,
    start_idx: int,
    x0: Any | None = None,
) -> dict[str, float]:
    import numpy as np

    n = int(sol_x.shape[0])
    e_xy: list[float] = []
    e_psi: list[float] = []
    e_v: list[float] = []
    for i in range(n):
        if i == 0 and x0 is not None:
            rx = float(x0[0])
            ry = float(x0[1])
            rpsi = float(x0[2])
            rv = float(x0[3])
        else:
            j = min(start_idx + i, len(x_r) - 1)
            rx = float(x_r[j])
            ry = float(y_r[j])
            rpsi = float(psi_r[j])
            rv = float(v_r[j])
        e_xy.append(float(np.hypot(float(sol_x[i, 0]) - rx, float(sol_x[i, 1]) - ry)))
        e_psi.append(abs(wrap_pi(float(sol_x[i, 2]) - rpsi)))
        e_v.append(abs(float(sol_x[i, 3]) - rv))
    return {
        "mean_e_xy": float(np.mean(e_xy)),
        "max_e_xy": float(np.max(e_xy)),
        "mean_abs_psi_err": float(np.mean(e_psi)),
        "max_abs_psi_err": float(np.max(e_psi)),
        "mean_abs_v_err": float(np.mean(e_v)),
        "max_abs_v_err": float(np.max(e_v)),
    }


def horizon_tracking_series(
    sol_x: Any,
    sol_u: Any,
    x_pts: Any,
    y_pts: Any,
    psi_pts: Any,
    v_pts: Any,
    start_idx: int,
    x0: Any,
    *,
    dt: float,
) -> dict[str, Any]:
    """Per-stage state vs the **same yref the MPC cost uses** (path-frame errors).

    Stage 0: reference = ego ``x0`` (matches ``path_tracking_solver`` / ``setStageReference``).
    Stage k>=1: reference = ``ref[min(start_idx + k, n-1)]`` with ``psi_bias`` on yaw.
    """
    import numpy as np

    x0 = np.asarray(x0, dtype=float)
    n = int(sol_x.shape[0])
    t = np.arange(n, dtype=float) * float(dt)
    psi_bias = round((float(x0[2]) - float(psi_pts[start_idx])) / (2.0 * math.pi)) * (2.0 * math.pi)

    ref_x = np.zeros(n, dtype=float)
    ref_y = np.zeros(n, dtype=float)
    ref_yaw = np.zeros(n, dtype=float)
    ref_v = np.zeros(n, dtype=float)
    e_lon = np.zeros(n, dtype=float)
    e_lat = np.zeros(n, dtype=float)
    e_psi = np.zeros(n, dtype=float)
    e_v = np.zeros(n, dtype=float)
    # Offset vs closest polyline index (diagnostic only; stage 0 differs from MPC cost).
    e_lat_poly = np.zeros(n, dtype=float)
    e_lon_poly = np.zeros(n, dtype=float)

    for i in range(n):
        if i == 0:
            rx = float(x0[0])
            ry = float(x0[1])
            rpsi = float(x0[2])
            rv = max(0.0, float(x0[3]))
        else:
            j = min(start_idx + i, len(x_pts) - 1)
            rx = float(x_pts[j])
            ry = float(y_pts[j])
            rpsi = float(psi_pts[j]) + psi_bias
            rv = max(0.0, float(v_pts[j]))
        ref_x[i] = rx
        ref_y[i] = ry
        ref_yaw[i] = rpsi
        ref_v[i] = rv
        dx = float(sol_x[i, 0]) - rx
        dy = float(sol_x[i, 1]) - ry
        c = math.cos(rpsi)
        s = math.sin(rpsi)
        e_lon[i] = dx * c + dy * s
        e_lat[i] = -dx * s + dy * c
        e_psi[i] = wrap_pi(float(sol_x[i, 2]) - rpsi)
        e_v[i] = float(sol_x[i, 3]) - rv

        j_poly = min(start_idx + i, len(x_pts) - 1)
        rx_p = float(x_pts[j_poly])
        ry_p = float(y_pts[j_poly])
        rpsi_p = float(psi_pts[j_poly])
        dx_p = float(sol_x[i, 0]) - rx_p
        dy_p = float(sol_x[i, 1]) - ry_p
        c_p = math.cos(rpsi_p)
        s_p = math.sin(rpsi_p)
        e_lon_poly[i] = dx_p * c_p + dy_p * s_p
        e_lat_poly[i] = -dx_p * s_p + dy_p * c_p

    n_u = int(sol_u.shape[0])
    a_cmd = np.zeros(n, dtype=float)
    d_cmd = np.zeros(n, dtype=float)
    a_cmd[:n_u] = sol_u[:, 0]
    d_cmd[:n_u] = sol_u[:, 1]
    if n_u > 0:
        a_cmd[n_u:] = sol_u[-1, 0]
        d_cmd[n_u:] = sol_u[-1, 1]

    return {
        "t": t,
        "start_idx": int(start_idx),
        "psi_bias": float(psi_bias),
        "ref_x": ref_x,
        "ref_y": ref_y,
        "ref_yaw": ref_yaw,
        "ref_v": ref_v,
        "e_lon": e_lon,
        "e_lat": e_lat,
        "e_psi": e_psi,
        "e_v": e_v,
        "e_lon_poly": e_lon_poly,
        "e_lat_poly": e_lat_poly,
        "a_state": np.asarray(sol_x[:, 4], dtype=float),
        "delta_state": np.asarray(sol_x[:, 5], dtype=float),
        "a_cmd": a_cmd,
        "delta_cmd": d_cmd,
    }


def closest_index(x0: Any, x_pts: Any, y_pts: Any) -> int:
    import numpy as np

    dx = x_pts - float(x0[0])
    dy = y_pts - float(x0[1])
    return int(np.argmin(dx * dx + dy * dy))


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Replay MPPI debug logs through temporal MPT with overridable W/W_e weights."
    )
    p.add_argument(
        "--log-dir", type=Path, required=True, help="Input log dir (index + *_reference.csv)"
    )
    p.add_argument("--out-dir", type=Path, required=True, help="Output directory for retuned CSVs")
    p.add_argument("--frame", type=int, default=None, help="Only retune frame N (default: all)")
    p.add_argument(
        "--params-yaml",
        type=Path,
        default=None,
        help="Optional yaml with weight keys (qlong,qlat,qpsi,qv,ra,rdelta,qe_scale,...); "
        "aliases qx→qlong, qy→qlat",
    )
    p.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="key=value",
        help="Override a weight (repeatable). Keys: qlong,qlat,qpsi,qv,ra,rdelta,"
        "qelong,qelat,qepsi,qev,qe_scale (aliases qx/qy, qex/qey)",
    )
    p.add_argument("--dt", type=float, default=0.1, help="MPC stage dt [s] (default 0.1)")
    p.add_argument(
        "--N", type=int, default=80, help="MPC horizon stages (default 80, match codegen)"
    )
    p.add_argument(
        "--lf",
        type=float,
        default=None,
        help="Bicycle lf [m] (default: wheel_base - ego_axle_to_box_center from log)",
    )
    p.add_argument(
        "--lr",
        type=float,
        default=None,
        help="Bicycle lr [m] (default: ego_axle_to_box_center from log)",
    )
    p.add_argument(
        "--copy-reference",
        action="store_true",
        help="Also copy reference (+ ego) CSVs into out-dir for visualizer replay",
    )
    p.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="Acados Python build directory (default: <out-dir>/_acados_build)",
    )
    p.add_argument(
        "--generate",
        action="store_true",
        help="Force regenerate/compile the Python acados solver in build-dir",
    )
    p.add_argument(
        "--use-ego-ic",
        action="store_true",
        default=True,
        help="Seed x0 from *_ego.csv (default; matches MPPI TemporalMptNominalSeeder)",
    )
    p.add_argument(
        "--use-reference-ic",
        action="store_true",
        help="Seed x0 from reference[0] instead of ego (C++ trajectory_optimizer plugin parity)",
    )
    return p.parse_args(argv)


class TemporalMptRetuneSession:
    """Warm acados solver for interactive / batch temporal-MPT weight retunes."""

    def __init__(
        self,
        *,
        n_horizon: int = 80,
        dt: float = 0.1,
        lf: float = 1.0,
        lr: float = 1.0,
        tau_a: float = 0.15,
        tau_d: float = 0.08,
        max_steer_rate: float = 3.0,
        build_dir: Path | None = None,
        generate: bool = False,
        quiet_build: bool = True,
    ) -> None:
        import numpy as np

        acados_mpt_root = _find_acados_mpt_root()
        _setup_paths(acados_mpt_root)

        from generators.path_tracking_mpc_temporal import PathTrackingMPCTemporal
        from scripts.utils import apply_cost_weights
        from scripts.utils import default_q_r_qe
        from scripts.utils import parse_weight_overrides
        from scripts.utils import scale_stage_and_terminal_w
        from scripts.utils import weights_dict_from_q_r_qe

        self._np = np
        self._apply_cost_weights = apply_cost_weights
        self._parse_weight_overrides = parse_weight_overrides
        self._scale_stage_and_terminal_w = scale_stage_and_terminal_w
        self._weights_dict_from_q_r_qe = weights_dict_from_q_r_qe

        from scripts.utils import NY_STAGE_DEFAULT
        from scripts.utils import pad_time_series_reference
        from scripts.utils import solve_autoware_temporal_mpc

        self._ny = NY_STAGE_DEFAULT
        self._pad = pad_time_series_reference
        self._solve = solve_autoware_temporal_mpc

        self.n_horizon = int(n_horizon)
        self.dt = float(dt)
        self.tf = float(self.n_horizon) * self.dt
        self.lf = float(lf)
        self.lr = float(lr)
        self.tau_a = max(1.0e-4, float(tau_a))
        self.tau_d = max(1.0e-4, float(tau_d))
        self.max_steer_rate = max(1.0e-6, float(max_steer_rate))
        self.p_vec = np.array([self.lf, self.lr, self.tau_a, self.tau_d], dtype=float)

        if build_dir is None:
            cache = os.environ.get("XDG_CACHE_HOME")
            root = Path(cache) if cache else Path.home() / ".cache"
            build_dir = root / "autoware" / "temporal_mpt_retune_build_fod_h0"
        self.build_dir = Path(build_dir).expanduser().resolve()
        self.build_dir.mkdir(parents=True, exist_ok=True)

        q, r, qe = default_q_r_qe()
        self._q, self._r, self._qe = q, r, qe
        self.weight_kv = weights_dict_from_q_r_qe(q, r, qe)

        old_cwd = os.getcwd()
        os.chdir(self.build_dir)
        try:
            json_exists = (self.build_dir / "acados_ocp.json").is_file()
            do_generate = bool(generate or not json_exists)
            if quiet_build:
                # First compile dumps make noise; keep the interactive UI usable.
                with open(os.devnull, "w") as devnull:
                    old_out, old_err = sys.stdout, sys.stderr
                    try:
                        sys.stdout = sys.stderr = devnull
                        mpc = PathTrackingMPCTemporal(
                            self.tf, self.n_horizon, build=True, generate=do_generate
                        )
                    finally:
                        sys.stdout, sys.stderr = old_out, old_err
            else:
                mpc = PathTrackingMPCTemporal(
                    self.tf, self.n_horizon, build=True, generate=do_generate
                )
            self.solver = mpc.acados_solver
            w, w_e = scale_stage_and_terminal_w(q, r, qe, n_horizon=self.n_horizon, tf=self.tf)
            apply_cost_weights(self.solver, self.n_horizon, w, w_e)
            self._apply_steer_rate_limit()
        finally:
            os.chdir(old_cwd)

    def _apply_steer_rate_limit(self) -> None:
        """Update lh/uh on stages that have nonlinear h (nh_0 / nh may differ)."""
        lim = float(self.max_steer_rate)
        lh = self._np.array([-lim])
        uh = self._np.array([lim])
        for i in range(self.n_horizon):
            # Stage 0 uses nh_0; intermediate stages use nh. Skip empty dims.
            try:
                self.solver.constraints_set(i, "lh", lh)
                self.solver.constraints_set(i, "uh", uh)
            except ValueError:
                continue

    def set_weights(self, overrides: dict[str, float]) -> dict[str, float]:
        q, r, qe = self._parse_weight_overrides(overrides)
        self._q, self._r, self._qe = q, r, qe
        self.weight_kv = self._weights_dict_from_q_r_qe(q, r, qe)
        w, w_e = self._scale_stage_and_terminal_w(q, r, qe, n_horizon=self.n_horizon, tf=self.tf)
        old_cwd = os.getcwd()
        os.chdir(self.build_dir)
        try:
            self._apply_cost_weights(self.solver, self.n_horizon, w, w_e)
        finally:
            os.chdir(old_cwd)
        return dict(self.weight_kv)

    def solve_frame(
        self,
        log_dir: Path,
        frame_id: int,
        *,
        use_ego_ic: bool = False,
    ) -> dict[str, Any]:
        """Solve one logged frame. Returns arrays + metrics (no disk writes).

        Initial state matches online C++ ``TrajectoryTemporalMPTOptimizer`` by default:
        ``x0 = reference[0]`` (trajectory front). Pass ``use_ego_ic=True`` to seed from
        ``*_ego.csv`` instead (odometry replay; not C++ plugin parity).
        """
        np = self._np
        tag = _frame_tag(frame_id)
        ref_path = Path(log_dir) / f"{tag}_reference.csv"
        if not ref_path.is_file():
            raise FileNotFoundError(f"Missing {ref_path}")

        _, x_r, y_r, z_r, psi_r, v_r, _ = load_trajectory_csv(ref_path)
        ego = load_ego_csv(Path(log_dir) / f"{tag}_ego.csv")

        # C++ plugin / MPPI seeder: x0 = [x,y,psi,v,a,delta]
        x0 = np.array(
            [
                float(x_r[0]),
                float(y_r[0]),
                float(psi_r[0]),
                max(0.0, float(v_r[0])),
                0.0,
                0.0,
            ],
            dtype=float,
        )
        z0 = float(z_r[0]) if len(z_r) else 0.0
        if use_ego_ic and ego is not None:
            x0 = np.array(
                [
                    float(ego.get("x", x0[0])),
                    float(ego.get("y", x0[1])),
                    float(ego.get("yaw", x0[2])),
                    float(ego.get("v", x0[3])),
                    float(ego.get("accel", ego.get("a", 0.0))),
                    float(ego.get("steer", 0.0)),
                ],
                dtype=float,
            )
            z0 = float(ego.get("z", z0))

        x_pts, y_pts, psi_pts, v_pts = self._pad(x_r, y_r, psi_r, self.n_horizon + 1, v=v_r)
        start_idx = closest_index(x0, x_pts, y_pts)

        w, w_e = self._scale_stage_and_terminal_w(
            self._q, self._r, self._qe, n_horizon=self.n_horizon, tf=self.tf
        )
        old_cwd = os.getcwd()
        os.chdir(self.build_dir)
        try:
            self._apply_cost_weights(self.solver, self.n_horizon, w, w_e)
            sol_x, sol_u, status = self._solve(
                self.solver,
                self.n_horizon,
                x0,
                x_pts,
                y_pts,
                psi_pts,
                v_pts,
                self.p_vec,
                self._ny,
                log_acados=True,
            )
        finally:
            os.chdir(old_cwd)

        n_out = int(sol_x.shape[0])
        t_out = np.arange(n_out, dtype=float) * self.dt
        a_out = np.zeros(n_out)
        d_out = np.zeros(n_out)
        a_out[:-1] = sol_u[:, 0]
        d_out[:-1] = sol_u[:, 1]
        if n_out > 1:
            a_out[-1] = a_out[-2]
            d_out[-1] = d_out[-2]
        metrics = tracking_metrics(sol_x, x_pts, y_pts, psi_pts, v_pts, start_idx, x0=x0)
        horizon = horizon_tracking_series(
            sol_x,
            sol_u,
            x_pts,
            y_pts,
            psi_pts,
            v_pts,
            start_idx,
            x0,
            dt=self.dt,
        )
        return {
            "frame_id": frame_id,
            "tag": tag,
            "status": int(status),
            "start_idx": int(start_idx),
            "x0": x0,
            "z0": z0,
            "t": t_out,
            "x": sol_x[:, 0].copy(),
            "y": sol_x[:, 1].copy(),
            "yaw": sol_x[:, 2].copy(),
            "v": sol_x[:, 3].copy(),
            "a": a_out,
            "steer": d_out,
            "a_state": sol_x[:, 4].copy(),
            "delta_state": sol_x[:, 5].copy(),
            "sol_u": sol_u,
            "sol_x": sol_x.copy(),
            "horizon": horizon,
            "ref_t": None,
            "ref_x": np.asarray(x_r, dtype=float),
            "ref_y": np.asarray(y_r, dtype=float),
            "ref_yaw": np.asarray(psi_r, dtype=float),
            "ref_v": np.asarray(v_r, dtype=float),
            "metrics": metrics,
            "weights": dict(self.weight_kv),
        }

    def write_result(
        self, out_dir: Path, result: dict[str, Any], *, copy_reference: bool, log_dir: Path
    ) -> None:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        tag = result["tag"]
        write_key_value_csv(out_dir / "cost_params.csv", result["weights"])
        write_trajectory_csv(
            out_dir / f"{tag}_optimized.csv",
            t=result["t"],
            x=result["x"],
            y=result["y"],
            z=self._np.full(len(result["x"]), float(result["z0"])),
            yaw=result["yaw"],
            v=result["v"],
            a=result["a"],
            steer=result["steer"],
        )
        with (out_dir / f"{tag}_controls.csv").open("w", newline="") as cf:
            cw = csv.writer(cf)
            cw.writerow(["t_idx", "accel_cmd", "steer_cmd"])
            sol_u = result["sol_u"]
            for i in range(sol_u.shape[0]):
                cw.writerow([i, f"{float(sol_u[i, 0]):.9f}", f"{float(sol_u[i, 1]):.9f}"])
        if copy_reference:
            ref_src = Path(log_dir) / f"{tag}_reference.csv"
            if ref_src.is_file():
                shutil.copy2(ref_src, out_dir / f"{tag}_reference.csv")
            ego_src = Path(log_dir) / f"{tag}_ego.csv"
            if ego_src.is_file():
                shutil.copy2(ego_src, out_dir / f"{tag}_ego.csv")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log_dir = args.log_dir.expanduser().resolve()
    out_dir = args.out_dir.expanduser().resolve()
    if not log_dir.is_dir():
        print(f"log-dir does not exist: {log_dir}", file=sys.stderr)
        return 1

    overrides: dict[str, float] = {}
    if args.params_yaml is not None:
        overrides.update(load_weights_yaml(args.params_yaml.expanduser().resolve()))
    for token in args.set:
        if "=" not in token:
            print(f"Bad --set (want key=value): {token}", file=sys.stderr)
            return 1
        k, v = token.split("=", 1)
        overrides[k.strip()] = float(v.strip())

    vehicle = load_key_value_csv(log_dir / "vehicle_params.csv")
    default_lf, default_lr = bicycle_lf_lr_from_vehicle_params(vehicle)
    tau_a, tau_d, max_steer_rate = actuator_params_from_vehicle(vehicle)
    lf = float(args.lf) if args.lf is not None else default_lf
    lr = float(args.lr) if args.lr is not None else default_lr
    use_ego_ic = not bool(args.use_reference_ic)

    build_dir = (
        args.build_dir.expanduser().resolve()
        if args.build_dir is not None
        else (out_dir / "_acados_build")
    )
    session = TemporalMptRetuneSession(
        n_horizon=int(args.N),
        dt=float(args.dt),
        lf=lf,
        lr=lr,
        tau_a=tau_a,
        tau_d=tau_d,
        max_steer_rate=max_steer_rate,
        build_dir=build_dir,
        generate=bool(args.generate),
        quiet_build=False,
    )
    weight_kv = session.set_weights(overrides)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_key_value_csv(out_dir / "cost_params.csv", weight_kv)
    print(
        "applied_weights "
        + " ".join(f"{k}={v:g}" for k, v in weight_kv.items())
        + f"  (N={session.n_horizon}, dt={session.dt}, lf={lf}, lr={lr}, "
        + f"tau_a={tau_a}, tau_d={tau_d}, ic={'ego' if use_ego_ic else 'reference[0]'})"
    )

    frame_ids = discover_log_frames(log_dir)
    if not frame_ids:
        print(f"No frames found in {log_dir}", file=sys.stderr)
        return 1

    summary_path = out_dir / "summary.csv"
    index_path = out_dir / "index.csv"
    with summary_path.open("w", newline="") as summary_f, index_path.open(
        "w", newline=""
    ) as index_f:
        summary_w = csv.writer(summary_f)
        index_w = csv.writer(index_f)
        summary_w.writerow(
            [
                "frame_id",
                "status",
                "start_idx",
                "n_reference",
                "n_optimized",
                "mean_e_xy",
                "max_e_xy",
                "mean_abs_psi_err",
                "max_abs_psi_err",
                "mean_abs_v_err",
                "max_abs_v_err",
            ]
        )
        index_w.writerow(
            [
                "frame_id",
                "stamp_sec",
                "stamp_nsec",
                "baseline_cost",
                "n_reference",
                "n_optimized",
                "crash_status",
            ]
        )

        processed = 0
        for frame_id in frame_ids:
            if args.frame is not None and frame_id != args.frame:
                continue
            result = session.solve_frame(log_dir, frame_id, use_ego_ic=use_ego_ic)
            session.write_result(
                out_dir, result, copy_reference=args.copy_reference, log_dir=log_dir
            )
            metrics = result["metrics"]
            tag = result["tag"]
            summary_w.writerow(
                [
                    frame_id,
                    result["status"],
                    result["start_idx"],
                    len(result["ref_x"]),
                    len(result["x"]),
                    f"{metrics['mean_e_xy']:.6f}",
                    f"{metrics['max_e_xy']:.6f}",
                    f"{metrics['mean_abs_psi_err']:.6f}",
                    f"{metrics['max_abs_psi_err']:.6f}",
                    f"{metrics['mean_abs_v_err']:.6f}",
                    f"{metrics['max_abs_v_err']:.6f}",
                ]
            )
            index_w.writerow(
                [
                    frame_id,
                    0,
                    0,
                    metrics["mean_e_xy"],
                    len(result["ref_x"]),
                    len(result["x"]),
                    result["status"],
                ]
            )
            processed += 1
            print(
                f"frame {tag}: status={result['status']} mean_e_xy={metrics['mean_e_xy']:.4f} "
                f"max_e_xy={metrics['max_e_xy']:.4f} mean_|e_psi|={metrics['mean_abs_psi_err']:.4f}"
            )

    print(f"Wrote {processed} frame(s) -> {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
