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

r"""Interactive temporal-MPT weight tuner for MPPI ``u_nom`` (matplotlib UI).

Online MPPI seeds ``u_nom`` from acados temporal MPT when ``use_temporal_mpt_as_nominal``
is enabled (``TemporalMptNominalSeeder``: ``x0=ego``,
``lr=ego_axle_to_box_center``, ``lf=wheel_base-lr``).

This tool:
  * plots **logged RViz orange** from ``*_nominal_traj.csv`` (open-loop ``u_nom`` path =
    ``mppi_nominal`` marker); falls back to ``*_optimized.csv`` on older logs
  * regenerates t-MPT with the same IC / ``lf,lr`` and optional weight overrides
  * overlays **retuned** acados ``x*`` for weight comparison

Example::

  ros2 run autoware_trajectory_processor temporal_mpt_offline_tuner.py -- \\
    --log-dir "$HOME/.cache/autoware/mppi_debug_log" \\
    --start-frame 0
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys
import tempfile
from typing import Any
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import Slider
import numpy as np

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from temporal_mpt_offline_retune import TemporalMptRetuneSession  # noqa: E402
from temporal_mpt_offline_retune import actuator_params_from_vehicle  # noqa: E402
from temporal_mpt_offline_retune import bicycle_lf_lr_from_vehicle_params  # noqa: E402
from temporal_mpt_offline_retune import discover_log_frames  # noqa: E402
from temporal_mpt_offline_retune import load_ego_csv  # noqa: E402
from temporal_mpt_offline_retune import load_key_value_csv  # noqa: E402
from temporal_mpt_offline_retune import load_weights_yaml  # noqa: E402

# Path-frame LINEAR_LS: qlong=along-track, qlat=cross-track (see apply_path_frame_stage).
DEFAULT_WEIGHTS: Dict[str, float] = {
    "qlong": 0.5,
    "qlat": 5.0,
    "qpsi": 0.05,
    "qv": 0.05,
    "ra": 0.025,
    "rdelta": 2.0,
    "qe_scale": 2.5,
}

SLIDER_SPECS: List[Tuple[str, float, float]] = [
    ("qlong", 0.0, 20.0),
    ("qlat", 0.0, 50.0),
    ("qpsi", 0.0, 50.0),
    ("qv", 0.0, 20.0),
    ("ra", 0.0, 5.0),
    ("rdelta", 0.0, 20.0),
    ("qe_scale", 0.1, 20.0),
]

MPPI_DT = 0.1


def _normalize_angle(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def _load_traj_with_steer(path: Path) -> Optional[Dict[str, Any]]:
    if not path.is_file():
        return None
    rows: List[List[float]] = []
    with path.open(newline="") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row:
                continue
            vals = [float(c) for c in row]
            if len(vals) < 6:
                continue
            rows.append(vals)
    if len(rows) < 2:
        return None
    arr = np.asarray(rows, dtype=float)
    return {
        "t": arr[:, 0],
        "x": arr[:, 1],
        "y": arr[:, 2],
        "yaw": arr[:, 4] if arr.shape[1] > 4 else np.zeros(len(rows)),
        "v": arr[:, 5] if arr.shape[1] > 5 else np.zeros(len(rows)),
        "a": arr[:, 6] if arr.shape[1] > 6 else np.zeros(len(rows)),
        "steer": arr[:, 7] if arr.shape[1] > 7 else np.zeros(len(rows)),
    }


def load_nominal_cmds(path: Path) -> Optional[Dict[str, np.ndarray]]:
    if not path.is_file():
        return None
    accel: List[float] = []
    steer: List[float] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            accel.append(float(row["accel_cmd"]))
            steer.append(float(row["steer_cmd"]))
    if not accel:
        return None
    return {"accel": np.asarray(accel, dtype=float), "steer": np.asarray(steer, dtype=float)}


def rollout_first_order_dubins(
    ego: Dict[str, float],
    accel_cmd: np.ndarray,
    steer_cmd: np.ndarray,
    *,
    wheel_base: float,
    accel_time_constant: float,
    steer_time_constant: float,
    max_steer_angle: float,
    max_steer_rate: float,
    min_accel: float,
    max_accel: float,
    dt: float = MPPI_DT,
) -> Dict[str, np.ndarray]:
    """Open-loop FirstOrderDubinsBicycle rollout (same plant as mppi_debug_visualizer)."""
    n = min(len(accel_cmd), len(steer_cmd))
    x = float(ego["x"])
    y = float(ego["y"])
    yaw = float(ego["yaw"])
    v = float(ego["v"])
    accel = float(ego.get("accel", 0.0))
    steer = float(ego.get("steer", 0.0))
    accel_tau = max(float(accel_time_constant), 1.0e-4)
    steer_tau = max(float(steer_time_constant), 1.0e-4)
    wb = max(float(wheel_base), 1.0e-6)
    max_delta = float(max_steer_angle)
    max_rate = float(max_steer_rate)

    xs: List[float] = []
    ys: List[float] = []
    yaws: List[float] = []
    vs: List[float] = []
    for i in range(n):
        a_cmd = float(accel_cmd[i])
        d_cmd = float(steer_cmd[i])
        accel_dot = (a_cmd - accel) / accel_tau
        steer_dot = (d_cmd - steer) / steer_tau
        steer_dot = max(-max_rate, min(max_rate, steer_dot))
        yaw_dot = (v / wb) * math.tan(steer)
        x = x + v * math.cos(yaw) * dt
        y = y + v * math.sin(yaw) * dt
        yaw = _normalize_angle(yaw + yaw_dot * dt)
        v = v + accel * dt
        steer = max(-max_delta, min(max_delta, steer + steer_dot * dt))
        accel = max(min_accel, min(max_accel, accel + accel_dot * dt))
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
        vs.append(v)

    t = np.arange(n, dtype=float) * dt
    return {
        "t": t,
        "x": np.asarray(xs),
        "y": np.asarray(ys),
        "yaw": np.asarray(yaws),
        "v": np.asarray(vs),
        "a": np.asarray(accel_cmd[:n], dtype=float),
        "steer": np.asarray(steer_cmd[:n], dtype=float),
    }


class TemporalMptOfflineTuner:
    def __init__(
        self,
        log_dir: Path,
        *,
        start_frame: int = 0,
        n_horizon: int = 80,
        dt: float = 0.1,
        params_yaml: Optional[Path] = None,
    ) -> None:
        self.log_dir = log_dir.expanduser().resolve()
        self.frame_ids = discover_log_frames(self.log_dir)
        if not self.frame_ids:
            raise FileNotFoundError(f"No frames in {self.log_dir}")
        self.index = 0
        if start_frame in self.frame_ids:
            self.index = self.frame_ids.index(start_frame)

        vehicle = load_key_value_csv(self.log_dir / "vehicle_params.csv")
        self.wheel_base = float(vehicle.get("wheel_base", 4.76))
        self.lf, self.lr = bicycle_lf_lr_from_vehicle_params(vehicle)
        self.accel_tau, self.steer_tau, self.max_steer_rate = actuator_params_from_vehicle(vehicle)
        self.max_steer = float(vehicle.get("max_steer_angle", 0.7))
        # Prefer steer_rate_lim from vehicle if actuator helper used defaults only when missing
        if "steer_rate_lim" in vehicle or "max_steer_rate" in vehicle:
            self.max_steer_rate = float(
                vehicle.get("steer_rate_lim", vehicle.get("max_steer_rate", self.max_steer_rate))
            )
        self.min_accel = float(vehicle.get("min_accel", -float(vehicle.get("vel_rate_lim", 5.0))))
        self.max_accel = float(vehicle.get("max_accel", float(vehicle.get("vel_rate_lim", 3.0))))

        runtime = load_key_value_csv(self.log_dir / "runtime_options.csv")
        self.logged_used_tmpt = bool(int(runtime.get("use_temporal_mpt_as_nominal", 0)))

        self.status = "Building acados solver…"
        self.logged_online: Optional[Dict[str, Any]] = None  # *_optimized.csv (RViz online path)
        self.logged_nominal: Optional[Dict[str, Any]] = None  # u_nom cmds (optional overlay)
        self.retuned_rollout: Optional[Dict[str, Any]] = None
        self.retuned_solve: Optional[Dict[str, Any]] = None
        self.out_dir = Path(tempfile.mkdtemp(prefix="temporal_mpt_retune_"))

        self.weights = dict(DEFAULT_WEIGHTS)
        if params_yaml is not None:
            self.weights.update(
                {k: v for k, v in load_weights_yaml(params_yaml).items() if k in DEFAULT_WEIGHTS}
            )

        print(
            f"Loading temporal MPT solver (N={n_horizon}, dt={dt}, "
            f"lf={self.lf:.4f}, lr={self.lr:.4f}, tau_a={self.accel_tau:.3f}, "
            f"tau_d={self.steer_tau:.3f}, IC=ego)…"
        )
        self.session = TemporalMptRetuneSession(
            n_horizon=n_horizon,
            dt=dt,
            lf=self.lf,
            lr=self.lr,
            tau_a=self.accel_tau,
            tau_d=self.steer_tau,
            max_steer_rate=self.max_steer_rate,
            quiet_build=True,
            generate=True,
        )
        self.dt = dt

        self.fig = plt.figure(figsize=(14.5, 8.5), constrained_layout=False)
        self.fig.canvas.manager.set_window_title("Temporal MPT Offline Retune (MPPI u_nom)")
        gs = self.fig.add_gridspec(
            4,
            1,
            left=0.06,
            right=0.68,
            top=0.92,
            bottom=0.08,
            hspace=0.28,
            height_ratios=[2.4, 1.0, 1.0, 1.0],
        )
        self.ax_xy = self.fig.add_subplot(gs[0])
        self.ax_yaw = self.fig.add_subplot(gs[1])
        self.ax_v = self.fig.add_subplot(gs[2])
        self.ax_u = self.fig.add_subplot(gs[3])
        self.status_artist = self.fig.text(0.06, 0.965, self.status, fontsize=9, va="top")

        self.sliders: Dict[str, Slider] = {}
        n = len(SLIDER_SPECS)
        slider_h = min(0.045, 0.70 / n - 0.008)
        gap = 0.012
        top = 0.90
        for i, (name, vmin, vmax) in enumerate(SLIDER_SPECS):
            ax = self.fig.add_axes([0.72, top - i * (slider_h + gap), 0.25, slider_h])
            valinit = float(self.weights.get(name, DEFAULT_WEIGHTS[name]))
            lo = min(vmin, valinit)
            hi = max(vmax, valinit)
            if hi <= lo:
                hi = lo + 1.0e-6
            self.sliders[name] = Slider(ax, name, lo, hi, valinit=valinit, valfmt="%0.3g")

        ax_prev = self.fig.add_axes([0.72, 0.06, 0.07, 0.045])
        ax_next = self.fig.add_axes([0.80, 0.06, 0.07, 0.045])
        ax_run = self.fig.add_axes([0.88, 0.06, 0.09, 0.045])
        self.btn_prev = Button(ax_prev, "Prev")
        self.btn_next = Button(ax_next, "Next")
        self.btn_run = Button(ax_run, "Retune")
        self.btn_prev.on_clicked(lambda _e: self._step(-1))
        self.btn_next.on_clicked(lambda _e: self._step(1))
        self.btn_run.on_clicked(lambda _e: self._retune())
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        flag = "ON" if self.logged_used_tmpt else "OFF (log flag)"
        print(
            f"Temporal MPT tuner: {self.log_dir} ({len(self.frame_ids)} frames).\n"
            f"Parity: x0=ego, lf={self.lf:.4f}m lr={self.lr:.4f}m "
            f"(lr=ego_axle_to_box_center; use_temporal_mpt_as_nominal logged={flag}).\n"
            "Plots: reference (black) | logged RViz orange (*_nominal_traj / *_optimized) (cyan) | "
            "retuned acados x* (orange).\n"
            "Cyan reads the logged state trajectory from disk (same path as RViz orange "
            "mppi_nominal when *_nominal_traj.csv exists).\n"
            "Keys: ←/→ step, r retune, d reset defaults, q quit."
        )
        self._load_frame(solve_retune=True)

    def _current_weights(self) -> Dict[str, float]:
        return {name: float(slider.val) for name, slider in self.sliders.items()}

    def _frame_id(self) -> int:
        return self.frame_ids[self.index]

    def _ego_for_frame(self, frame_id: int) -> Optional[Dict[str, float]]:
        return load_ego_csv(self.log_dir / f"{frame_id:06d}_ego.csv")

    def _rollout_cmds(
        self, ego: Dict[str, float], accel: np.ndarray, steer: np.ndarray
    ) -> Dict[str, Any]:
        return rollout_first_order_dubins(
            ego,
            accel,
            steer,
            wheel_base=self.wheel_base,
            accel_time_constant=self.accel_tau,
            steer_time_constant=self.steer_tau,
            max_steer_angle=self.max_steer,
            max_steer_rate=self.max_steer_rate,
            min_accel=self.min_accel,
            max_accel=self.max_accel,
            dt=self.dt,
        )

    def _load_logged_online(self, frame_id: int) -> Optional[Dict[str, Any]]:
        """Load logged RViz orange path from nominal_traj or optimized CSV."""
        tag = f"{frame_id:06d}"
        traj = _load_traj_with_steer(self.log_dir / f"{tag}_nominal_traj.csv")
        source = "nominal_traj"
        if traj is None:
            traj = _load_traj_with_steer(self.log_dir / f"{tag}_optimized.csv")
            source = "optimized"
        if traj is None:
            return None
        n = len(traj["x"])
        traj["t"] = np.arange(n, dtype=float) * self.dt
        traj["source"] = source
        ego = self._ego_for_frame(frame_id)
        if ego is not None:
            traj["ego"] = ego
        return traj

    def _load_logged_nominal(self, frame_id: int) -> Optional[Dict[str, Any]]:
        ego = self._ego_for_frame(frame_id)
        cmds = load_nominal_cmds(self.log_dir / f"{frame_id:06d}_nominal.csv")
        if ego is None or cmds is None:
            return None
        out = self._rollout_cmds(ego, cmds["accel"], cmds["steer"])
        out["ego"] = ego
        out["accel_cmd"] = cmds["accel"]
        out["steer_cmd"] = cmds["steer"]
        return out

    def _solve_retuned(self, weights: Dict[str, float]) -> Dict[str, Any]:
        self.session.set_weights(weights)
        # MPPI TemporalMptNominalSeeder parity: ego IC + lf/lr already set on session.
        return self.session.solve_frame(self.log_dir, self._frame_id(), use_ego_ic=True)

    def _update_status(self) -> None:
        frame_id = self._frame_id()
        parts = [f"frame {frame_id}"]
        if self.logged_online is None:
            parts.append("NO *_nominal_traj.csv / *_optimized.csv")
        else:
            parts.append(f"logged={self.logged_online.get('source', '?')}")
        if self.retuned_solve is not None:
            parts.append(f"status={self.retuned_solve['status']}")
            sol = self.retuned_solve
            if "x" in sol and "ref_x" in sol:
                nxy = min(len(sol["x"]), len(sol["ref_x"]))
                if nxy > 0:
                    dxy = float(
                        np.max(
                            np.hypot(
                                sol["x"][:nxy] - sol["ref_x"][:nxy],
                                sol["y"][:nxy] - sol["ref_y"][:nxy],
                            )
                        )
                    )
                    tip = float(
                        np.hypot(
                            sol["x"][nxy - 1] - sol["ref_x"][nxy - 1],
                            sol["y"][nxy - 1] - sol["ref_y"][nxy - 1],
                        )
                    )
                    parts.append(f"acados max|Δxy|={dxy:.3f}m tip={tip:.3f}m")
            parts.append(f"qlat={self.weights.get('qlat', 0):.3g}")
            parts.append(f"qψ={self.weights.get('qpsi', 0):.3g}")
            parts.append(f"Rδ={self.weights.get('rdelta', 0):.3g}")
        if self.logged_online is not None and self.retuned_solve is not None:
            nxy = min(len(self.logged_online["x"]), len(self.retuned_solve["x"]))
            if nxy > 0:
                dxy = float(
                    np.max(
                        np.hypot(
                            self.retuned_solve["x"][:nxy] - self.logged_online["x"][:nxy],
                            self.retuned_solve["y"][:nxy] - self.logged_online["y"][:nxy],
                        )
                    )
                )
                tip = float(
                    np.hypot(
                        self.retuned_solve["x"][nxy - 1] - self.logged_online["x"][nxy - 1],
                        self.retuned_solve["y"][nxy - 1] - self.logged_online["y"][nxy - 1],
                    )
                )
                parts.append(f"vs logged online max|Δxy|={dxy:.3f}m tip={tip:.3f}m")
        self.status = "  ".join(parts)

    def _load_frame(self, *, solve_retune: bool) -> None:
        frame_id = self._frame_id()
        self.logged_online = self._load_logged_online(frame_id)
        self.logged_nominal = self._load_logged_nominal(frame_id)
        self.retuned_solve = None
        self.retuned_rollout = None
        if solve_retune:
            self.status = f"Solving t-MPT for frame {frame_id}…"
            self._redraw(busy=True)
            self.fig.canvas.flush_events()
            try:
                self.weights = self._current_weights()
                self.retuned_solve = self._solve_retuned(self.weights)
                ego = self._ego_for_frame(frame_id)
                if ego is not None:
                    self.retuned_rollout = self._rollout_cmds(
                        ego,
                        self.retuned_solve["sol_u"][:, 0],
                        self.retuned_solve["sol_u"][:, 1],
                    )
                self.session.write_result(
                    self.out_dir,
                    self.retuned_solve,
                    copy_reference=True,
                    log_dir=self.log_dir,
                )
                self._update_status()
            except Exception as exc:  # noqa: BLE001
                self.status = f"Solve failed: {exc}"
                print(self.status, file=sys.stderr)
        else:
            self._update_status()
        self._redraw()

    def _step(self, delta: int) -> None:
        self.index = max(0, min(len(self.frame_ids) - 1, self.index + delta))
        self._load_frame(solve_retune=True)

    def _retune(self) -> None:
        self._load_frame(solve_retune=True)

    def _reset_defaults(self) -> None:
        for name, slider in self.sliders.items():
            slider.set_val(float(DEFAULT_WEIGHTS[name]))
        self.weights = dict(DEFAULT_WEIGHTS)
        self._retune()

    def _redraw(self, *, busy: bool = False) -> None:
        frame_id = self._frame_id()
        tag = f"{frame_id:06d}"
        ref = _load_traj_with_steer(self.log_dir / f"{tag}_reference.csv")
        logged = self.logged_online
        sol = self.retuned_solve
        roll = self.retuned_rollout

        for ax in (self.ax_xy, self.ax_yaw, self.ax_v, self.ax_u):
            ax.clear()

        if ref is not None:
            t_ref = np.arange(len(ref["x"]), dtype=float) * self.dt
            self.ax_xy.plot(ref["x"], ref["y"], "k-", lw=2.0, label="reference (DP)", zorder=2)
            self.ax_yaw.plot(t_ref, ref["yaw"], "k-", lw=1.4, label="ref ψ")
            self.ax_v.plot(t_ref, ref["v"], "k-", lw=1.4, label="ref v")

        # Cyan = logged RViz orange path from disk (nominal_traj preferred, else optimized).
        if logged is not None:
            src = logged.get("source", "online")
            self.ax_xy.plot(
                logged["x"],
                logged["y"],
                "C0-",
                lw=2.2,
                label=f"logged online (*_{src})",
                zorder=4,
            )
            self.ax_yaw.plot(logged["t"], logged["yaw"], "C0-", lw=1.6, label="logged ψ")
            self.ax_v.plot(logged["t"], logged["v"], "C0-", lw=1.6, label="logged v")
            self.ax_u.plot(logged["t"], logged["a"], "C0-", lw=1.5, label="logged a")
            self.ax_u.plot(logged["t"], logged["steer"], "C0--", lw=1.5, label="logged δ")
            ego = logged.get("ego")
            if ego is not None:
                self.ax_xy.plot(ego["x"], ego["y"], "C0o", ms=7, zorder=6, label="ego")

        # Primary retune view: acados optimized state.
        if sol is not None:
            self.ax_xy.plot(sol["x"], sol["y"], "C1-", lw=2.4, label="retuned acados x*", zorder=5)
            self.ax_yaw.plot(sol["t"], sol["yaw"], "C1-", lw=1.8, label="acados ψ*")
            self.ax_v.plot(sol["t"], sol["v"], "C1-", lw=1.8, label="acados v*")
            self.ax_u.plot(sol["t"], sol["a"], "C1-", lw=1.5, label="acados a*")
            self.ax_u.plot(sol["t"], sol["steer"], "C3-", lw=1.5, label="acados δ*")

        if roll is not None:
            self.ax_xy.plot(
                roll["x"], roll["y"], "C1--", lw=1.6, alpha=0.7, label="u* Dubins rollout", zorder=3
            )
            self.ax_yaw.plot(roll["t"], roll["yaw"], "C1--", lw=1.2, alpha=0.7, label="rollout ψ")
            self.ax_v.plot(roll["t"], roll["v"], "C1--", lw=1.2, alpha=0.7, label="rollout v")

        self.ax_xy.set_aspect("equal", adjustable="datalim")
        self.ax_xy.set_title(f"Frame {frame_id} / {self.frame_ids[-1]}")
        self.ax_xy.set_xlabel("x [m]")
        self.ax_xy.set_ylabel("y [m]")
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.legend(loc="best", fontsize=8)

        self.ax_yaw.set_ylabel("yaw [rad]")
        self.ax_yaw.grid(True, alpha=0.3)
        self.ax_yaw.legend(loc="best", fontsize=8)

        self.ax_v.set_ylabel("v [m/s]")
        self.ax_v.grid(True, alpha=0.3)
        self.ax_v.legend(loc="best", fontsize=8)

        self.ax_u.set_ylabel("a / δ")
        self.ax_u.set_xlabel("t [s]")
        self.ax_u.grid(True, alpha=0.3)
        self.ax_u.legend(loc="best", fontsize=8)

        self.status_artist.set_text(self.status + (" …" if busy else ""))
        self.fig.canvas.draw_idle()

    def _on_key(self, event) -> None:
        if event.key in ("right", "n"):
            self._step(1)
        elif event.key in ("left", "p"):
            self._step(-1)
        elif event.key == "home":
            self.index = 0
            self._load_frame(solve_retune=True)
        elif event.key == "end":
            self.index = len(self.frame_ids) - 1
            self._load_frame(solve_retune=True)
        elif event.key == "r":
            self._retune()
        elif event.key == "d":
            self._reset_defaults()
        elif event.key == "q":
            plt.close(self.fig)

    def spin(self) -> None:
        plt.show()


def parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Retune temporal-MPT weights against logged MPPI u_nom (*_nominal.csv).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--log-dir", type=Path, required=True)
    p.add_argument("--start-frame", type=int, default=0)
    p.add_argument("--N", type=int, default=80)
    p.add_argument("--dt", type=float, default=0.1)
    p.add_argument("--params-yaml", type=Path, default=None)
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    raw = [a for a in (sys.argv[1:] if argv is None else argv) if a != "--"]
    args = parse_args(raw)
    log_dir = args.log_dir.expanduser().resolve()
    if not log_dir.is_dir():
        print(f"log-dir does not exist: {log_dir}", file=sys.stderr)
        return 1
    TemporalMptOfflineTuner(
        log_dir,
        start_frame=args.start_frame,
        n_horizon=args.N,
        dt=args.dt,
        params_yaml=args.params_yaml,
    ).spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
