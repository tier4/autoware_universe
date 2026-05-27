"""Deterministic 2D baseline models used by extraction, training, and evaluation."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, tan, atan2


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    steer: float = 0.0
    ax: float = 0.0


@dataclass
class Command:
    velocity: float = 0.0
    acceleration: float = 0.0
    steer: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def wrap_angle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def integrate_constant_body_twist(state: State, vx: float, vy: float, wz: float, dt: float) -> State:
    next_state = State(**state.__dict__)
    yaw0 = state.yaw
    if abs(wz) < 1.0e-8:
        next_state.x += (cos(yaw0) * vx - sin(yaw0) * vy) * dt
        next_state.y += (sin(yaw0) * vx + cos(yaw0) * vy) * dt
    else:
        yaw1 = yaw0 + wz * dt
        next_state.x += vx / wz * (sin(yaw1) - sin(yaw0)) + vy / wz * (
            cos(yaw1) - cos(yaw0)
        )
        next_state.y += vx / wz * (-cos(yaw1) + cos(yaw0)) + vy / wz * (
            sin(yaw1) - sin(yaw0)
        )
    next_state.yaw = wrap_angle(yaw0 + wz * dt)
    next_state.vx = vx
    next_state.vy = vy
    next_state.wz = wz
    return next_state


def step_ackermann(state: State, command: Command, dt: float, wheelbase: float = 2.7) -> State:
    vx = command.velocity if abs(command.velocity) > 1.0e-6 else state.vx + command.acceleration * dt
    wz = vx * tan(command.steer) / max(wheelbase, 1.0e-6)
    next_state = integrate_constant_body_twist(state, vx, 0.0, wz, dt)
    next_state.steer = command.steer
    next_state.ax = (vx - state.vx) / dt if dt > 0.0 else 0.0
    return next_state


def step_differential(state: State, command: Command, dt: float) -> State:
    return integrate_constant_body_twist(state, command.vx, 0.0, command.wz, dt)


def step_holonomic(state: State, command: Command, dt: float) -> State:
    return integrate_constant_body_twist(state, command.vx, command.vy, command.wz, dt)


def step_baseline(state: State, command: Command, dt: float, model_type: str, wheelbase: float = 2.7) -> State:
    if model_type == "ackermann":
        return step_ackermann(state, command, dt, wheelbase)
    if model_type in ("differential", "skid_steer"):
        return step_differential(state, command, dt)
    if model_type in ("holonomic", "omni", "mecanum"):
        return step_holonomic(state, command, dt)
    raise ValueError(f"unsupported baseline model type: {model_type}")

