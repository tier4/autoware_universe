"""
---------------
Utility functions for stop distance evaluation in Autoware-Frenetix integration.

Implements jerk- and acceleration-constrained stopping distance estimation
based on Autoware's motion_utils logic. All equations are derived from the
TIER IV implementation in distance.cpp.

"""

import math
from typing import Optional, Tuple


# === Constant-jerk update primitive (x, v, a) -> (x_new, v_new, a_new) ===
def _cj_update(x: float, v: float, a: float, j: float, t: float) -> Tuple[float, float, float]:
    """Integrate position, velocity, acceleration under constant jerk for time t."""
    a_new = a + j * t
    v_new = v + a * t + 0.5 * j * t * t
    x_new = x + v * t + 0.5 * a * t * t + (1.0 / 6.0) * j * t * t * t
    return x_new, v_new, a_new


# === Validity check for end-state of braking plan ===
def _valid_check(v_end: float, a_end: float,
                 v_target: float, a_target: float,
                 v_margin: float = 0.3, a_margin: float = 0.1) -> bool:
    """Check whether (v_end, a_end) is within acceptable tolerances."""
    v_min, v_max = v_target - abs(v_margin), v_target + abs(v_margin)
    a_min, a_max = a_target - abs(a_margin), a_target + abs(a_margin)
    if not (v_min <= v_end <= v_max):
        return False
    if not (a_min <= a_end <= a_max):
        return False
    return True


# === Type 1: trapezoid acceleration (jerk down, hold, jerk up) ===
def _decel_type1(v0: float, vt: float, a0: float,
                 am: float, ja: float, jd: float, t_hold: float) -> Optional[float]:
    eps = 1e-3
    j1 = jd if am < a0 else ja
    t1 = (am - a0) / j1 if abs((am - a0) / j1) > eps else 0.0
    x1, v1, a1 = _cj_update(0.0, v0, a0, j1, t1)

    t2 = t_hold if t_hold > eps else 0.0
    x2, v2, a2 = _cj_update(x1, v1, a1, 0.0, t2)

    t3 = (0.0 - am) / ja if abs((0.0 - am) / ja) > eps else 0.0
    x3, v3, a3 = _cj_update(x2, v2, a2, ja, t3)

    if not _valid_check(v3, a3, vt, 0.0):
        return None
    return x3


# === Type 2: triangle acceleration (jerk down, jerk up, no hold) ===
def _decel_type2(v0: float, vt: float, a0: float, ja: float, jd: float) -> Optional[float]:
    eps = 1e-3
    a1_sq = (vt - v0 - 0.5 * (0.0 - a0) / jd * a0) * (2.0 * ja * jd / (ja - jd))
    if a1_sq <= 0.0:
        return None
    a1 = -math.sqrt(a1_sq)

    t1 = (a1 - a0) / jd if abs((a1 - a0) / jd) > eps else 0.0
    x1, v1, _ = _cj_update(0.0, v0, a0, jd, t1)

    t2 = (0.0 - a1) / ja if abs((0.0 - a1) / ja) > eps else 0.0
    x2, v2, a2 = _cj_update(x1, v1, a1, ja, t2)

    if not _valid_check(v2, a2, vt, 0.0):
        return None
    return x2


# === Type 3: single-phase jerk to zero acceleration ===
def _decel_type3(v0: float, vt: float, a0: float, ja: float) -> Optional[float]:
    eps = 1e-3
    t1 = (0.0 - a0) / ja
    t1 = t1 if t1 > eps else 0.0
    x1, v1, a1 = _cj_update(0.0, v0, a0, ja, t1)

    if not _valid_check(v1, a1, vt, 0.0):
        return None
    return x1


# === Public interface ===
def compute_min_stop_distance(
    v0: float,
    a0: float,
    acc_min: float = -3.0,
    jerk_acc: float = 1.0,
    jerk_dec: float = -1.0,
) -> Optional[float]:
    """
    Compute the minimal distance required to brake from v0 to 0.0 m/s
    using a jerk-limited deceleration profile.

    Args:
        v0: current velocity [m/s]
        a0: current acceleration [m/s²]
        acc_min: minimum (most negative) acceleration during braking [m/s²]
        jerk_acc: maximum positive jerk (relaxing decel) [m/s³]
        jerk_dec: minimum negative jerk (applying decel) [m/s³]

    Returns:
        The distance [m] required to stop under given limits,
        or None if no feasible profile exists.
    """
    if v0 < 0.2:
        return 0.0

    eps = 1e-3
    j_before = jerk_dec if acc_min < a0 else jerk_acc
    t_before = (acc_min - a0) / j_before

    j_after = jerk_acc
    t_after = (0.0 - acc_min) / j_after

    t_hold_num = (0.0 - v0
                  - a0 * t_before
                  - 0.5 * j_before * (t_before ** 2)
                  - acc_min * t_after
                  - 0.5 * j_after * (t_after ** 2))
    t_hold = t_hold_num / acc_min if abs(acc_min) > eps else 0.0

    is_decel_needed = 0.5 * (0.0 - a0) / jerk_acc * a0 > (0.0 - v0)

    if t_hold > eps:
        d1 = _decel_type1(v0, 0.0, a0, acc_min, jerk_acc, jerk_dec, t_hold)
        if d1 is not None:
            return d1

    if is_decel_needed or a0 > eps:
        d2 = _decel_type2(v0, 0.0, a0, jerk_acc, jerk_dec)
        if d2 is not None:
            return d2

    return _decel_type3(v0, 0.0, a0, jerk_acc)


# === Optional: simple CLI test ===
if __name__ == "__main__":
    v0_test = 20.0  # m/s
    a0_test = 0.0
    d = compute_min_stop_distance(v0_test, a0_test, acc_min=-3.0, jerk_acc=1.0, jerk_dec=-1.0)
    print(f"Estimated stop distance for v0={v0_test:.2f} m/s: {d:.2f} m" if d else "No valid plan.")
