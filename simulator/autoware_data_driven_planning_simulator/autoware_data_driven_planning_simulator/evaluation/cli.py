"""CLI helpers for offline evaluation."""

from __future__ import annotations

import argparse
from math import isfinite


def parse_horizons(value: str) -> list[float]:
    try:
        horizons = [float(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as error:
        raise argparse.ArgumentTypeError("--horizons-s must contain only numeric values") from error
    if not horizons:
        raise argparse.ArgumentTypeError("--horizons-s must contain at least one horizon")
    if any(not isfinite(horizon) or horizon <= 0.0 for horizon in horizons):
        raise argparse.ArgumentTypeError("--horizons-s values must be finite and greater than zero")
    return horizons

