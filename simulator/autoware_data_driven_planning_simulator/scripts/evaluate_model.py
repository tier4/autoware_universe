#!/usr/bin/env python3
"""Evaluate baseline or learned artifacts on an extracted dataset."""

import argparse

import yaml

from autoware_data_driven_planning_simulator.evaluation.cli import parse_horizons
from autoware_data_driven_planning_simulator.evaluation.evaluator import evaluate_dataset


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-csv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--vehicle-profile")
    parser.add_argument("--model-artifact")
    parser.add_argument("--sampling-period-s", type=float, default=0.1)
    parser.add_argument(
        "--horizons-s",
        type=parse_horizons,
        default=parse_horizons("1.0,3.0,5.0,10.0"),
        help="Comma-separated rollout horizons in seconds.",
    )
    args = parser.parse_args()
    vehicle_profile = None
    if args.vehicle_profile:
        with open(args.vehicle_profile) as f:
            vehicle_profile = yaml.safe_load(f)
    report = evaluate_dataset(
        args.dataset_csv,
        args.output,
        vehicle_profile=vehicle_profile,
        model_artifact=args.model_artifact,
        sampling_period_s=args.sampling_period_s,
        horizons_s=args.horizons_s,
    )
    print(yaml.safe_dump(report, sort_keys=False))


if __name__ == "__main__":
    main()
