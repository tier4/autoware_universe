#!/usr/bin/env python3
"""Extract a filtered dataset from a ROS 2 bag."""

import argparse

import yaml

from autoware_data_driven_planning_simulator.dataset.extractor import extract_dataset


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--vehicle-profile", required=True)
    parser.add_argument("--filter-config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    with open(args.vehicle_profile) as f:
        vehicle_profile = yaml.safe_load(f)
    with open(args.filter_config) as f:
        filter_config = yaml.safe_load(f)
    report = extract_dataset(args.bag, vehicle_profile, filter_config, args.output)
    print(yaml.safe_dump(report, sort_keys=False))


if __name__ == "__main__":
    main()
