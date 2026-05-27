#!/usr/bin/env python3
"""Export model artifact metadata for runtime use."""

import argparse

import yaml

from autoware_data_driven_planning_simulator.training.linear_model import export_metadata


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    artifact = export_metadata(args.model_dir, args.output)
    print(yaml.safe_dump(artifact, sort_keys=False))


if __name__ == "__main__":
    main()
