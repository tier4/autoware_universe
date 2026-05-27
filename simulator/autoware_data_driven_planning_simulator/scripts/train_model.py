#!/usr/bin/env python3
"""Train deterministic CPU-friendly direct/residual models."""

import argparse

import yaml

from autoware_data_driven_planning_simulator.training.linear_model import train_linear_model
from autoware_data_driven_planning_simulator.training.mlp_model import train_mlp_model


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-csv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--model-type",
        choices=["linear_direct", "mlp_direct", "mlp_residual", "sequence_model"],
        default="linear_direct",
    )
    parser.add_argument("--ridge", type=float, default=1.0e-6)
    parser.add_argument("--hidden-size", type=int, default=32)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--learning-rate", type=float, default=1.0e-2)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.model_type == "linear_direct":
        artifact = train_linear_model(args.dataset_csv, args.output, args.ridge)
    elif args.model_type in ("mlp_direct", "mlp_residual"):
        artifact = train_mlp_model(
            args.dataset_csv,
            args.output,
            model_type=args.model_type,
            hidden_size=args.hidden_size,
            epochs=args.epochs,
            learning_rate=args.learning_rate,
            seed=args.seed,
        )
    else:
        # Sequence models use the same deterministic MLP trainer initially, with history-window
        # features expected to be materialized by the extractor in a later iteration.
        artifact = train_mlp_model(
            args.dataset_csv,
            args.output,
            model_type="mlp_direct",
            hidden_size=args.hidden_size,
            epochs=args.epochs,
            learning_rate=args.learning_rate,
            seed=args.seed,
        )
        artifact["model_type"] = "sequence_model"
        with open(f"{args.output}/model_artifact.yaml", "w") as f:
            yaml.safe_dump(artifact, f, sort_keys=False)
    print(yaml.safe_dump(artifact, sort_keys=False))


if __name__ == "__main__":
    main()
