# autoware_data_driven_planning_simulator

This package provides the first implementation slice of the data-driven 2D simulator described in
`../data_driven_special_vehicle_simulator_package_ja.md`.

## Scope

- Extract synchronized datasets from ROS 2 bags (`sqlite3` and `mcap` through `rosbag2_py`).
- Validate maneuver coverage before training.
- Run deterministic 2D baseline models for Ackermann, differential/skid-steer, and holonomic vehicles.
- Train/export a deterministic CPU linear next-state artifact as the initial learned backend.
- Evaluate one-step and short-horizon-ready metrics on extracted datasets.

The package intentionally starts with deterministic CPU-friendly components. Advanced Koopman, GP residual,
and physics-informed backends are later extensions.

## Runtime Node

```bash
ros2 launch autoware_data_driven_planning_simulator data_driven_planning_simulator.launch.py
```

The node publishes:

- `output/odometry`
- `output/twist`
- `output/steering`
- `/tf` as `odom -> base_link` by default

It subscribes to either:

- `input/ackermann_control_command`
- `input/cmd_vel`

depending on `command_type`.

## Dataset Extraction

```bash
ros2 run autoware_data_driven_planning_simulator extract_dataset.py \
  --bag path/to/bag \
  --vehicle-profile path/to/vehicle_profile.yaml \
  --filter-config path/to/dataset_filter.yaml \
  --output /tmp/ddsim_dataset
```

Outputs include:

- `dataset.csv`
- `train.csv`, `validation.csv`, `test.csv`
- `dataset.npz`
- `metadata.yaml`
- `data_validation_report.yaml`

## Training

```bash
ros2 run autoware_data_driven_planning_simulator train_model.py \
  --dataset-csv /tmp/ddsim_dataset/train.csv \
  --output /tmp/ddsim_model
```

The initial artifact is a deterministic linear CPU direct next-state model:

- `model_artifact.yaml`
- `normalization.yaml`
- `linear_weights.csv`

## Evaluation

```bash
ros2 run autoware_data_driven_planning_simulator evaluate_model.py \
  --dataset-csv /tmp/ddsim_dataset/test.csv \
  --output /tmp/ddsim_eval
```

The evaluator writes `evaluation_report.yaml`.
