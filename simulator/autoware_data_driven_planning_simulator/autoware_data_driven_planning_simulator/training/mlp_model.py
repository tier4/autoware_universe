"""Small deterministic NumPy MLP trainer.

This is intentionally dependency-light so the first learned backend can run in CI without
PyTorch/ONNX. It supports direct next-state and residual state-delta targets.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from autoware_data_driven_planning_simulator.training.linear_model import FEATURES, TARGETS, _load_matrix


def _target_matrix(y: np.ndarray, x: np.ndarray, model_type: str) -> np.ndarray:
    if model_type == "mlp_direct":
        return y
    if model_type == "mlp_residual":
        state_columns = [FEATURES.index(name) for name in ["state_x", "state_y", "state_yaw", "state_vx", "state_vy", "state_wz"]]
        return y - x[:, state_columns]
    raise ValueError(f"unsupported MLP model type: {model_type}")


def train_mlp_model(
    dataset_csv: str,
    output_dir: str,
    model_type: str = "mlp_direct",
    hidden_size: int = 32,
    epochs: int = 200,
    learning_rate: float = 1.0e-2,
    seed: int = 42,
) -> dict:
    x, y = _load_matrix(dataset_csv)
    if x.size == 0:
        raise RuntimeError("empty dataset")

    target = _target_matrix(y, x, model_type)
    x_mean = x.mean(axis=0)
    x_std = x.std(axis=0)
    x_std[x_std < 1.0e-9] = 1.0
    y_mean = target.mean(axis=0)
    y_std = target.std(axis=0)
    y_std[y_std < 1.0e-9] = 1.0
    xn = (x - x_mean) / x_std
    yn = (target - y_mean) / y_std

    rng = np.random.default_rng(seed)
    w1 = rng.normal(0.0, 0.05, size=(xn.shape[1], hidden_size))
    b1 = np.zeros(hidden_size)
    w2 = rng.normal(0.0, 0.05, size=(hidden_size, yn.shape[1]))
    b2 = np.zeros(yn.shape[1])

    for _ in range(epochs):
        h = np.tanh(xn @ w1 + b1)
        pred = h @ w2 + b2
        error = (pred - yn) / max(len(xn), 1)
        grad_w2 = h.T @ error
        grad_b2 = error.sum(axis=0)
        grad_h = error @ w2.T * (1.0 - h * h)
        grad_w1 = xn.T @ grad_h
        grad_b1 = grad_h.sum(axis=0)
        w1 -= learning_rate * grad_w1
        b1 -= learning_rate * grad_b1
        w2 -= learning_rate * grad_w2
        b2 -= learning_rate * grad_b2

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    np.savez(
        output / "mlp_weights.npz",
        w1=w1,
        b1=b1,
        w2=w2,
        b2=b2,
        x_mean=x_mean,
        x_std=x_std,
        y_mean=y_mean,
        y_std=y_std,
    )
    np.savetxt(output / "mlp_w1.csv", w1, delimiter=",")
    np.savetxt(output / "mlp_b1.csv", b1[None, :], delimiter=",")
    np.savetxt(output / "mlp_w2.csv", w2, delimiter=",")
    np.savetxt(output / "mlp_b2.csv", b2[None, :], delimiter=",")
    np.savetxt(output / "mlp_x_mean.csv", x_mean[None, :], delimiter=",")
    np.savetxt(output / "mlp_x_std.csv", x_std[None, :], delimiter=",")
    np.savetxt(output / "mlp_y_mean.csv", y_mean[None, :], delimiter=",")
    np.savetxt(output / "mlp_y_std.csv", y_std[None, :], delimiter=",")
    artifact = {
        "backend": "numpy_mlp_cpu",
        "model_type": model_type,
        "features": FEATURES,
        "targets": TARGETS,
        "weights_npz": "mlp_weights.npz",
        "runtime_csv": {
            "w1": "mlp_w1.csv",
            "b1": "mlp_b1.csv",
            "w2": "mlp_w2.csv",
            "b2": "mlp_b2.csv",
            "x_mean": "mlp_x_mean.csv",
            "x_std": "mlp_x_std.csv",
            "y_mean": "mlp_y_mean.csv",
            "y_std": "mlp_y_std.csv",
        },
        "hidden_size": hidden_size,
        "epochs": epochs,
        "seed": seed,
        "determinism": {
            "provider": "numpy_cpu",
            "thread_count": 1,
            "fallback": "baseline",
        },
    }
    with (output / "model_artifact.yaml").open("w") as f:
        yaml.safe_dump(artifact, f, sort_keys=False)
    return artifact

