import os
import json
import shutil
import numpy as np
from dataclasses import asdict

class TrajectoryLogger:
    """
    Logs trajectory data for offline analysis.
    Supports two modes:
    1. 'trajectories': Logs the final computed trajectories (lightweight).
    2. 'recompute': Logs the inputs to the planner to allow for recalculation later (complete).
    """
    def __init__(self, save_dir, mode='trajectories'):
        """
        Initializes the logger.

        Args:
            save_dir (str): The absolute path to the directory where data should be saved.
            mode (str): The logging mode. Either 'trajectories' or 'recompute'.
        """
        if mode not in ['trajectories', 'recompute']:
            raise ValueError("Mode must be either 'trajectories' or 'recompute'")
            
        self.save_dir = save_dir
        self.mode = mode
        self.log_counter = 0

        # Clear the directory on initialization to ensure a clean slate
        if os.path.exists(self.save_dir):
            shutil.rmtree(self.save_dir)
        os.makedirs(self.save_dir, exist_ok=True)
        print(f"Logger initialized in '{self.mode}' mode. Saving data to: {self.save_dir}")

    def save_debug_data(self, **kwargs):
        """
        Saves the debug data according to the selected mode.
        
        For 'trajectories' mode, expects:
            - optimal_trajectory (Trajectory)
            - feasible_trajectories (list of Trajectory)
            - reference_path (np.ndarray)
            - obstacle_positions (np.ndarray, optional)

        For 'recompute' mode, expects:
            - cartesian_state (CartesianState): Current cartesian planner state.
            - curvilinear_state (CurvilinearState): Current curvilinear planner state.
            - reference_path (np.ndarray)
            - obstacle_positions (np.ndarray, optional)
        """
        if self.mode == 'trajectories':
            self._save_trajectories_data(**kwargs)
        elif self.mode == 'recompute':
            self._save_recompute_data(**kwargs)

        self.log_counter += 1

    def _save_trajectories_data(self, optimal_trajectory, feasible_trajectories, infeasible_trajectories, reference_path, obstacle_positions=None, **kwargs):
        """Saves pre-computed trajectory data."""
        data_to_save = {
            "data_mode": "trajectories",
            "optimal_trajectory": self._serialize_trajectory(optimal_trajectory),
            "feasible_trajectories": [self._serialize_trajectory(t) for t in feasible_trajectories[:]],
            "infeasible_trajectories": [self._serialize_trajectory(t) for t in infeasible_trajectories[:100]],
            "reference_path": reference_path.tolist() if reference_path is not None else [],
            "obstacle_positions": obstacle_positions.tolist() if obstacle_positions is not None else []
        }
        filepath = os.path.join(self.save_dir, f"plot_data_{self.log_counter:05d}.json")
        with open(filepath, 'w') as f:
            json.dump(data_to_save, f)

    def _save_recompute_data(self, cartesian_state, curvilinear_state, reference_path, obstacle_positions=None, **kwargs):
        """Saves planner inputs for offline recalculation."""
        data_to_save = {
            "data_mode": "recompute",
            "cartesian_state": self._serialize_state(cartesian_state),
            "curvilinear_state": self._serialize_state(curvilinear_state),
            "reference_path": reference_path.tolist() if reference_path is not None else [],
            "obstacle_positions": obstacle_positions.tolist() if obstacle_positions is not None else []
        }
        filepath = os.path.join(self.save_dir, f"plot_data_{self.log_counter:05d}.json")
        with open(filepath, 'w') as f:
            json.dump(data_to_save, f)

    def _serialize_trajectory(self, trajectory):
        """Converts a Trajectory object into a serializable dictionary."""
        if trajectory is None:
            return None
        return {
            'cartesian': {
                'x': list(trajectory.cartesian.x),
                'y': list(trajectory.cartesian.y)
            },
            'costMap': trajectory.costMap
        }

    def _serialize_state(self, state):
        """Converts a dataclass State object into a serializable dictionary."""
        if state is None:
            return None
        return asdict(state)
