import json
import os
import numpy as np
import shutil

class TrajectoryLogger:
    """
    Handles saving trajectory data to JSON files for later visualization.
    """
    def __init__(self, save_dir):
        """
        Initializes the logger and ensures the save directory exists.
        
        :param save_dir: The absolute path to the directory where data files will be stored.
        """
        self.save_dir = save_dir
        self.counter = 0
        
        # Clear existing files in the directory if it exists
        if os.path.exists(self.save_dir):
            shutil.rmtree(self.save_dir)
        
        # Recreate the directory to ensure it's empty and available
        os.makedirs(self.save_dir, exist_ok=True)

    def log_trajectories(self, optimal_trajectory, feasible_trajectories, infeasible_trajectories, reference_path, obstacle_positions):
        """
        Saves the relevant trajectory and path information to a JSON file.
        
        :param optimal_trajectory: The calculated optimal trajectory object.
        :param feasible_trajectories: A list of feasible trajectory objects.
        :param infeasible_trajectories: A list of infeasible trajectory objects.
        :param reference_path: A numpy array representing the reference path.
        :param obstacle_positions: A numpy array with the positions of obstacles.
        """
        
        # Helper function to convert a trajectory object into a serializable dictionary
        def traj_to_dict(traj):
            return {
                'cartesian': {
                    'x': list(traj.cartesian.x),
                    'y': list(traj.cartesian.y)
                },
                'costMap': traj.costMap
            }

        # reduce number of trajectories
        feasible_trajectories = feasible_trajectories[:200]

        # Collect all data into a single dictionary
        data_to_save = {
            'reference_path': reference_path.tolist() if reference_path is not None else [],
            'optimal_trajectory': traj_to_dict(optimal_trajectory) if optimal_trajectory is not None else {},
            'feasible_trajectories': [traj_to_dict(t) for t in feasible_trajectories],
            # 'infeasible_trajectories': [traj_to_dict(t) for t in infeasible_trajectories],
            'obstacle_positions': obstacle_positions.tolist() if obstacle_positions is not None else []
        }
        
        # Save the data to a file
        file_path = os.path.join(self.save_dir, f"plot_data_{self.counter:04d}.json")
        try:
            with open(file_path, 'w') as f:
                json.dump(data_to_save, f, indent=4)
        except Exception as e:
            print(f"Error while saving data to {file_path}: {e}")
            
        self.counter += 1

