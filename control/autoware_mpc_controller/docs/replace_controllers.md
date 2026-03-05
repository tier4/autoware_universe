# Longitudinal MPC (no combined MPC)

The **combined longitudinal + lateral MPC** and **`mpc_controller_node`** have been removed from this package. This package now provides only:

- **Longitudinal-only MPC** — Used by `autoware_trajectory_follower_node` when `longitudinal_controller_mode: longitudinal_mpc`. The trajectory follower runs the lateral controller (e.g. MPC or pure_pursuit) and the longitudinal MPC in the same process and publishes a single `control_cmd`.

To use longitudinal MPC: set `longitudinal_controller_mode: longitudinal_mpc` in the trajectory follower config (or launch arg) and ensure the longitudinal MPC param file is loaded (e.g. `longitudinal_mpc.param.yaml`). See README and `autoware_launch` config for paths.
