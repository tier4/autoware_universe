from dataclasses import dataclass
from typing import List, Optional
import copy
import numpy as np
import threading
from tf_transformations import euler_from_quaternion

# Import frenetix core functions
import frenetix
import frenetix.trajectory_functions
import frenetix.trajectory_functions.feasability_functions as ff
import frenetix.trajectory_functions.cost_functions as cf
from autoware_frenetix_planner.sampling_matrix import SamplingHandler
from autoware_frenetix_planner.sampling_matrix import generate_sampling_matrix

@dataclass
class CartesianState:
    """
    Represents the current cartesian kinematic state of the ego vehicle.
    """
    position: List[float]           # [x, y] in meters
    orientation: float              # heading/yaw in radians
    velocity: float                 # longitudinal velocity in m/s
    acceleration: float             # longitudinal acceleration in m/s^2
    steering_angle: Optional[float] = None  # steering angle in radians
    timestamp: Optional[float] = None       # ROS time (float seconds)

@dataclass
class CurvilinearState:
    """
    Represents the current curvilinear state of the ego vehicle.
    """
    s: float                       # longitudinal position along reference path
    s_dot: float                   # longitudinal velocity
    s_ddot: float                  # longitudinal acceleration
    d: float                       # lateral offset from reference path
    d_dot: float                   # lateral velocity
    d_ddot: float                  # lateral acceleration
    timestamp: Optional[float] = None  # ROS time (float seconds)

class FrenetixMotionPlanner:
    """
    This Class is used to interface with the Frenetix planner.
    It manages the reference path, vehicle parameters, cost weights,
    and both cartesian and curvilinear ego states.
    """

    def __init__(self, logger):
        # Initialize parameters
        self.params = {
            "planning_horizon": 8.0,
            "sampling_dt": 0.1,
            "t_min": 1.1,
            "d_min": -3.0,
            "d_max": 3.0,
            "sampling_min": 2,
            "sampling_max": 3,
            "velocity_limit": 20.0,
            "desired_velocity": 8.33,
            "delta_max": 0.6,
            "wheelbase": 2.7,
            "v_switch": 3.0,
            "a_max": 7.0,
            "v_delta_max": 2.0,
            "length": 5.0,
            "width": 2.0,
            "wb_rear_axle": 1.0,
            "v_max": 30.0,
            "cost_weights": {
                "acceleration": 0.0,
                "jerk": 0.0,
                "lateral_jerk": 0.0,
                "longitudinal_jerk": 5.0,
                "orientation_offset": 0.0,
                "lane_center_offset": 0.0,
                "distance_to_reference_path": 1.0,
                "prediction": 0.0,
                "distance_to_obstacles": 0.0,
                "velocity_offset": 0.0,
                "positive_velocity_offset": 10.0,
                "negative_velocity_offset": 0.5,
                "cartesian_lateral_acceleration": 5.0
            }
        }
        
        # Initialize variables
        self.position_lock = threading.Lock()
        self.optimal_trajectory = None
        self.desired_velocity = self.params['desired_velocity']
        self.coordinate_system_cpp = None
        self.reference_path = None
        self.last_endpoint = None
        self.endpoint_threshold = 1.0
        self.cartesian_state: Optional[CartesianState] = None
        self.previous_position = None
        self.curvilinear_state: Optional[CurvilinearState] = None
        self.curvilinear_state_previous: Optional[CurvilinearState] = None
        self.logger = logger

        # Planning cycle control
        self.planning_counter = 0
        self.planning_interval = 3  # Plan every 3 cycles (300ms if called every 100ms)
        self.last_planned_trajectory = None

        # Initialize the sampling handler
        self.sampling_handler = SamplingHandler(dt=self.params['sampling_dt'], 
                                                max_sampling_number=3,
                                                t_min=self.params['t_min'], 
                                                horizon=self.params['planning_horizon'],
                                                delta_d_max=self.params['d_max'],
                                                delta_d_min=self.params['d_min'],
                                                d_ego_pos=True)

        # Initialize the trajectory handler
        self.handler: frenetix.TrajectoryHandler = frenetix.TrajectoryHandler(dt=self.params['sampling_dt'])
        self.coordinate_system_cpp: frenetix.CoordinateSystemWrapper

        # Set the cost weights and feasibility functions
        self._trajectory_handler_set_constant_cost_functions()
        self._trajectory_handler_set_constant_feasibility_functions()
        # self._trajectory_handler_set_changing_cost_functions()

    def set_reference_path(self, reference_path=None):
        """
        Sets the reference path and coordinate system only if the endpoint changes significantly.
        :param reference_path: np.ndarray of shape (N, 2)
        """
        if reference_path is None or len(reference_path) == 0:
            self.logger.info("Received empty reference path, ignoring.")
            return False

        current_endpoint = reference_path[-1]

        if self.reference_path is None or self.last_endpoint is None:
            self.reference_path = reference_path
            self.last_endpoint = current_endpoint
            self.logger.info(f"Reference path set. Endpoint: {current_endpoint}")
            self._set_reference_and_coordinate_system(self.reference_path)
            return True

        dist = np.linalg.norm(current_endpoint - self.last_endpoint)
        if dist > self.endpoint_threshold:
            self.reference_path = reference_path
            self.logger.info(f"Reference path endpoint changed by {dist:.2f} m. Updating path. New endpoint: {current_endpoint}")
            self.last_endpoint = current_endpoint
            self._set_reference_and_coordinate_system(self.reference_path)
            return True

        self.logger.debug(f"Reference path endpoint changed only by {dist:.2f} m (< threshold {self.endpoint_threshold} m), ignoring update.")
        return False

    def set_cartesian_state(self, position, orientation, velocity=None, acceleration=None, steering_angle=None, timestamp=None):
        """
        Sets the ego vehicle's cartesian state and checks for large jumps.
        :param position: object with .x and .y attributes
        :param orientation: quaternion as [x, y, z, w]
        :param velocity: float (optional)
        :param acceleration: float (optional)
        :param steering_angle: float (optional)
        :param timestamp: ROS time (float seconds)
        :return: True if state updated, False otherwise
        """
        try:
            pose_orientation_euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        except Exception as e:
            self.logger.error(f"Failed to convert quaternion to euler: {e}")
            return False

        velocity_threshold = 0.05  # m/s
        velocity_value = 0.0 if velocity is not None and abs(velocity) < velocity_threshold else (velocity if velocity is not None else 0.0)
        acceleration_value = acceleration if acceleration is not None else 0.0

        new_state = CartesianState(
            position=[position.x, position.y],
            orientation=pose_orientation_euler,
            velocity=velocity_value,
            acceleration=acceleration_value,
            steering_angle=steering_angle,
            timestamp=timestamp
        )

        # Jump detection using previous position
        if self.previous_position is not None:
            prev_pos = np.array(self.previous_position)
            curr_pos = np.array(new_state.position)
            dist = np.linalg.norm(curr_pos - prev_pos)
            if dist > 2.0:
                self.logger.debug(f"Ego position jump detected: {dist:.2f}m. Reference path will be reset! New ego position: ({curr_pos[0]:.2f}, {curr_pos[1]:.2f})")
                self.reference_path = None

        self.previous_position = [position.x, position.y]
        self.cartesian_state = new_state

        # Debug message for current cartesian state
        self.logger.debug(
            f"Cartesian state: pos=({self.cartesian_state.position[0]:.2f}, {self.cartesian_state.position[1]:.2f}), "
            f"orientation={self.cartesian_state.orientation:.2f}, vel={self.cartesian_state.velocity:.2f}, acc={self.cartesian_state.acceleration:.2f}, "
            f"steering={self.cartesian_state.steering_angle}, stamp={self.cartesian_state.timestamp}"
        )

        return True

    def set_curvilinear_state(self, s, s_dot, s_ddot, d, d_dot, d_ddot, theta, kappa=None, timestamp=None):
        """
        Sets the ego vehicle's curvilinear state.
        :param s: longitudinal position
        :param s_dot: longitudinal velocity
        :param s_ddot: longitudinal acceleration
        :param d: lateral offset
        :param d_dot: lateral velocity
        :param d_ddot: lateral acceleration
        :param theta: heading in curvilinear frame
        :param kappa: curvature (optional)
        :param timestamp: ROS time (float seconds)
        """
        new_state = CurvilinearState(
            s=s,
            s_dot=s_dot,
            s_ddot=s_ddot,
            d=d,
            d_dot=d_dot,
            d_ddot=d_ddot,
            theta=theta,
            kappa=kappa,
            timestamp=timestamp
        )

        

        self.curvilinear_state_previous = self.curvilinear_state
        self.curvilinear_state = new_state

    def set_desired_velocity(self, desired_velocity: float, current_speed: float = None,
                             v_limit: float = 36):
      """
      Sets desired velocity and calculates velocity for each sample
      :param desired_velocity: velocity in m/s
      :param current_speed: velocity in m/s
      :param v_limit: limit velocity due to behavior planner in m/s
      :return: velocity in m/s
      """
      self.desired_velocity = desired_velocity

      min_v = max(0.001, current_speed - self.params['a_max'] * self.params['planning_horizon'])
      max_v = min(min(current_speed + (self.params['a_max'] / 5.0) * self.params['planning_horizon'], v_limit),
                  self.params['v_max'])

      self.sampling_handler.set_v_sampling(min_v, max_v)

      self.logger.warn('Sampled interval of velocity: {:.2f} m/s - {:.2f} m/s'.format(min_v, max_v))

    def _set_reference_and_coordinate_system(self, reference_path: np.ndarray):
        """
        Sets the reference path and the coordinate system.
        :param reference_path: new reference_path as np.ndarray
        """
        if reference_path is not None:
            self.reference_path = reference_path
            # params = CLCSParams()
            # params.subdivision.max_curvature = 0.20
            # params.default_proj_domain_limit = float(int(1e5))
            # self.coordinate_system = CurvilinearCoordinateSystem(reference_path, params=params, preprocess_path=False)
            self.coordinate_system_cpp = frenetix.CoordinateSystemWrapper(reference_path)

    # Set constant feasibility functions
    def _trajectory_handler_set_constant_feasibility_functions(self):
        """
        Sets the constant feasibility functions for the trajectory handler
        """
        self.handler.add_feasability_function(ff.CheckYawRateConstraint(deltaMax=self.params['delta_max'],
                                                                        wheelbase=self.params['wheelbase'],
                                                                        wholeTrajectory=True
                                                                        ))
        self.handler.add_feasability_function(ff.CheckAccelerationConstraint(switchingVelocity=self.params['v_switch'],
                                                                             maxAcceleration=self.params['a_max'],
                                                                             wholeTrajectory=True)
                                                                             )
        self.handler.add_feasability_function(ff.CheckCurvatureConstraint(deltaMax=self.params['delta_max'],
                                                                          wheelbase=self.params['wheelbase'],
                                                                          wholeTrajectory=True
                                                                          ))
        self.handler.add_feasability_function(ff.CheckCurvatureRateConstraint(wheelbase=self.params['wheelbase'],
                                                                              velocityDeltaMax=self.params['v_delta_max'],
                                                                              wholeTrajectory=True
                                                                              ))

    # Set constant cost functions
    def _trajectory_handler_set_constant_cost_functions(self):
        """
        Sets the constant cost functions for the trajectory handler
        """
        name = "acceleration"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculateAccelerationCost(name, self.params['cost_weights'][name]))

        name = "jerk"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculateJerkCost(name, self.params['cost_weights'][name]))

        name = "lateral_jerk"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculateLateralJerkCost(name, self.params['cost_weights'][name]))

        name = "longitudinal_jerk"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculateLongitudinalJerkCost(name, self.params['cost_weights'][name]))

        name = "orientation_offset"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculateOrientationOffsetCost(name, self.params['cost_weights'][name]))

        name = "lane_center_offset"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculateLaneCenterOffsetCost(name, self.params['cost_weights'][name]))

        name = "distance_to_reference_path"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculateDistanceToReferencePathCost(name, self.params['cost_weights'][name]))

        name = "cartesian_lateral_acceleration"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculateCartesianLateralAccelerationCost(name, self.params['cost_weights'][name], latAccRef=2.0))

    # Set changing cost functions
    def _trajectory_handler_set_changing_cost_functions(self):
        self.handler.add_function(frenetix.trajectory_functions.FillCoordinates(
            lowVelocityMode=False,
            initialOrientation=self.cartesian_state.orientation,
            coordinateSystem=self.coordinate_system_cpp,
            horizon=self.params['planning_horizon']
        ))

        # name = "prediction"
        # if name in self.cost_weights.keys():
        #     self.handler.add_cost_function(
        #         cf.CalculateCollisionProbabilityFast(name, self.cost_weights[name], self.obstacle_predictions,
        #                                              self.vehicle_params['length'], self.vehicle_params['width'], self.vehicle_params['wb_rear_axle']))

        # """
        # :param obstacle_positions: Array of obstacle positions at the current time step
        #                            such as [[x1, y1], [x2, y2], [x3, y3], [x4, y4], [x5, y5]]
        #                            it means that there are 5 obstacles at the current time step
        # """
        # name = "distance_to_obstacles"
        # if name in self.cost_weights.keys() and self.cost_weights[name] > 0 and self.obstacle_positions is not None:
        #     # convert obstacle positions to numpy array
        #     self.obstacle_positions = np.array(self.obstacle_positions)

        #     self.handler.add_cost_function(cf.CalculateDistanceToObstacleCost(name, self.cost_weights[name], self.obstacle_positions))
        #     self.obstacle_positions = None

        name = "velocity_offset"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculateVelocityOffsetCost(
                name,
                self.params['cost_weights'][name],
                self.desired_velocity,
                0.1,
                1.1,
                limit_to_t_min=False,
                norm_order=2
            ))

        name = "positive_velocity_offset"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculatePositiveVelocityOffsetCost(
                name,
                self.params['cost_weights'][name],
                self.desired_velocity,
                0.1,
                3.0,
                limit_to_t_min=False,
                norm_order=2
            ))

        name = "negative_velocity_offset"
        if name in self.params['cost_weights'].keys() and self.params['cost_weights'][name] > 0:
            self.handler.add_cost_function(cf.CalculateNegativeVelocityOffsetCost(
                name,
                self.params['cost_weights'][name],
                self.desired_velocity,
                0.1,
                4.0,
                limit_to_t_min=True,
                norm_order=2
            ))


    # Calculate sampling matrix
    def _generate_sampling_matrix(self, samp_level: int):

        N = int(self.params['planning_horizon'] / self.params['sampling_dt'])
        t1_range = np.array(list(self.sampling_handler.t_sampling.to_range(samp_level).union({N*self.params['sampling_dt']})))
        ss1_range = np.array(list(self.sampling_handler.v_sampling.to_range(samp_level).union({self.curvilinear_state.s_dot})))
        d1_range = np.array(list(self.sampling_handler.d_sampling.to_range(samp_level).union({self.curvilinear_state.d})))

        self.logger.debug(f"Sampling velocity range (ss1_range): {ss1_range}")

        sampling_matrix = generate_sampling_matrix(t0_range=0.0,
                                                   t1_range=t1_range,
                                                   s0_range=self.curvilinear_state.s,
                                                   ss0_range=max(self.curvilinear_state.s_dot, 0.2),
                                                   sss0_range=self.curvilinear_state.s_ddot,
                                                   ss1_range=ss1_range,
                                                   sss1_range=0,
                                                   d0_range=self.curvilinear_state.d,
                                                   dd0_range=self.curvilinear_state.d_dot,
                                                   ddd0_range=self.curvilinear_state.d_ddot,
                                                   d1_range=d1_range,
                                                   dd1_range=0.0,
                                                   ddd1_range=0.0)

        return sampling_matrix
  
    def _plan(self):


        desired_velocity = 5.0

        # if self.curvilinear_state.s > 50:
        #     desired_velocity = 0.0

        # set desired velocity
        self.set_desired_velocity(desired_velocity=desired_velocity, current_speed=self.curvilinear_state.s_dot)

        # set current d position for lateral sampling
        self.sampling_handler.set_d_sampling(self.curvilinear_state.d)

        # set changing cost functions
        self._trajectory_handler_set_changing_cost_functions()

        # Initialization of while loop
        optimal_trajectory = None
        feasible_trajectories = []
        infeasible_trajectories = []
        sampling_level = self.params['sampling_min']
        max_sampling_level = self.params['sampling_max']

        # sample until trajectory has been found or sampling sets are empty
        while optimal_trajectory is None and sampling_level < max_sampling_level:
            self.handler.reset_Trajectories()

            # generate sampling matrix with current sampling level
            sampling_matrix = self._generate_sampling_matrix(sampling_level)

            # generate trajectories
            self.handler.generate_trajectories(sampling_matrix, False)

            self.handler.evaluate_all_current_functions_concurrent(True)

            feasible_trajectories = []
            infeasible_trajectories = []
            for idx, trajectory in enumerate(self.handler.get_sorted_trajectories()):
                # check if trajectory is feasible
                max_vel = max(trajectory.cartesian.v)
                cost = trajectory.cost
                if trajectory.feasible:
                    feasible_trajectories.append(trajectory)
                    self.logger.debug(f"Feasible trajectory {idx}: max_velocity={max_vel:.2f} m/s, cost={cost:.2f}")
                elif trajectory.valid:
                    self.logger.debug(f"Infeasible trajectory {idx}: max_velocity={max_vel:.2f} m/s, cost={cost:.2f}")
                    infeasible_trajectories.append(trajectory)

            # debug trajectories

            if self.curvilinear_state.s_dot < 1.0:
              optimal_trajectory = feasible_trajectories[0] if feasible_trajectories else infeasible_trajectories[0]
            else:
              optimal_trajectory = feasible_trajectories[0] if feasible_trajectories else None

            if optimal_trajectory is not None:
              self.logger.debug(f"Cartesian (theta): {[f'{ori:.4f}' for ori in optimal_trajectory.cartesian.theta]}")
              self.logger.warn(f"Cartesian (v): {[f'{ori:.4f}' for ori in optimal_trajectory.cartesian.v]}")
              self.logger.warn(f"max velocity evaluation: {max(optimal_trajectory.cartesian.v):.4f}")
              self.logger.warn(f"max acceleration evaluation: {max(optimal_trajectory.cartesian.a):.4f}")
              self.logger.debug(f"Curvilinear (theta): {[f'{ori:.4f}' for ori in optimal_trajectory.curvilinear.theta]}")
              self.logger.debug(f"Curvilinear (s): {[f'{ori:.4f}' for ori in optimal_trajectory.curvilinear.s]}")
              self.logger.debug(f"Curvilinear (s_dot): {[f'{ori:.4f}' for ori in optimal_trajectory.curvilinear.s_dot]}")
              self.logger.debug(f"Curvilinear (d): {[f'{ori:.4f}' for ori in optimal_trajectory.curvilinear.d]}") 
              self.logger.debug(f"Curvilinear (d_dot): {[f'{ori:.4f}' for ori in optimal_trajectory.curvilinear.d_dot]}")

              for key in optimal_trajectory.costMap.keys():
                self.logger.warn(f"{key} cost evaluation: {optimal_trajectory.costMap[key][0]:.2f}")

              # self.logger.warn(f"cartesian Lateral Acceleration cost evaluation: {optimal_trajectory.costMap['cartesian_lateral_acceleration'][1]:.2f}")

            # increase sampling level (i.e., density) if no optimal trajectory could be found
            sampling_level += 1

          # --- Debug message with all relevant planning info ---
        self.logger.debug(
          f"Planning finished: "
          f"sampling_level={sampling_level-1}, "
          f"num_feasible={len(feasible_trajectories)}, "
          f"num_infeasible={len(infeasible_trajectories)}, "
          f"optimal_trajectory_found={'yes' if optimal_trajectory is not None else 'no'}"
          )
        
        if optimal_trajectory is not None:
          self.logger.debug(
              f"Optimal trajectory: cost={optimal_trajectory.cost:.2f}, "
              f"length={len(optimal_trajectory.cartesian.x)}, "
              f"max velocity={max(optimal_trajectory.cartesian.v)}"
          )
          return optimal_trajectory
        else:
            self.logger.warn("No feasible trajectory found in Frenetix planning cycle.")
            return None

      
    def cyclic_plan(self):
        """
        Plans a trajectory cyclically. Only replans every planning_interval cycles,
        otherwise returns the last planned trajectory.
        """
        if self.coordinate_system_cpp is None or self.reference_path is None:
            self.logger.warn("Reference path or coordinate system not set, cannot plan.")
            return None

        if self.cartesian_state is None:
            self.logger.warn("Cartesian state not set, cannot plan.")
            return None

        # Increment planning counter
        self.planning_counter += 1

        # Check if we should plan a new trajectory
        should_plan = (self.planning_counter % self.planning_interval == 0) or (self.last_planned_trajectory is None)

        if not should_plan:
            # Return last planned trajectory
            self.logger.debug(f"Using cached trajectory (cycle {self.planning_counter})")
            return self.last_planned_trajectory

        # Plan new trajectory
        self.logger.info(f"Planning new trajectory (cycle {self.planning_counter})")
        
        with self.position_lock:
            cartesian_state = copy.deepcopy(self.cartesian_state)

        cartesian_state_frenetix = frenetix.CartesianPlannerState(np.asarray(cartesian_state.position), 
                                                                  cartesian_state.orientation, 
                                                                  cartesian_state.velocity, 
                                                                  cartesian_state.acceleration, 
                                                                  cartesian_state.steering_angle)

        curvilinear_state_frenetix = frenetix.compute_initial_state(coordinate_system=self.coordinate_system_cpp,
                                                                    x_0=cartesian_state_frenetix,
                                                                    wheelbase=self.params['wheelbase'],
                                                                    low_velocity_mode=False)
        
        self.curvilinear_state = CurvilinearState(s=curvilinear_state_frenetix.x0_lon[0],
                                                  s_dot=curvilinear_state_frenetix.x0_lon[1],
                                                  s_ddot=curvilinear_state_frenetix.x0_lon[2],
                                                  d=curvilinear_state_frenetix.x0_lat[0],
                                                  d_dot=curvilinear_state_frenetix.x0_lat[1],
                                                  d_ddot=curvilinear_state_frenetix.x0_lat[2],
                                                  timestamp=cartesian_state.timestamp)
        
        self.logger.debug(
              f"Curvilinear state: s={self.curvilinear_state.s:.2f}, s_dot={self.curvilinear_state.s_dot:.2f}, s_ddot={self.curvilinear_state.s_ddot:.2f}, "
              f"d={self.curvilinear_state.d:.2f}, d_dot={self.curvilinear_state.d_dot:.2f}, d_ddot={self.curvilinear_state.d_ddot:.2f}, "
              f"stamp={self.curvilinear_state.timestamp}"
          )

        # plan optimal trajectory
        optimal_trajectory = self._plan()
        
        # Cache the planned trajectory
        if optimal_trajectory is not None:
            self.last_planned_trajectory = optimal_trajectory
            self.logger.debug(f"New trajectory planned and cached")
        else:
            self.logger.warn(f"Planning failed, keeping last trajectory")
        
        return optimal_trajectory











