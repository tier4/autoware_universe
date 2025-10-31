from typing import Optional, Tuple
import copy
import uuid
import numpy as np
import threading
from tf_transformations import euler_from_quaternion
from autoware_frenetix_planner.config import CartesianState, CurvilinearState

from shapely.geometry import LineString, MultiLineString
import time

# Import frenetix core functions
import frenetix
import frenetix.trajectory_functions
import frenetix.trajectory_functions.feasability_functions as ff
import frenetix.trajectory_functions.cost_functions as cf
from autoware_frenetix_planner.sampling_matrix import SamplingHandler
from autoware_frenetix_planner.sampling_matrix import generate_sampling_matrix
from autoware_frenetix_planner.trajectory_logger import TrajectoryLogger
import autoware_frenetix_planner.debug_map as dm
from autoware_frenetix_planner.route_utils import process_route_to_drivable_area, convert_lanelet_linestrings_to_shapely

class FrenetixMotionPlanner:
    """
    This Class is used to interface with the Frenetix planner.
    It manages the reference path, vehicle parameters, cost weights,
    and both cartesian and curvilinear ego states.
    """

    def __init__(self, logger, params, logging_enabled=False):
        self.logger = logger

        # Initialize parameters
        self.params = params
        self.logger.info("Planner initialized with provided parameters.")

        # Initialize variables
        self.position_lock = threading.Lock()
        self.optimal_trajectory = None
        self.desired_velocity = self.params.desired_velocity
        self.coordinate_system_cpp = None
        self.lanelet_map = None
        self.reference_path = None
        self.route = None
        self.last_endpoint = None
        self.endpoint_threshold = 1.0
        self.cartesian_state: Optional[CartesianState] = None
        self.previous_position = None

        # Obstacle and prediction settings
        self.obstacle_positions = []
        self.obstacle_predictions_covariance = [[0.1, 0.0], [0.0, 0.1]]
        self.obstacle_predictions = {}  # Optimized obstacle predictions storage

        # driveable area
        self.shapely_left_border: Optional[MultiLineString] = None
        self.shapely_right_border: Optional[MultiLineString] = None
    
        # Planning cycle control
        self.last_planned_trajectory = None
        # TODO add as ros2 parameter
        # How far (meters) can we deviate laterally from the plan before replanning?
        self.replan_lat_deviation_threshold = 0.5
        # How close (remaining seconds) to the end of the trajectory should we trigger a replan?
        self.replan_completion_time_threshold = 6.0
        # How far in the future (seconds) on the old trajectory should we start the new plan? (Splicing)
        self.replan_splice_time_s = 0
        self.force_replan = False  # Flag to force replanning

        # Initialize the sampling handler
        self.sampling_handler = SamplingHandler(dt=self.params.sampling_dt, 
                                                max_sampling_number=3,
                                                t_min=self.params.t_min, 
                                                horizon=self.params.planning_horizon,
                                                delta_d_max=self.params.d_max,
                                                delta_d_min=self.params.d_min,
                                                d_ego_pos=True)

        # Initialize the trajectory handler
        self.handler: frenetix.TrajectoryHandler = frenetix.TrajectoryHandler(dt=self.params.sampling_dt)
        self.coordinate_system_cpp: frenetix.CoordinateSystemWrapper

        # Set the cost weights and feasibility functions
        self._trajectory_handler_set_constant_cost_functions()
        self._trajectory_handler_set_constant_feasibility_functions()
        # self._trajectory_handler_set_changing_cost_functions()

        # Initialize the trajectory logger
        if logging_enabled:
          self.trajectory_logger = TrajectoryLogger(save_dir="/workspace/src/universe/autoware_universe/planning/autoware_frenetix_planner/test/logs",
                                                    mode="trajectories")
        else:
            self.trajectory_logger = None

        # debug only
        self.left = None
        self.right = None
        self.ax = None

    # Method to update parameters at runtime
    def update_params(self, new_params):
        """
        Updates the planner's parameters and re-initializes components that depend on them.
        """
        self.logger.info("Updating planner parameters...")
        self.params = new_params

        # Force replan
        self.force_replan = True
        
        # Re-initialize components that depend on the parameters
        self._trajectory_handler_set_constant_cost_functions()

        # Re-initialize feasibility functions if they depend on parameters
        self._trajectory_handler_set_constant_feasibility_functions()

        self.logger.info("Planner parameters updated successfully.")

    def set_lanelet_map(self, lanelet_map):
        """
        Sets the lanelet map for the planner.
        :param lanelet_map: Lanelet2 map object
        """
        self.lanelet_map = lanelet_map
        self.logger.info("Lanelet map set for the planner.")

    def set_route(self, route):
        """
        Sets the current route for the planner.
        :param route: List of lanelet IDs representing the route
        """
        self.route = route
        self.logger.info(f"Route set with {len(route)} lanelets.")

    def process_route(self):
        """
        Processes the current route to determine the drivable area. (if route and map is available)
        """
        if self.lanelet_map is None or self.route is None:
            self.logger.error("Lanelet map or route not set. Cannot process drivable area.")
            return None, None, None

        all_drivable_ids, left_boundary, right_boundary = process_route_to_drivable_area(
            self.lanelet_map, self.route)
        
        # Convert boundaries to Shapely MultiLineStrings for further processing
        self.logger.debug("Converting route boundaries to Shapely objects...")
        self.shapely_left_border = convert_lanelet_linestrings_to_shapely(left_boundary, self.logger)
        self.shapely_right_border = convert_lanelet_linestrings_to_shapely(right_boundary, self.logger)

        # if self.left is not None:
        #     for ln in self.left:
        #         ln[0].remove()
        
        # if self.right is not None:
        #     for ln in self.right:
        #         ln[0].remove()

        if self.ax is None:
            pass
          # self.ax = dm.debug_map(self.lanelet_map)

        # self.left = dm.plot_line_string_list(self.ax, left_boundary, color='green', linewidth=3)
        # self.right = dm.plot_line_string_list(self.ax, right_boundary, color='red', linewidth=3)

        self.logger.debug(f"Processed drivable area with {len(all_drivable_ids)} unique lanelet IDs.")

    def set_objects(self, objects_msg):
        """
        Processes predicted objects for collision checking and stores them in optimized format.
        
        Args:
            objects_msg: PredictedObjects message containing obstacle predictions
            
        Returns:
            bool: True if objects were successfully processed and updated, False otherwise
        """
        # Early exit for empty or invalid messages
        if objects_msg is None or len(objects_msg.objects) == 0:
            self.logger.debug("Received empty objects message, clearing predictions.")
            self.obstacle_predictions = {}
            self.obstacle_positions = np.zeros((0, 2), dtype=np.float64)
            return False

        # Initialize predictions dictionaries
        predictions = {}

        # Initialize obstacle positions
        self.obstacle_positions = []

        # Process each object in the message
        for obj in objects_msg.objects:
            try:
                # Generate compact object ID (modulo to keep it manageable)
                obj_id = uuid.UUID(bytes=np.array(obj.object_id.uuid).tobytes()).int % 100
      
                # Extract predicted path (use first prediction path if available)
                if not obj.kinematics.predicted_paths:
                    self.logger.debug(f"Object {obj_id} has no predicted paths, skipping.")
                    continue
                    
                predicted_path = obj.kinematics.predicted_paths[0].path
                path_length = len(predicted_path)
                
                if path_length == 0:
                    self.logger.debug(f"Object {obj_id} has empty predicted path, skipping.")
                    continue
                
                # Store current position for distance cost functions
                self.obstacle_positions.append([predicted_path[0].position.x, predicted_path[0].position.y])
                
                # Pre-allocate array for better performance
                pwc_list = [None] * path_length
                
                # Loop to extract all data and create C++ objects
                for i, path_point in enumerate(predicted_path):
                    
                    # Extract position
                    position = np.array([path_point.position.x, 
                                         path_point.position.y, 
                                         0.0], 
                                         dtype=np.float64)
                    
                    # Extract orientation
                    orientation = np.array([path_point.orientation.x, 
                                            path_point.orientation.y, 
                                            path_point.orientation.z, 
                                            path_point.orientation.w], 
                                            dtype=np.float64)

                    # Create 6x6 covariance matrix from 2x2 position covariance
                    covariance_matrix = np.zeros((6, 6), dtype=np.float64)
                    covariance_matrix[:2, :2] = np.array(self.obstacle_predictions_covariance, dtype=np.float64)
                    
                    # Create PoseWithCovariance object for C++
                    pwc = frenetix.PoseWithCovariance(position, orientation, covariance_matrix)
                    pwc_list[i] = pwc
                
                # Store Frenetix compatible prediction data
                predictions[obj_id] = frenetix.PredictedObject(
                    int(obj_id),
                    pwc_list,
                    round(obj.shape.dimensions.x, 2), 
                    round(obj.shape.dimensions.y, 2)
                )
                
            except Exception as e:
                self.logger.error(f"Failed to process object {obj_id}: {e}")
                continue
        
        # Log processing results
        self.logger.debug(f"Successfully processed {len(predictions)} obstacles", 
                         throttle_duration_sec=1.0)
        
        # Update predictions and current positions
        self.obstacle_predictions = predictions
        self.obstacle_positions = np.array(self.obstacle_positions, dtype=np.float64)

        return len(predictions) > 0


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
            # Force replan
            self.force_replan = True
            return True

        dist = np.linalg.norm(current_endpoint - self.last_endpoint)
        if dist > self.endpoint_threshold:
            self.reference_path = reference_path
            self.logger.info(f"Reference path endpoint changed by {dist:.2f} m. Updating path. New endpoint: {current_endpoint}")
            self.last_endpoint = current_endpoint
            self._set_reference_and_coordinate_system(self.reference_path)
            # Force replan
            self.force_replan = True
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

      min_v = max(0.001, current_speed - self.params.a_max * self.params.planning_horizon)
      max_v = min(min(current_speed + (self.params.a_max / 5.0) * self.params.planning_horizon, v_limit),
                  self.params.v_max)

      self.sampling_handler.set_v_sampling(min_v, max_v)

      self.logger.debug('Sampled interval of velocity: {:.2f} m/s - {:.2f} m/s'.format(min_v, max_v))

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
        self.handler.add_feasability_function(ff.CheckYawRateConstraint(deltaMax=self.params.delta_max,
                                                                        wheelbase=self.params.wheelbase,
                                                                        wholeTrajectory=True
                                                                        ))
        self.handler.add_feasability_function(ff.CheckAccelerationConstraint(switchingVelocity=self.params.v_switch,
                                                                             maxAcceleration=self.params.a_max,
                                                                             wholeTrajectory=True)
                                                                             )
        self.handler.add_feasability_function(ff.CheckCurvatureConstraint(deltaMax=self.params.delta_max,
                                                                          wheelbase=self.params.wheelbase,
                                                                          wholeTrajectory=True
                                                                          ))
        self.handler.add_feasability_function(ff.CheckCurvatureRateConstraint(wheelbase=self.params.wheelbase,
                                                                              velocityDeltaMax=self.params.v_delta_max,
                                                                              wholeTrajectory=True
                                                                              ))

    # Set constant cost functions
    def _trajectory_handler_set_constant_cost_functions(self):
        """
        Sets the constant cost functions for the trajectory handler
        """
        name = "acceleration"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(cf.CalculateAccelerationCost(name, self.params.cost_weights[name]))

        name = "jerk"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(cf.CalculateJerkCost(name, self.params.cost_weights[name]))

        name = "lateral_jerk"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(cf.CalculateLateralJerkCost(name, self.params.cost_weights[name]))

        name = "longitudinal_jerk"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(cf.CalculateLongitudinalJerkCost(name, self.params.cost_weights[name]))

        name = "orientation_offset"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(cf.CalculateOrientationOffsetCost(name, self.params.cost_weights[name]))

        name = "lane_center_offset"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(cf.CalculateLaneCenterOffsetCost(name, self.params.cost_weights[name]))

        name = "distance_to_reference_path"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(cf.CalculateDistanceToReferencePathCost(name, self.params.cost_weights[name]))

        name = "cartesian_lateral_acceleration"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(cf.CalculateCartesianLateralAccelerationCost(name, self.params.cost_weights[name], latAccRef=2.0))

    # Set changing cost functions
    def _trajectory_handler_set_changing_cost_functions(self):
        self.handler.add_function(frenetix.trajectory_functions.FillCoordinates(
            lowVelocityMode=False,
            initialOrientation=self.cartesian_state.orientation,
            coordinateSystem=self.coordinate_system_cpp,
            horizon=self.params.planning_horizon
        ))

        name = "prediction"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(
                cf.CalculateCollisionProbabilityFast(name, self.params.cost_weights[name], self.obstacle_predictions,
                                                     self.params.length, self.params.width, self.params.wb_rear_axle, 0.5))

        # name = "distance_to_obstacles"
        # if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0 and self.obstacle_positions is not None:
        #     # convert obstacle positions to numpy array
        #     self.obstacle_positions = np.array(self.obstacle_positions, dtype=np.float64)
        #     self.handler.add_cost_function(cf.CalculateDistanceToObstacleCost(name, self.params.cost_weights[name], self.obstacle_positions))

        # name = "positive_velocity_offset"
        # if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
        #     self.handler.add_cost_function(cf.CalculatePositiveVelocityOffsetCost(
        #         name,
        #         self.params.cost_weights[name],
        #         self.desired_velocity,
        #         0.1,
        #         3.0,
        #         limit_to_t_min=False,
        #         norm_order=2
        #     ))

        name = "positive_velocity_offset"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(cf.CalculatePositiveVelocityOffsetCost(
                name,
                self.params.cost_weights[name],
                self.desired_velocity,
                0.1,
                3.0,
                limit_to_t_min=False,
                norm_order=2
            ))

        name = "negative_velocity_offset"
        if name in self.params.cost_weights.keys() and self.params.cost_weights[name] > 0:
            self.handler.add_cost_function(cf.CalculateNegativeVelocityOffsetCost(
                name,
                self.params.cost_weights[name],
                self.desired_velocity,
                0.1,
                4.0,
                limit_to_t_min=True,
                norm_order=2
            ))


    # Calculate sampling matrix
    def _generate_sampling_matrix(self, samp_level: int, planning_start_state_curvilinear):

        N = int(self.params.planning_horizon / self.params.sampling_dt)
        t1_range = np.array(list(self.sampling_handler.t_sampling.to_range(samp_level).union({N*self.params.sampling_dt})))
        ss1_range = np.array(list(self.sampling_handler.v_sampling.to_range(samp_level).union({planning_start_state_curvilinear.s_dot})))
        d1_range = np.array(list(self.sampling_handler.d_sampling.to_range(samp_level).union({planning_start_state_curvilinear.d})))

        self.logger.debug(f"Sampling velocity range (ss1_range): {ss1_range}")

        sampling_matrix = generate_sampling_matrix(t0_range=0.0,
                                                   t1_range=t1_range,
                                                   s0_range=planning_start_state_curvilinear.s,
                                                   ss0_range=max(planning_start_state_curvilinear.s_dot, 0.2),
                                                   sss0_range=planning_start_state_curvilinear.s_ddot,
                                                   ss1_range=ss1_range,
                                                   sss1_range=0,
                                                   d0_range=planning_start_state_curvilinear.d,
                                                   dd0_range=planning_start_state_curvilinear.d_dot,
                                                   ddd0_range=planning_start_state_curvilinear.d_ddot,
                                                   d1_range=d1_range,
                                                   dd1_range=0.0,
                                                   ddd1_range=0.0)

        return sampling_matrix
    
    def boundary_collision_check(self, trajectory) -> bool:
        """
        Checks if a given trajectory (as a buffered polygon) intersects
        with the pre-computed shapely boundary linestrings.

        Args:
            trajectory: A Frenetix trajectory object.

        Returns:
            True if collision is detected, False otherwise.
        """
        # 1. Check if boundaries are available
        if not self.shapely_left_border or not self.shapely_right_border:
            self.logger.warn("Shapely boundaries not set. Skipping collision check.",
                             throttle_duration_sec=5.0)
            return False  # Assume no collision if boundaries are missing

        # 2. Check for empty trajectory
        if len(trajectory.cartesian.x) == 0:
            return True  # Empty trajectory is invalid

        try:
            # 3. Create numpy array from trajectory points
            # np.column_stack creates an N-by-2 array, which Shapely handles efficiently
            traj_points = np.column_stack((trajectory.cartesian.x, 
                                           trajectory.cartesian.y))
            
            if len(traj_points) < 2:
                self.logger.debug("Trajectory has fewer than 2 points, cannot form LineString.")
                return True  # Treat invalid trajectory as a collision

            # Create LineString directly from numpy array
            traj_line = LineString(traj_points)

            # 4. Create the buffered polygon (swept path)
            # We use half the vehicle width as the buffer radius.
            # cap_style=2 (flat) is more realistic for a vehicle shape than round.
            vehicle_half_width = self.params.width / 2.0
            buffered_trajectory = traj_line.buffer(vehicle_half_width, cap_style=2) 

            # 5. Perform the intersection check
            if buffered_trajectory.intersects(self.shapely_left_border):
                self.logger.debug("Trajectory COLLIDES with LEFT border.")
                return True
            
            if buffered_trajectory.intersects(self.shapely_right_border):
                self.logger.debug("Trajectory COLLIDES with RIGHT border.")
                return True

        except Exception as e:
            self.logger.error(f"Error in static collision check: {e}")
            return True  # Assume collision on error

        # 6. No intersections found
        return False
    
    def trajectories_collision_check(self, trajectories: list, check_all: bool = False):
        """
        Iterates through a list of trajectories and returns the first one
        that does not collide (currently only map boundaries are considered).

        Args:
            trajectories: A list of Frenetix trajectory objects, assumed to be
                          sorted by cost (best first).
            check_all: If False (default), stop and return the first valid
                       trajectory found. If True, check all trajectories
                       and return the best valid one (if any) and a complete
                       list of all colliding ones.

        Returns:
            A tuple:
            (Optional[Trajectory]): The first/best collision-free trajectory, or None if all collide.
            (list): A list of trajectories from the input that *did* collide.
        """
        self.logger.debug(f"Checking {len(trajectories)} trajectories for boundary collision (check_all={check_all})...")
        colliding_trajectories = []
        optimal_collision_free_trajectory = None
        
        for i, trajectory in enumerate(trajectories):
            # Check for collision with the drivable area boundaries
            if self.boundary_collision_check(trajectory):
                # This trajectory collides, add it to the list
                colliding_trajectories.append(trajectory)
            else:
                # This is a collision-free trajectory.
                
                if optimal_collision_free_trajectory is None:
                    # This is the first collision-free one we've found.
                    # We store it as the "best" one.
                    self.logger.info(f"Collision-free trajectory found at index {i}.")
                    optimal_collision_free_trajectory = trajectory
                
                if not check_all:
                    # EFFICIENT MODE: We are not checking all.
                    # Since we found the first valid one, we can stop
                    # and return immediately.
                    self.logger.debug("Stopping check early (check_all=False).")
                    return optimal_collision_free_trajectory, colliding_trajectories
        
        # This point is reached if:
        # 1. check_all=True (full loop completed)
        # 2. check_all=False (full loop completed, meaning all trajectories collided)
        
        if optimal_collision_free_trajectory is None:
            # This case happens if the for-loop completes and no valid trajectory was found.
            self.logger.warn(f"All {len(trajectories)} 'feasible' trajectories collided with map boundary.")
        
        # Return the results after checking all (or all collided in efficient mode)
        return optimal_collision_free_trajectory, colliding_trajectories
  
    def _plan(self, planning_start_state_curvilinear: CurvilinearState):

        desired_velocity = self.params.desired_velocity

        # set desired velocity
        self.set_desired_velocity(desired_velocity=desired_velocity, current_speed=planning_start_state_curvilinear.s_dot)

        # set current d position for lateral sampling
        self.sampling_handler.set_d_sampling(planning_start_state_curvilinear.d)

        # set changing cost functions
        self._trajectory_handler_set_changing_cost_functions()

        # Initialization of while loop
        optimal_trajectory = None
        feasible_trajectories = []
        infeasible_trajectories = []
        sampling_level = self.params.sampling_min
        max_sampling_level = self.params.sampling_max

        # sample until trajectory has been found or sampling sets are empty
        while optimal_trajectory is None and sampling_level < max_sampling_level:
            self.handler.reset_Trajectories()

            # generate sampling matrix with current sampling level
            sampling_matrix = self._generate_sampling_matrix(sampling_level, planning_start_state_curvilinear)

            # generate trajectories
            self.handler.generate_trajectories(sampling_matrix, False)

            self.handler.evaluate_all_current_functions_concurrent(True)

            feasible_trajectories = []
            infeasible_trajectories = []
            for trajectory in self.handler.get_sorted_trajectories():
                # check if trajectory is feasible
                if trajectory.feasible:
                    feasible_trajectories.append(trajectory)
                elif trajectory.valid:
                    infeasible_trajectories.append(trajectory)
            
            # dm.plot_trajectories(self.ax, feasible_trajectories, color='green', linewidth=0.5)
            # dm.plot_trajectories(self.ax, collision_trajectories, color='red', linewidth=0.5)

            # debug trajectories
          
            if self.trajectory_logger is not None:
              self.trajectory_logger.save_debug_data(optimal_trajectory=optimal_trajectory, 
                                                    feasible_trajectories=feasible_trajectories,  
                                                    infeasible_trajectories=infeasible_trajectories,
                                                    reference_path=self.reference_path, 
                                                    obstacle_positions=self.obstacle_positions,
                                                    cartesian_state=self.cartesian_state,
                                                    curvilinear_state=self.curvilinear_state,
                                                    )

            try:
              # if vehicle is still very slow allow infeasible trajectories #
              if planning_start_state_curvilinear.s_dot < 1.0:
                trajectories = feasible_trajectories if feasible_trajectories else infeasible_trajectories
              else:
                trajectories = feasible_trajectories if feasible_trajectories else None

              # check trajectories for collision
              start = time.time()
              optimal_trajectory, colliding = self.trajectories_collision_check(trajectories, check_all=False) 
              self.logger.debug(f"Collision checking time: {time.time() - start:.4f} seconds.")

            except Exception as e:
              self.logger.error(f"No optimal trajectory found!")
              optimal_trajectory = None

            # if optimal_trajectory is not None:
            #   self.logger.debug(f"Cartesian (theta): {[f'{ori:.4f}' for ori in optimal_trajectory.cartesian.theta]}")
            #   self.logger.warn(f"Cartesian (v): {[f'{ori:.4f}' for ori in optimal_trajectory.cartesian.v]}")
            #   self.logger.warn(f"max velocity evaluation: {max(optimal_trajectory.cartesian.v):.4f}")
            #   self.logger.debug(f"max acceleration evaluation: {max(optimal_trajectory.cartesian.a):.4f}")
            #   self.logger.debug(f"Curvilinear (theta): {[f'{ori:.4f}' for ori in optimal_trajectory.curvilinear.theta]}")
            #   self.logger.debug(f"Curvilinear (s): {[f'{ori:.4f}' for ori in optimal_trajectory.curvilinear.s]}")
            #   self.logger.debug(f"Curvilinear (s_dot): {[f'{ori:.4f}' for ori in optimal_trajectory.curvilinear.s_dot]}")
            #   self.logger.debug(f"Curvilinear (d): {[f'{ori:.4f}' for ori in optimal_trajectory.curvilinear.d]}") 
            #   self.logger.debug(f"Curvilinear (d_dot): {[f'{ori:.4f}' for ori in optimal_trajectory.curvilinear.d_dot]}")

            #   for key in optimal_trajectory.costMap.keys():
            #     self.logger.warn(f"{key} cost evaluation: {optimal_trajectory.costMap[key][0]:.2f}")

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
        
    def _get_current_curvilinear_state(self) -> Optional[CurvilinearState]:
        """
        Gets the current cartesian state and converts it to curvilinear.
        """
        with self.position_lock:
            if self.cartesian_state is None:
                self.logger.warn("Cartesian state not set, cannot get curvilinear state.")
                return None
            cartesian_state = copy.deepcopy(self.cartesian_state)

        cartesian_state_frenetix = frenetix.CartesianPlannerState(np.asarray(cartesian_state.position), 
                                                                  cartesian_state.orientation, 
                                                                  cartesian_state.velocity, 
                                                                  cartesian_state.acceleration, 
                                                                  cartesian_state.steering_angle)

        try:
            curvilinear_state_frenetix = frenetix.compute_initial_state(coordinate_system=self.coordinate_system_cpp,
                                                                        x_0=cartesian_state_frenetix,
                                                                        wheelbase=self.params.wheelbase,
                                                                        low_velocity_mode=False)
        except Exception as e:
            self.logger.error(f"Failed to compute initial state: {e}")
            return None
        
        current_curvilinear_state = CurvilinearState(s=curvilinear_state_frenetix.x0_lon[0],
                                                     s_dot=curvilinear_state_frenetix.x0_lon[1],
                                                     s_ddot=curvilinear_state_frenetix.x0_lon[2],
                                                     d=curvilinear_state_frenetix.x0_lat[0],
                                                     d_dot=curvilinear_state_frenetix.x0_lat[1],
                                                     d_ddot=curvilinear_state_frenetix.x0_lat[2],
                                                     timestamp=cartesian_state.timestamp)
        
        self.logger.debug(
              f"Current Curvilinear state: s={current_curvilinear_state.s:.2f}, d={current_curvilinear_state.d:.2f}"
          )
        return current_curvilinear_state
    
    def _should_replan(self, current_curvilinear_state: CurvilinearState) -> Tuple[bool, str]:
        """
        Checks if a replan is necessary based on deviation or trajectory completion.
        """
        # Must plan if we don't have a trajectory
        if self.last_planned_trajectory is None:
            return True, "No previous trajectory"

        try:
            old_s = self.last_planned_trajectory.curvilinear.s
            old_d = self.last_planned_trajectory.curvilinear.d

            if len(old_s) < 2:
                return True, "Previous trajectory too short"

            current_s = current_curvilinear_state.s
            current_d = current_curvilinear_state.d

            # Check for completion
            # end_s = old_s[-1]
            # if end_s - current_s < self.replan_completion_s_threshold:
            #     return True, f"Trajectory completion (s={current_s:.1f}, end_s={end_s:.1f})"

            # Check for remaining time
            current_index = np.argmin(np.abs(old_s - current_s))
            total_steps = len(old_s)
            remaining_steps = total_steps - current_index
            trigger_steps_threshold = int(self.replan_completion_time_threshold / self.params.sampling_dt)
            if remaining_steps < trigger_steps_threshold:
                return True, f"Trajectory completion (remaining time < {self.replan_completion_time_threshold}s)"

            # Check for lateral deviation
            # Find the planned 'd' value at our current 's' position
            planned_d_at_current_s = np.interp(current_s, old_s, old_d)
            deviation = abs(planned_d_at_current_s - current_d)
            
            if deviation > self.replan_lat_deviation_threshold:
                return True, f"Lateral deviation ({deviation:.2f}m > {self.replan_lat_deviation_threshold}m)"

            # 4. TODO: Check for environment change (e.g., new obstacle invalidates path)
            # This is more complex, would require re-evaluating trajectory cost
            
        except Exception as e:
            self.logger.error(f"Error in _should_replan check: {e}")
            return True, "Error in check" # Fail-safe: replan on error

        # 5. If no triggers hit, keep following
        return False, "Following"
    
    def _get_replan_start_state(self, current_curvilinear_state: CurvilinearState) -> Tuple[CurvilinearState, int]:
        """
        Determines the state from which to start the new plan.
        This implements the "splicing" logic.

        Returns:
            A tuple: (CurvilinearState, int)
            - The state to plan from (can be current or future state)
            - The index on the old trajectory to splice at (0 if planning from current state)
        """
        # If we have no old trajectory, we must plan from the current ego state
        if self.last_planned_trajectory is None:
            self.logger.debug("No old trajectory, planning from current ego state.")
            return current_curvilinear_state, 0

        try:
            # Find the point on the old trajectory that is N seconds in the future
            # from our *current s* position
            current_s = current_curvilinear_state.s
            old_s = self.last_planned_trajectory.curvilinear.s
            
            # Find the index of the closest point on the old path
            closest_index = np.argmin(np.abs(old_s - current_s))
            
            # Calculate how many steps in the future to start the plan
            # This is the "splicing" point
            splice_steps = int(self.replan_splice_time_s / self.params.sampling_dt)
            replan_index = min(closest_index + splice_steps, len(old_s) - 1)

            # Extract the state from the old trajectory at that future point
            replan_state = CurvilinearState(
                s=self.last_planned_trajectory.curvilinear.s[replan_index],
                s_dot=self.last_planned_trajectory.curvilinear.s_dot[replan_index],
                s_ddot=self.last_planned_trajectory.curvilinear.s_ddot[replan_index],
                d=self.last_planned_trajectory.curvilinear.d[replan_index],
                d_dot=self.last_planned_trajectory.curvilinear.d_dot[replan_index],
                d_ddot=self.last_planned_trajectory.curvilinear.d_ddot[replan_index],
            )
            
            self.logger.info(f"Planning from future state (index {replan_index}): s={replan_state.s:.2f}, d={replan_state.d:.2f}")
            return replan_state, replan_index
            
        except Exception as e:
            self.logger.error(f"Failed to get replan start state, planning from ego: {e}")
            # Fallback: plan from the current ego state
            return current_curvilinear_state, 0

  
    def cyclic_plan(self):
        """
        This is the main entry point from the node.
        It decides IF to replan, FROM WHERE to replan, and then calls _plan.
        If no replan is needed, it returns the previously cached trajectory.
        """
        if self.coordinate_system_cpp is None or self.reference_path is None:
            self.logger.warn("Reference path or coordinate system not set, cannot plan.")
            return None

        # Get the current ego state in curvilinear coordinates
        current_curvilinear_state = self._get_current_curvilinear_state()
        if current_curvilinear_state is None:
            # Failed to get state (e.g., off route), keep old plan as fallback
            self.logger.warn("Failed to get current curvilinear state. Returning last trajectory.")
            return self.last_planned_trajectory 

        # Decide IF a replan is needed
        if self.force_replan:
            replan_needed = True
            reason = "Forced replan due to parameter or reference path change"
            self.force_replan = False  # Reset the flag
        else:
          # replan_needed, reason = self._should_replan(current_curvilinear_state)
          replan_needed = True
          reason = "Temporary override: always replan"

        if not replan_needed:
            # No replan needed, just return the existing trajectory
            self.logger.debug(f"Following existing trajectory. Reason: {reason}")
            return self.last_planned_trajectory

        # --- REPLANNING IS NEEDED ---
        self.logger.info(f"Replanning triggered. Reason: {reason}")

        # Decide FROM WHERE to start the new plan (for splicing)
        replan_start_state, replan_index = self._get_replan_start_state(current_curvilinear_state)

        # Try to plan new optimal trajectory
        optimal_trajectory = self._plan(replan_start_state)
        
        # Process the new optimal trajectory
        if optimal_trajectory is not None:
            
            # Check if we have an old trajectory to splice with
            if self.last_planned_trajectory is not None:
                # --- Call C++ Splicing ---
                self.logger.debug(f"Splicing new trajectory at index {replan_index} using C++.")
                spliced_trajectory = self.handler.splice(
                    self.last_planned_trajectory, 
                    optimal_trajectory,     
                    replan_index
                )
                self.last_planned_trajectory = spliced_trajectory
                self.logger.debug("New trajectory planned, spliced, and cached")
            else:
                # This is the first plan, no splicing needed
                self.last_planned_trajectory = optimal_trajectory
                self.logger.debug("New (first) trajectory planned and cached")
        
        else:
            self.logger.warn("Replanning failed, keeping old trajectory as fallback.")
            # Cache is not updated, we return the old valid plan
        
        # Return the final C++ object
        return self.last_planned_trajectory











