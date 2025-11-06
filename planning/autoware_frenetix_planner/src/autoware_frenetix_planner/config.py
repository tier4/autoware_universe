from dataclasses import dataclass
from typing import List, Optional

@dataclass
class DictLikeDataClass:
    """
    Base class to provide dictionary-like access to dataclass fields.
    """
    def __getitem__(self, key):
        return getattr(self, key)
    
    def __contains__(self, key):
        return hasattr(self, key)

    def keys(self):
        return self.__dataclass_fields__.keys()
    
@dataclass
class CartesianState(DictLikeDataClass):
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
class CurvilinearState(DictLikeDataClass):
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


@dataclass
class EvasiveParams(DictLikeDataClass):
    """Holds all evasive maneuver parameters."""
    acc_min: float
    jerk_acc: float
    jerk_dec: float
    stop_buffer: float
    velocity_threshold: float
    release_longitudinal_back: float
    release_lateral_center: float

@dataclass
class CostWeightsParams(DictLikeDataClass):
    """Holds all cost function weights."""
    acceleration: float
    jerk: float
    lateral_jerk: float
    longitudinal_jerk: float
    orientation_offset: float
    lane_center_offset: float
    distance_to_reference_path: float
    prediction: float
    distance_to_obstacles: float
    velocity_offset: float
    positive_velocity_offset: float
    negative_velocity_offset: float
    cartesian_lateral_acceleration: float

@dataclass
class FrenetixPlannerParams(DictLikeDataClass):
    """Main configuration for the Frenetix motion planner."""
    planning_horizon: float
    sampling_dt: float
    t_min: float
    d_min: float
    d_max: float
    sampling_min: int
    sampling_max: int
    velocity_limit: float
    desired_velocity: float
    delta_max: float
    wheelbase: float
    v_switch: float
    a_max: float
    v_delta_max: float
    length: float
    width: float
    wb_rear_axle: float
    v_max: float
    cost_weights: CostWeightsParams
    evasive: EvasiveParams
  