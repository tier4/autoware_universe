#!/usr/bin/python3
"""This module handles object detection processing from ROSBags
Objects are processed as current frame detections without historical tracking
"""
from queue import Queue
import numpy as np
from typing import Dict, List, Optional

from autoware_perception_msgs.msg import TrackedObjects, TrackedObject, Shape
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Twist, Polygon, Point32

def create_transform_matrix(position, orientation) -> np.ndarray:
    """Convert position and quaternion to 4x4 transform matrix"""
    from scipy.spatial.transform import Rotation
    
    # Create rotation matrix from quaternion
    r = Rotation.from_quat([
        orientation.x, 
        orientation.y, 
        orientation.z, 
        orientation.w
    ])
    rotation_matrix = r.as_matrix()
    
    # Create 4x4 transform matrix
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = [position.x, position.y, position.z]
    
    return transform

def extract_velocity_vector(twist: Twist) -> np.ndarray:
    """Extract 3D velocity vector from Twist message"""
    return np.array([
        twist.linear.x,
        twist.linear.y, 
        twist.linear.z
    ])

def extract_object_footprint(shape: Shape) -> np.ndarray:
    """Extract object footprint from shape message"""
    if shape.type == Shape.BOUNDING_BOX:
        # Convert box to polygon points
        length = shape.dimensions.x
        width = shape.dimensions.y
        
        # Create box corners in local coordinates
        corners = np.array([
            [length/2, width/2, 0],
            [length/2, -width/2, 0],
            [-length/2, -width/2, 0],
            [-length/2, width/2, 0]
        ])
        return corners
    elif shape.type == Shape.POLYGON:
        # Convert polygon points to numpy array
        points = []
        for point in shape.footprint.points:
            points.append([point.x, point.y, 0])
        return np.array(points)
    else:
        # Default to simple point
        return np.array([[0, 0, 0]])

class CurrentFrameObject:
    """Represents a single object in current frame"""
    def __init__(self, obj: TrackedObject):
        # Basic properties
        self.uuid = ''.join([chr(x) for x in obj.object_id.uuid])
        self.object_type = obj.classification[0].label if obj.classification else "UNKNOWN"
        
        # Pose and motion
        self.transform = create_transform_matrix(
            obj.kinematics.initial_pose_with_covariance.pose.position,
            obj.kinematics.initial_pose_with_covariance.pose.orientation
        )
        self.velocity = extract_velocity_vector(
            obj.kinematics.initial_twist_with_covariance.twist
        )

        ## The velocity is in the local frame of the object, so we need to transform it to the global frame
        self.velocity = self.transform[:3, :3] @ self.velocity

        
        # Shape information
        self.footprint = extract_object_footprint(obj.shape)
        # Transform footprint to global coordinates
        self.global_footprint = self._transform_footprint()
        
    def _transform_footprint(self) -> np.ndarray:
        """Transform footprint points to global coordinates"""
        # Add homogeneous coordinate
        points_h = np.hstack([self.footprint, np.ones((len(self.footprint), 1))])
        # Transform points
        global_points_h = points_h @ self.transform.T
        # Return 3D points
        return global_points_h[:, :3]

class BaseTracker:
    def __init__(self, trace_back_step=10):
        self.trace_back_step = trace_back_step
        self.buffer_size = trace_back_step
        
        # Ego vehicle state history (still needed for trajectory extraction)
        self.ego_transforms: Queue = Queue(maxsize=self.buffer_size)
        self.ego_velocities: Queue = Queue(maxsize=self.buffer_size)
        self.ego_timestamps: Queue = Queue(maxsize=self.buffer_size)
        
        self.current_timestamp: Optional[float] = None


    def step_ego_pose(self, msg: Odometry):
        """Update ego vehicle state"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Create transform matrix
        transform = create_transform_matrix(
            msg.pose.pose.position,
            msg.pose.pose.orientation
        )
        
        # Extract velocity
        velocity = extract_velocity_vector(msg.twist.twist)
        
        # Update queues
        if self.ego_transforms.full():
            self.ego_transforms.get()
            self.ego_velocities.get()
            self.ego_timestamps.get()
            
        self.ego_transforms.put(transform)
        self.ego_velocities.put(velocity)
        self.ego_timestamps.put(timestamp)

    def step_key_frame(self, keyframe):
        """Generate output for key frame"""
        # Convert queues to lists for easier slicing
        ego_transforms = list(self.ego_transforms.queue)
        ego_velocities = list(self.ego_velocities.queue)
        ego_timestamps = list(self.ego_timestamps.queue)
        

            
        return {
            "timestamp": self.current_timestamp,
            "ego_history": {
                "transforms": ego_transforms,
                "velocities": ego_velocities,
                "timestamps": ego_timestamps
            },
        }