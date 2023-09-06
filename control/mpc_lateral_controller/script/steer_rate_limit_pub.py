import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_localization_msgs.msg import KinematicState
from time import sleep
import numpy as np

class ArrayPublisherNode(Node):

    def __init__(self):
        super().__init__('steer_rate_limit_publisher')
        self.publisher_ = self.create_publisher(AckermannControlCommand, 'steer_rate_limit', 1)

        self.subscription = self.create_subscription(KinematicState, 'ego_velocity', self.ego_velocity_callback, 1)


        # Define arrays
        self.stere_rate_dps = [11.25, 11.25,  9.  ,  6.75,  4.5 ,  2.25,  1.5 ,  1.5 ,  1.5]  # steering angle rate limit list depending on velocity [deg/s] [72, 72, 36, 18, 9, 3.6, 3.6]
        self.vel = [0.0,   1.388,  2.778,  4.167,  5.556,   6.944,  8.333,  9.722,  11.11] # velocity list for steering angle rate limit interpolation in ascending order [m/s]
        self.ego_velocity = 0.0

        # Verify both arrays have the same length
        assert len(self.stere_rate_dps) == len(self.vel), "Arrays r and v must have the same length"

        # Start publishing
        self.timer = self.create_timer(0.3, self.publish_array)

        # Define current index and direction
        self.index = 0
        self.direction = 1 # 1 for forward, -1 for backward

    def ego_velocity_callback(self, msg):
        self.ego_velocity = msg.twist_with_covariance.twist.twist.linear.x

    def publish_array(self):
        msg = AckermannControlCommand()
        msg.lateral.steering_tire_rotation_rate = self.stere_rate_dps[self.index] * 3.1415 / 180.0 * self.direction
        msg.longitudinal.speed = self.vel[self.index]
        msg.lateral.steering_tire_angle = np.interp(self.ego_velocity, self.vel, self.stere_rate_dps) * 3.1415 / 180.0 * self.direction
        self.publisher_.publish(msg)

        # Update index and check direction
        if self.index == len(self.stere_rate_dps) - 1:
            if self.direction == -1:
                self.index += self.direction
            else:
                self.direction = -1
        elif self.index == 0:
            if self.direction == 1:
                self.index += self.direction
            else:
                self.direction = 1
        else:
            self.index += self.direction

        

        

def main(args=None):
    rclpy.init(args=args)
    node = ArrayPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
