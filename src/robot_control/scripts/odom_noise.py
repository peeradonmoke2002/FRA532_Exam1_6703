#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import tf_transformations
import numpy as np

class OdomNoise(Node):
    def __init__(self):
        super().__init__('odom_noise')
        queue_size = 10
        self.publisher = self.create_publisher(Odometry, '/odom', queue_size)
        self.subscriber = self.create_subscription(Odometry, '/odom_diff', self.odom_callback, queue_size)
        self.timer = self.create_timer(1/100.0, self.timer_callback)  # 100 Hz update rate
        self.last_odom = None

        # Noise parameters (adjust these for "much noise")
        self.noise_std_pos = 1.0    # Standard deviation for x,y position noise
        self.noise_std_yaw = 0.5    # Standard deviation for yaw noise
        self.noise_std_lin = 0.5    # Standard deviation for linear velocity noise
        self.noise_std_ang = 0.2    # Standard deviation for angular velocity noise

    def odom_callback(self, msg):
        # Store the latest odometry message from /odom_diff
        self.last_odom = msg

    def timer_callback(self):
        if self.last_odom is None:
            return

        new_odom = Odometry()
        new_odom.header = self.last_odom.header
        new_odom.child_frame_id = self.last_odom.child_frame_id

        # --- Process and add noise to the Pose ---
        orig_pose = self.last_odom.pose.pose

        # Add noise to the position (x and y)
        noisy_pose = Pose()
        noisy_pose.position.x = orig_pose.position.x + np.random.normal(0, self.noise_std_pos)
        noisy_pose.position.y = orig_pose.position.y + np.random.normal(0, self.noise_std_pos)
        noisy_pose.position.z = orig_pose.position.z  # leaving z unchanged

        # Convert the original orientation quaternion to Euler angles
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([
            orig_pose.orientation.x,
            orig_pose.orientation.y,
            orig_pose.orientation.z,
            orig_pose.orientation.w
        ])
        # Add noise to the yaw angle
        noisy_yaw = yaw + np.random.normal(0, self.noise_std_yaw)
        new_quat = tf_transformations.quaternion_from_euler(roll, pitch, noisy_yaw)
        noisy_pose.orientation.x = new_quat[0]
        noisy_pose.orientation.y = new_quat[1]
        noisy_pose.orientation.z = new_quat[2]
        noisy_pose.orientation.w = new_quat[3]

        new_odom.pose.pose = noisy_pose

        # --- Process and add noise to the Twist ---
        orig_twist = self.last_odom.twist.twist
        noisy_twist = Twist()
        noisy_twist.linear.x = orig_twist.linear.x + np.random.normal(0, self.noise_std_lin)
        noisy_twist.linear.y = orig_twist.linear.y + np.random.normal(0, self.noise_std_lin)
        noisy_twist.linear.z = orig_twist.linear.z
        noisy_twist.angular.x = orig_twist.angular.x
        noisy_twist.angular.y = orig_twist.angular.y
        noisy_twist.angular.z = orig_twist.angular.z + np.random.normal(0, self.noise_std_ang)
        new_odom.twist.twist = noisy_twist

        # --- Set (high) covariance matrices to reflect increased uncertainty ---
        # These values are arbitrary—tweak them based on how "bad" you want the odometry to be.
        pose_cov = np.diag([2.0, 2.0, 0.1, 0.5, 0.5, 0.5])
        twist_cov = np.diag([1.0, 1.0, 0.1, 0.5, 0.5, 0.5])
        new_odom.pose.covariance = pose_cov.flatten().tolist()
        new_odom.twist.covariance = twist_cov.flatten().tolist()

        self.publisher.publish(new_odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomNoise()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
