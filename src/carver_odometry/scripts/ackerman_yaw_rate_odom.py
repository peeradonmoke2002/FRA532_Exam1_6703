#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion, 
                               Twist, TwistWithCovariance, Vector3, TransformStamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Header, Float32
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import numpy as np

class AckermannYawRateOdom(Node):
    def __init__(self):
        super().__init__('ackermann_yaw_rate_odom')
        queue_size = 10
        self.dt_loop = 1 / 200.0  # 100 Hz update rate

        # Robot parameters
        self.wheel_base = 1.27196        
        self.wheel_radius = 0.175   
        self.track_width = 0.79085   

        # Publishers and timer
        self.publisher = self.create_publisher(Odometry, 'odom', queue_size)
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)

        # Subscribers
        self.subscription_joinstate = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, queue_size)
        self.subscription_imu = self.create_subscription(
            Imu, '/imu', self.feedback_imu, queue_size)

        # Transform broadcaster
        self.tf_br = TransformBroadcaster(self)

        # Covariance matrices (optional – currently commented out in messages)
        self.pose_cov = np.diag([1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])
        self.twist_cov = np.diag([1.0e-9, 1.0e-6, 1.0e-9, 1.0e-9, 1.0e-9, 1.0e-9])
        
        # Spawn pose: Adjust these values so they match your simulation’s initial pose if needed.
        spawn_x_val = 0.0
        spawn_y_val = 0.0
        spawn_yaw_val = 0.0  # In radians; note that this should be consistent with your IMU's frame.
        self.robot_position = np.array([spawn_x_val, spawn_y_val, spawn_yaw_val])
        self.orientation = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_position[2])

        # Odometry state variables
        self.relative_yaw = 0.0  # Change in yaw from the initial IMU reading
        self.absolute_yaw = 0.0  # Absolute yaw for integrating the robot’s position
        self.wheelspeed = 0.0
        self.wheel_omega = np.array([0.0, 0.0])
        self.steering_angles = np.array([0.0, 0.0])
        self.ax = 0.0
        self.last_callback_time = self.get_clock().now()
        self.initial_orientation = None  # To store the initial IMU yaw

    def joint_state_callback(self, msg):
        if ('base_rear_left_wheel_joint' in msg.name and 'base_rear_right_wheel_joint' in msg.name):

            left_wheel_index = msg.name.index('base_rear_left_wheel_joint')
            right_wheel_index = msg.name.index('base_rear_right_wheel_joint')

            # Get rear wheel velocities (convert encoder rate to linear velocity)
            v_rl = msg.velocity[left_wheel_index] * self.wheel_radius
            v_rr = msg.velocity[right_wheel_index] * self.wheel_radius
            self.wheel_omega = np.array([-v_rl, v_rr])


    def get_wheel_speed(self, wheel_omega: np.ndarray) -> float:
        # Convert average wheel angular speed to linear speed
        wheelspeed = ((wheel_omega[0] + wheel_omega[1]) / 2.0)
        return wheelspeed
    

    def feedback_imu(self, msg):
        # Get orientation from IMU message and convert quaternion to Euler yaw
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        if orientation_list == [0.0, 0.0, 0.0, 0.0]:
            return  # skip invalid data
        
        _, _, yaw = tf_transformations.euler_from_quaternion(orientation_list)
        if self.initial_orientation is None:
            self.initial_orientation = yaw
        self.relative_yaw = yaw - self.initial_orientation
        self.ax = msg.linear_acceleration.x

    def timer_callback(self):
        self.wheelspeed = self.get_wheel_speed(self.wheel_omega)
        # Update absolute yaw (here, robot_position[2] remains initial yaw; consider integrating angular velocity if needed)
        self.absolute_yaw = self.robot_position[2] + self.relative_yaw
        vx = self.wheelspeed * math.cos(self.absolute_yaw)
        vy = self.wheelspeed * math.sin(self.absolute_yaw)
        # Update robot position in the plane
        self.robot_position[0] += vx * self.dt_loop
        self.robot_position[1] += vy * self.dt_loop

        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.absolute_yaw)

        # Populate the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = Pose(
            position=Point(
                x=self.robot_position[0],
                y=self.robot_position[1],
                z=0.0
            ),
            orientation=Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3]
            )
        )
        # odom_msg.pose.covariance = self.pose_cov.flatten().tolist()
        odom_msg.twist.twist.linear = Vector3(x=vx, y=0.0, z=0.0)
        # odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.absolute_yaw)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        # odom_msg.twist.covariance = self.twist_cov.flatten().tolist()

        self.publisher.publish(odom_msg)
        
        # Publish TF transform from odom to base_footprint
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_br.sendTransform(transform)

        # print('x:', np.round(self.robot_position[0], 3), 
        #       'y:', np.round(self.robot_position[1], 3), 
        #       'yaw:', np.round(self.absolute_yaw, 3))
def main(args=None):
    rclpy.init(args=args)
    pub_odom_node = AckermannYawRateOdom()
    rclpy.spin(pub_odom_node)
    pub_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()