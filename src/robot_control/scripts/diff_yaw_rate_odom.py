#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Point, Pose, Quaternion, 
                               Vector3, TransformStamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Header, Float32
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import numpy as np

class DiffDriveOdomYawRate(Node):
    def __init__(self):
        super().__init__('diff_drive_odom_yaw_rate')
        queue_size = 10
        self.dt_loop = 1 / 100.0  # 100 Hz update rate

        # Mir Robot parameters
        self.track_width = 0.445208  
        self.wheel_radius = 0.0625
        # Publishers and timer
        self.publisher = self.create_publisher(Odometry, 'yawrate/odom', queue_size)
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)

        # Subscribers
        self.subscription_joinstate = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, queue_size)
        self.subscription_imu = self.create_subscription(
            Imu, '/imu_data/data', self.feedback_imu, queue_size)

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
        self.imu_yaw_rate = 0.0  # Yaw rate from the IMU
        self.relative_yaw = 0.0  # Change in yaw from the initial IMU reading
        self.wheelspeed = 0.0
        self.wheel_omega = np.array([0.0, 0.0])
        self.ax = 0.0
        self.last_callback_time = self.get_clock().now()
        self.initial_orientation = None  # To store the initial IMU yaw

    def joint_state_callback(self, msg):
        if ('left_wheel_joint' in msg.name and 'right_wheel_joint' in msg.name):
            left_index = msg.name.index('left_wheel_joint')
            right_index = msg.name.index('right_wheel_joint')
            # print(f"left_wheel_joint - Position: {msg.position[left_index]}, Velocity: {msg.velocity[left_index]}")
            # print(f"right_wheel_joint - Position: {msg.position[right_index]}, Velocity: {msg.velocity[right_index]}")
            # Get rear wheel velocities (convert encoder rate to linear velocity)
            v_rl = msg.velocity[left_index] * self.wheel_radius
            v_rr = msg.velocity[right_index] * self.wheel_radius
            self.wheel_omega = np.array([v_rl, v_rr])
        # else:
        #     print('Joint names not found in message')


    def get_wheel_speed(self, wheel_omega: np.ndarray) -> float:
        # Convert average wheel angular speed to linear speed
        wheelspeed = ((wheel_omega[0] + wheel_omega[1]) / 2.0)
        return wheelspeed
    

    def feedback_imu(self, msg):
        # Process orientation (for updating yaw if needed)
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        if orientation_list == [0.0, 0.0, 0.0, 0.0]:
            return  # Skip invalid data

        _, _, yaw = tf_transformations.euler_from_quaternion(orientation_list)
        if self.initial_orientation is None:
            self.initial_orientation = yaw
        self.relative_yaw = yaw - self.initial_orientation

        # Directly get the yaw rate from the IMU
        self.imu_yaw_rate = msg.angular_velocity.z

        # Optional: Store acceleration if needed
        self.ax = msg.linear_acceleration.x

    def timer_callback(self):
        self.wheelspeed = self.get_wheel_speed(self.wheel_omega)
        # Use the relative yaw from the IMU for orientation
        yaw = self.relative_yaw
        vx = self.wheelspeed * math.cos(yaw)
        vy = self.wheelspeed * math.sin(yaw)
        # Update robot position in the plane
        self.robot_position[0] += vx * self.dt_loop
        self.robot_position[1] += vy * self.dt_loop
        
        # Optionally, update the robot's yaw if you want to integrate the IMU yaw over time
        # For example:
        # self.robot_position[2] += self.imu_yaw_rate * self.dt_loop

        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        
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
        odom_msg.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        # Use the IMU yaw rate for the angular component, not the yaw angle
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=yaw)
        
        self.publisher.publish(odom_msg)
        
        # Publish the transform
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation

        # self.tf_br.sendTransform(transform)

        # debug # 
        # print('x:', np.round(self.robot_position[0], 3), 
        #       'y:', np.round(self.robot_position[1], 3), 
        #       'yaw:', np.round(yaw, 3))



def main(args=None):
    rclpy.init(args=args)
    pub_odom_node = DiffDriveOdomYawRate()
    rclpy.spin(pub_odom_node)
    pub_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()