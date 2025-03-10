#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf_transformations as tf_trans

class MergeTwoIMU(Node):
    def __init__(self):
        super().__init__('merge_two_imu')
        # Create subscribers for the two IMU topics
        self.imu_left_sub = Subscriber(self, Imu, 'imu_L')
        self.imu_right_sub = Subscriber(self, Imu, 'imu_R')

        # Approximate time synchronizer (adjust queue_size and slop as needed)
        self.ts = ApproximateTimeSynchronizer(
            [self.imu_left_sub, self.imu_right_sub],
            queue_size=10,
            slop=0.05)
        self.ts.registerCallback(self.imu_callback)

        # Publisher for the merged IMU message
        self.merged_pub = self.create_publisher(Imu, 'imu', 10)
        self.get_logger().info('MergeTwoIMU node started')

    def imu_callback(self, imu_left: Imu, imu_right: Imu):
        merged_imu = Imu()

        # Set header
        merged_imu.header.stamp = self.get_clock().now().to_msg()
        merged_imu.header.frame_id = imu_left.header.frame_id

        # Average angular velocity
        merged_imu.angular_velocity.x = (imu_left.angular_velocity.x + imu_right.angular_velocity.x) / 2.0
        merged_imu.angular_velocity.y = (imu_left.angular_velocity.y + imu_right.angular_velocity.y) / 2.0
        merged_imu.angular_velocity.z = (imu_left.angular_velocity.z + imu_right.angular_velocity.z) / 2.0

        # Average linear acceleration
        merged_imu.linear_acceleration.x = (imu_left.linear_acceleration.x + imu_right.linear_acceleration.x) / 2.0
        merged_imu.linear_acceleration.y = (imu_left.linear_acceleration.y + imu_right.linear_acceleration.y) / 2.0
        merged_imu.linear_acceleration.z = (imu_left.linear_acceleration.z + imu_right.linear_acceleration.z) / 2.0

        # Average orientation using tf_transformations quaternion_slerp.
        # Convert quaternion messages to lists [x, y, z, w]
        q_left = [imu_left.orientation.x,
                  imu_left.orientation.y,
                  imu_left.orientation.z,
                  imu_left.orientation.w]
        q_right = [imu_right.orientation.x,
                   imu_right.orientation.y,
                   imu_right.orientation.z,
                   imu_right.orientation.w]

        # Compute the slerp with interpolation factor 0.5
        q_merged = tf_trans.quaternion_slerp(q_left, q_right, 0.5)
        merged_imu.orientation.x = q_merged[0]
        merged_imu.orientation.y = q_merged[1]
        merged_imu.orientation.z = q_merged[2]
        merged_imu.orientation.w = q_merged[3]

        # Optionally, copy over covariance from one of the messages
        merged_imu.orientation_covariance = imu_left.orientation_covariance
        merged_imu.angular_velocity_covariance = imu_left.angular_velocity_covariance
        merged_imu.linear_acceleration_covariance = imu_left.linear_acceleration_covariance

        # Publish merged IMU message
        self.merged_pub.publish(merged_imu)

def main(args=None):
    rclpy.init(args=args)
    node = MergeTwoIMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
