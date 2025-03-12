#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__('ground_truth_odom')
        self.ground_truth_subscriber = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.gazebo_callback, 10
        )
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_br = TransformBroadcaster(self)
        dt_loop = 1/200
        self.create_timer(dt_loop, self.timer_callback)  # 10 Hz rate
        self.pos = None
        self.ori = None
        self.twist = None
        self.get_logger().info("Ground truth odometry node started.")

    def gazebo_callback(self, msg: ModelStates):
        # Find the index of the model
        try:
            index = msg.name.index("carver_description")
        except ValueError:
            self.get_logger().warn("carver_description not found in model states!")
            return  # Exit callback if model not found

        # Extract pose and twist data
        self.pos = msg.pose[index].position
        self.ori = msg.pose[index].orientation
        self.twist = msg.twist[index]  # Contains both linear and angular velocity

    def timer_callback(self):
        if self.pos is None or self.ori is None or self.twist is None:
            # Wait for data before publishing
            return

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Fill pose
        odom_msg.pose.pose.position = self.pos
        odom_msg.pose.pose.orientation = self.ori
        # odom_msg.pose.covariance = [0.0] * 36  # Optional: set covariance to zero

        # Fill twist (velocity)
        odom_msg.twist.twist.linear = self.twist.linear
        odom_msg.twist.twist.angular = self.twist.angular
        # odom_msg.twist.covariance = [0.0] * 36  # Optional: set covariance to zero

        # Publish odometry
        self.publisher.publish(odom_msg)

        # Publish TF from odom -> base_footprint
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.pos.x
        transform.transform.translation.y = self.pos.y
        transform.transform.translation.z = self.pos.z
        transform.transform.rotation = self.ori
        self.tf_br.sendTransform(transform)

        # Optional log for debugging
        # self.get_logger().info("Published ground truth odometry and TF.")

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
