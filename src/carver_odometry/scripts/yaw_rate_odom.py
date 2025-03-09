#!/usr/bin/python3

from carver_odometry.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node


class YawRateOdom(Node):
    def __init__(self):
        super().__init__('yaw_rate_odom')

def main(args=None):
    rclpy.init(args=args)
    node = YawRateOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
