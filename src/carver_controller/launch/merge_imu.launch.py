import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import launch_ros.actions


def generate_launch_description():

   
    merge_imu_launch = Node(
    	package="carver_controller",
    	executable="marge_two_imu.py"
    )


    launch_description = LaunchDescription()

 
    launch_description.add_action(merge_imu_launch)
   

    return launch_description