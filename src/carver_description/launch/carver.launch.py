#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro    
from launch.actions import DeclareLaunchArgument
    
# for open robot_state_publisher
def generate_launch_description():
    
    pkg = get_package_share_directory('carver_description')
    
    path_description = os.path.join(pkg,'urdf','carver_core.urdf.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()
    #robot_desc_xml = xacro.process_file(path_description,mappings={'robot_name': namespace}).toxml()
    
    parameters = [{'robot_description':robot_desc_xml}]
    #parameters.append({'frame_prefix':namespace+'/'})
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output="both",
                                  parameters=parameters
    )
    


    launch_description = LaunchDescription()
    
    launch_description.add_action(robot_state_publisher)
 
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        robot_state_publisher
    ])