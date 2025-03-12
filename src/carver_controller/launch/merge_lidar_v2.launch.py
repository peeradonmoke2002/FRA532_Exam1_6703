from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='ira_laser_tools',
            name='carver_laser_scan_merger',
            executable='laserscan_multi_merger',
            parameters=[{'laserscan_topics': "scan_LB scan_RF",
                         'destination_frame': "laser",
                         'scan_destination_topic': "scan",
                         'cloud_destination_topic': "scan_cloud",
                         'min_height': -0.25,
                         'max_completion_time': 0.05,
                         'max_merge_time_diff': 0.005,
                         'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'best_effort': True}],
            namespace=namespace,    # adds namespace to topic names and frames
            output='screen')
    ])
