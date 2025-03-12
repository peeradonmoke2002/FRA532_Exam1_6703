import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
import launch_ros.actions
from launch.conditions import IfCondition, UnlessCondition
import xacro

def generate_launch_description():
    warehouse_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')

    # Add Here
    mir_description_dir = get_package_share_directory('mir_description')
    mir_gazebo_dir = get_package_share_directory('mir_gazebo')


    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/no_roof_small_warehouse.launch.py'])
    )

    # Add Here
    launch_mir_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'mir_launch.py')
        )
    )

    launch_mir_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_gazebo_dir, 'launch',
                         'include', 'mir_gazebo_common.py')
        )
    )

    merge_laser_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("ros2_laser_scan_merger"), "launch", "merge_2_scan.launch.py")
        )
    )

    # launch_teleop = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     namespace='',
    #     output='screen',
    #     prefix='xterm -e')

    diff_yaw_rate_odom = Node(
        package='robot_control',
        executable='diff_yaw_rate_odom.py',
        namespace='',
        output='screen'
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mir_robot',
                '-topic', 'robot_description',
                '-b'],
        namespace='',
        output='screen')

    static_tf = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )



    ld = LaunchDescription()
    # Start controllers in correct order
    # ld.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=spawn_robot,
    #             on_exit=[joint_state_broadcaster_spawner],
    #         )
    #     )
    # )

    # ld.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster_spawner,
    #             on_exit=[velocity_controller_spawner],
    #         )
    #     )
    # )

    # ld.add_action(diff_yaw_rate_odom)
    ld.add_action(warehouse_world_cmd)
    ld.add_action(launch_mir_description)
    ld.add_action(launch_mir_gazebo_common)
    ld.add_action(spawn_robot)

    return ld