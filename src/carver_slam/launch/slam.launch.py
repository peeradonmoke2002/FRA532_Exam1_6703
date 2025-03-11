import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, \
    SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config_dir = os.path.join(get_package_share_directory('carver_description'), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'gazebo.rviz')
    
    merge_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("carver_controller"), "launch", "merge_lidar.launch.py")
        )
    )

    merge_imu_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("carver_controller"), "launch", "merge_imu.launch.py")
        )
    )


    controller = Node(
    	package="carver_controller",
    	executable="ackermann_controller.py",
        name="ackermann_controller",
        output="screen"
    )

    ackerman_yaw_rate_odom = Node(
    	package="carver_odometry",
    	executable="ackerman_yaw_rate_odom.py",
        name="ackerman_yaw_rate_odom",
        output="screen"
    )

    carver_odometry = get_package_share_directory('carver_odometry')
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(carver_odometry, 'params','ekf.yaml')]
    )

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("carver_slam"),
                                   'config', 'mapping_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    launch_mapping = Node(
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}])
    

    ld = LaunchDescription()

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=controller,
                on_exit=[ackerman_yaw_rate_odom],
            )
        )
    )


    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ackerman_yaw_rate_odom,
                on_exit=[robot_localization_node],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_localization_node,
                on_exit=[launch_mapping],
            )
        )
    )


    # ld.add_action(robot_localization_node)
    ld.add_action(merge_lidar_launch)
    ld.add_action(merge_imu_launch)
    ld.add_action(controller)
    # ld.add_action(ackerman_yaw_rate_odom)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(rviz)
    # ld.add_action(launch_mapping)

    return ld