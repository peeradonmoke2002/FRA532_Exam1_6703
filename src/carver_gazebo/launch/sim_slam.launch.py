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


    package_name = "carver_gazebo"
    package_name_world = "aws_robomaker_small_warehouse_world"
    package_name_controller = "carver_controller"
    package_name_slam = "carver_slam"
    package_name_odometry = "carver_odometry"
    package_name_urdf = "carver_description"

    world_file = "small_city.world"
    # world_file = "empty.world"
    # world_file = "no_roof_small_warehouse.world"
    gazebo_models_path = 'models'
    rviz_file_name = "mapping.rviz"

    spawn_x_val = "0.0"
    spawn_y_val = "0.0"
    spawn_z_val = "0.0"
    spawn_yaw_val = "0.0"

    
    # Paths
    rviz_file_path = os.path.join(get_package_share_directory(package_name_urdf), "rviz", rviz_file_name)
    # world_path = os.path.join(get_package_share_directory(package_name), "worlds", world_file)
    world_path = os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'), 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", "")

    config_mapping_file = os.path.join(get_package_share_directory(package_name_slam),
                                   'config', 'mapping_async.yaml')
    carver_odometry_ekf = os.path.join(get_package_share_directory(package_name_odometry), 'params','ekf.yaml')

    # Include Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name_urdf), "launch", "carver.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items()
    )




    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={'world': world_path}.items()
    )  

    merge_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("carver_controller"), "launch", "merge_lidar.launch.py")
        ),
        # launch_arguments={"use_sim_time": "false"}.items()
    )

    merge_imu_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("carver_controller"), "launch", "merge_imu.launch.py")
        )
    )
    # Spawn the robot at a specific location
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "carver_description",
            "-x", spawn_x_val,
            "-y", spawn_y_val,
            "-z", spawn_z_val,
            "-Y", spawn_yaw_val
        ],
        output="screen"
    )


    joint_state_publisher = Node(package='joint_state_publisher',
                                executable='joint_state_publisher',
                                name='joint_state_publisher',
                                output='screen')


    # slam_toolbox =  Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='async_slam_toolbox_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}, config_mapping_file])
    

    slam_toolbox_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
            '--ros-args',
            '-p', f'use_sim_time:=true',
            '--params-file', config_mapping_file
        ],
        output='screen'
    )

    robot_localization_node = Node(
         package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[carver_odometry_ekf]
    )

    print("GAZEBO_WORD_PATH = ", world_path)
    print("GAZEBO_MODEL_PATH = " + str(os.environ["GAZEBO_MODEL_PATH"]))
    # Controller Spawners

    controller = Node(
    	package=package_name_controller,
    	executable="ackermann_controller.py",
        name='ackermann_controller',
    )

    ackerman_yaw_rate_odom = Node(
        package='carver_odometry',
        executable='ackerman_yaw_rate_odom.py',
        name='ackerman_odom',
    )
    


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

   # Start RViz

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file_path],
    )

    set_contoller_manager_use_sim_time = ExecuteProcess(
    cmd=['ros2', 'param', 'set', '/controller_manager', 'use_sim_time', 'true'],
    output='screen')

     # Create LaunchDescription
    launch_description = LaunchDescription()

    # Start controllers in correct order
    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=spawn_entity,
    #             on_exit=[joint_state_broadcaster_spawner,
    #                      set_contoller_manager_use_sim_time],
    #         )
    #     )
    # )


    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster_spawner,
    #             on_exit=[position_controller_spawner],
    #         )
    #     )
    # )

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=position_controller_spawner,
    #             on_exit=[velocity_controller_spawner],
    #         )
    #     )
    # )

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=velocity_controller_spawner,
    #             on_exit=[rviz_node],
    #         )
    #     )
    # )

    # launch_description.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=rviz_node,
    #             on_exit=[ackerman_yaw_rate_odom],
    #         )
    #     )
    # )


    # Static Transform Publisher (world -> odom)
    static_tf = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )

    # Add launch actions
    launch_description.add_action(rviz_node)
    launch_description.add_action(gazebo)
    launch_description.add_action(spawn_entity)
    # launch_description.add_action(controller)
    launch_description.add_action(rsp)
    # launch_description.add_action(static_tf)
    # launch_description.add_action(set_contoller_manager_use_sim_time)

    # launch_description.add_action(merge_lidar_launch)
    # launch_description.add_action(merge_imu_launch)
    # launch_description.add_action(ackerman_yaw_rate_odom)
    # launch_description.add_action(slam_toolbox_process)
    # launch_description.add_action(robot_localization_node)

    return launch_description

    