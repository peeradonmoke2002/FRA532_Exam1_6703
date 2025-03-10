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

    package_name = "carver_gazebo"
    package_name_urdf = "carver_description"
    world_file = "small_city.world"
    # world_file = "empty.world"

    gazebo_models_path = 'models'
    rviz_file_name = "merge_lidar.rviz"

    spawn_x_val = "0.0"
    spawn_y_val = "0.0"
    spawn_z_val = "0.0"
    spawn_yaw_val = "0.0"

    
    # Paths
    rviz_file_path = os.path.join(get_package_share_directory(package_name_urdf), "rviz", rviz_file_name)
    world_path = os.path.join(get_package_share_directory(package_name), "worlds", world_file)
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", "")

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
        )
    )


    # Spawn the robot at a specific location
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "carver_description",
            '-timeout', '120.0',
            "-x", spawn_x_val,
            "-y", spawn_y_val,
            "-z", spawn_z_val,
            "-Y", spawn_yaw_val
        ],
        output="screen"
    )

    print("GAZEBO_WORD_PATH = ", world_path)
    print("GAZEBO_MODEL_PATH = " + str(os.environ["GAZEBO_MODEL_PATH"]))
    # Controller Spawners

    controller = Node(
    	package="carver_controller",
    	executable="ackermann_controller.py"
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
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file_path],
        output="screen"
    )

     # Create LaunchDescription
    launch_description = LaunchDescription()

    # Start controllers in correct order
    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[position_controller_spawner],
            )
        )
    )

    launch_description.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=position_controller_spawner,
                on_exit=[velocity_controller_spawner],
            )
        )
    )




    # Add launch actions
    launch_description.add_action(rviz)
    launch_description.add_action(gazebo)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(controller)
    launch_description.add_action(rsp)
    launch_description.add_action(merge_lidar_launch)
   

    return launch_description