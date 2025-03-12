import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():


    mir_description_dir = get_package_share_directory('mir_description')
    mir_gazebo_dir = get_package_share_directory('mir_gazebo')
    robot_slam_package = "robot_slam"

    package_name_world = "aws_robomaker_small_warehouse_world"
    world_file = "no_roof_small_warehouse.world"
    gazebo_models_path = 'models'

    ### rviz ###
    rviz_config_file = "mapping.rviz"

    spawn_x_val = "0.0"
    spawn_y_val = "0.0"
    spawn_z_val = "0.0"
    spawn_yaw_val = "0.0"

    ## Paths ##
    ### use slam dir ###
    rviz_file_path = os.path.join(get_package_share_directory(robot_slam_package), "rviz", rviz_config_file)

    # world_path = os.path.join(get_package_share_directory(package_name), "worlds", world_file)
    world_path = os.path.join(get_package_share_directory(package_name_world), 'worlds', 'no_roof_small_warehouse', world_file)

    pkg_share = FindPackageShare(package=package_name_world).find(package_name_world)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", "")

    # Include Robot State Publisher
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={'world': world_path}.items()
    )  

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("robot_slam"), "launch", "mapping.launch.py")
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Spawn the robot at a specific location
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "mir_robot",
            "-x", spawn_x_val,
            "-y", spawn_y_val,
            "-z", spawn_z_val,
            "-Y", spawn_yaw_val
        ],
        output="screen"
    )
    
    ## Will use later ##

    # robot_localization_node = Node(
    #      package='robot_localization',
    #         executable='ekf_node',
    #         name='ekf_filter_node',
    #         output='screen',
    #         parameters=[carver_odometry_ekf]
    # )

    print("GAZEBO_WORD_PATH = ", world_path)
    print("GAZEBO_MODEL_PATH = " + str(os.environ["GAZEBO_MODEL_PATH"]))

    ## Controller Spawners ##

    # diffdrive_controller = Node(
    # 	package=package_name_controller,
    # 	executable="diffdrive_controller.py",
    #     name='diffdrive_controller',
    # )

   ## Start RViz ##
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file_path],
    )

     # Create LaunchDescription
    ld = LaunchDescription()


    # Add launch actions
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(rviz_node)
    ld.add_action(launch_mir_description)
    ld.add_action(launch_mir_gazebo_common)
    ld.add_action(slam_toolbox)


    return ld

    