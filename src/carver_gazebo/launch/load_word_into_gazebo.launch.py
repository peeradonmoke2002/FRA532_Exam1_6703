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
    world_file = "small_city.world"
    gazebo_models_path = 'models'


    # Paths
    world_path = os.path.join(get_package_share_directory(package_name), "worlds", world_file)
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", "")


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={'world': world_path}.items()
    )  
    print("GAZEBO_WORD_PATH = ", world_path)
    print("GAZEBO_MODEL_PATH = " + str(os.environ["GAZEBO_MODEL_PATH"]))

    # Create LaunchDescription
    ld = LaunchDescription()

   
    # Add launch actions
    ld.add_action(gazebo)

   

    return ld