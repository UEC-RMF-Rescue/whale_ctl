from launch import LaunchDescription
from launch_ros.action import Node
from launch_ros.action import PushRosNamespace
from launch.action import ExecuteProcess, SetEnvironmentVariable
from launch.action import DeclearLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.action import IncludeLaunchDescription
from launch.launch_description_source import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = whale_ctl

    actuator_driver = Node(
        package = pkg_name,
        executable = "actuator_driver"
        name = "actuator"
        namespace = "whale"
    )


