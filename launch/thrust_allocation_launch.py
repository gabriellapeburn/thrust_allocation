import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    file_path = os.path.expanduser("~/robot_param.yaml")
    with open(file_path, "r") as stream:
        try:
            data_loaded = yaml.safe_load(stream)
            robot_namespace = '/'+ data_loaded['robot_namespace']

        except yaml.YAMLError as exc:
            print(exc)

    return LaunchDescription([
      Node(
            package='thrust_allocation',
            namespace= robot_namespace,
            executable='thrust_allocation_node',
            name='thrust_allocation',
            parameters=[
                {'Kp': 1}, 
                {'Kd ': 0.1}, 
                {'Ki': 0.001},
            ],
            remappings=[
                ('drive', 'platform/motors/cmd_drive'),
            ]
        ),
   ])