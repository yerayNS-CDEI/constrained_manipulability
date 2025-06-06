import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Abstract robot launch
    abstract_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('constrained_manipulability'), 'launch'),
            '/abstract_robot.launch.py']),
        launch_arguments={
            'root_col': 'collision_base_link',
            'tip_col': 'collision_tool0',
            'root': 'base_link',
            'tip': 'tool0'}.items()
    )

    return LaunchDescription([
        abstract_robot_launch,
    ])