from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('falcon_ros')) / 'config/safety.yaml')
    return LaunchDescription([Node(package='falcon_ros', executable='safety_bridge.py', output='screen', parameters=[config])])
