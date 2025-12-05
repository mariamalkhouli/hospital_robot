import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'hospital_robot'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Include the Hardware Launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hardware.launch.py')
        )
    )

    # 2. SLAM Config
    slam_config = os.path.join(pkg_share, 'config', 'mapper_params.yaml')

    # 3. SLAM Node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config]
    )

    return LaunchDescription([
        hardware_launch,
        slam_node
    ])