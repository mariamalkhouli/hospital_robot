import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'hospital_robot'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. PATHS
    nav_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'hospital_map.yaml')

    # 2. Include Hardware Launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hardware.launch.py')
        )
    )

    # 3. Navigation Nodes
    lifecycle_nodes = [
        'map_server', 
        'amcl', 
        'controller_server', 
        'planner_server', 
        'recoveries_server', 
        'bt_navigator', 
        'waypoint_follower'
    ]

    return LaunchDescription([
        # Start Hardware
        hardware_launch,

        # --- NAVIGATION STACK ---
        
        # Load Map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav_config, {'yaml_filename': map_file}]
        ),
        
        # Localization (AMCL)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav_config]
        ),
        
        # Path Controller (Local)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav_config]
        ),
        
        # Path Planner (Global)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav_config]
        ),
        
        # Recoveries (Spin/Backup)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav_config]
        ),
        
        # Behavior Tree Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav_config]
        ),
        
        # Lifecycle Manager (Wake everything up)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': lifecycle_nodes}
            ]
        )
    ])