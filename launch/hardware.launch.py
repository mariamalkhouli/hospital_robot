import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'hospital_robot'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. ROBOT DESCRIPTION (URDF)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()
    
    # 2. CONFIG FILES
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 3. NODES
    return LaunchDescription([
        # --- PHYSICAL ROBOT PUBLISHER ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # --- ACTUATORS ---
        Node(package=pkg_name, executable='motor_control', name='motor_control'),
        Node(package=pkg_name, executable='arduino_bridge', name='arduino_bridge'),
        
        # --- WEB INTERFACE ---
        Node(package=pkg_name, executable='web_server', name='web_server'),

        # --- SENSORS ---
        Node(package=pkg_name, executable='lidar_driver', name='lidar_driver'),
        Node(package=pkg_name, executable='imu_node', name='imu_node'),
        Node(package=pkg_name, executable='ultrasonic_node', name='ultrasonic_node'),
        Node(package=pkg_name, executable='cliff_detector', name='cliff_detector'),

        # --- ODOMETRY ---
        Node(package=pkg_name, executable='encoder_driver', name='encoder_driver'),
        Node(package=pkg_name, executable='wheel_odometry', name='wheel_odometry'),

        # --- SENSOR FUSION (EKF) ---
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        
        # --- VISUALIZATION BRIDGE (Foxglove) ---
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765, 'send_buffer_limit': 10000000}]
        )
    ])