import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'hospital_robot'
    
    # 1. PATHS
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # 2. PARSE URDF
    doc = xacro.process_file(urdf_file)
    robot_desc = doc.toxml()

    # 3. DEFINE NODES

    # Robot State Publisher (Publishes the physical structure / TFs)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Motor Control Node
    motor_node = Node(
        package=pkg_name,
        executable='motor_control', # Ensure this matches setup.py entry_point
        name='motor_control',
        output='screen'
    )

    # Ultrasonic Sensors Node
    ultrasonic_node = Node(
        package=pkg_name,
        executable='ultrasonic_4x_node',
        name='ultrasonic_node',
        output='screen',
        parameters=[{'serial_port': '/dev/ttyAMA0'}]
    )

    # LiDAR Driver (Using the package you cloned: ldlidar_stl_ros2)
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ld06_node',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD06',
            'topic_name': 'scan',
            'frame_id': 'lidar_link',   # Matches URDF
            'port_name': '/dev/ttyUSB0', # Check if your Lidar is on USB0 or USB1
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False
        }]
    )

    # Web Server (Flask App)
    web_node = Node(
        package=pkg_name,
        executable='web_server',
        name='web_server',
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        motor_node,
        ultrasonic_node,
        lidar_node,
        web_node
    ])