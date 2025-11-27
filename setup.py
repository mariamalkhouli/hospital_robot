import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'hospital_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copy the launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.py'))), 
        # Copy config files
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))), 
                 (os.path.join('share', package_name, 'urdf'), 
         glob(os.path.join('urdf', '*'))),
        # Copy the templates (*.html files only)
        (os.path.join('share', package_name, 'templates'), 
          glob(os.path.join('templates', '*.html'))),
          
        # --- CRITICAL FIX: Explicitly copy files within static subdirectories ---
        # This prevents the build system from trying to copy the *directories* (like 'css') themselves,
        # which was causing the "not a regular file" error.
        (os.path.join('share', package_name, 'static', 'css'), 
          glob(os.path.join('static', 'css', '*'))),
        (os.path.join('share', package_name, 'static', 'js'), 
          glob(os.path.join('static', 'js', '*'))),
        (os.path.join('share', package_name, 'static', 'img'), 
          glob(os.path.join('static', 'img', '*'))),
          
        # Copy maps
        (os.path.join('share', package_name, 'maps'), 
         glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools', 'flask', 'Pillow', 'gpiozero'], # Added gpiozero
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='Hospital Delivery Robot Package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control = hospital_robot.motor_control:main',
            'web_server = hospital_robot.app:main',
            'encoder_driver = hospital_robot.encoder_driver_node:main',
            'ultrasonic_4x_node = hospital_robot.ultrasonic_4x_node:main',
            'imu_node = hospital_robot.imu_node:main',
            # --- New Entry Points to create later ---
            'battery_manager = hospital_robot.battery_manager:main',
            'dispenser_control = hospital_robot.dispenser_control:main',
            'mission_planner = hospital_robot.mission_planner:main', # For sequencing delivery/charge steps
        ],
    },
)