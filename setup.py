import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'hospital_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        

        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
         

        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
         

        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*')),
         

        (os.path.join('share', package_name, 'maps'), 
         glob('maps/*')),


        (os.path.join('share', package_name, 'templates'), 
          glob(os.path.join('templates', '*.html'))),
         

        (os.path.join('share', package_name, 'static', 'css'), 
         glob(package_name + '/static/css/*')),
        (os.path.join('share', package_name, 'static', 'js'), 
         glob(package_name + '/static/js/*')),
        (os.path.join('share', package_name, 'static', 'img'), 
         glob(package_name + '/static/img/*')),
    ],
    install_requires=['setuptools', 'flask', 'Pillow', 'gpiozero', 'pyserial', 'smbus2'],
    zip_safe=True,
    maintainer='hosbot',
    maintainer_email='mariamusama4@gmail.com',
    description='Hospital Delivery Robot',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'motor_control = hospital_robot.motor_node:main',
            'web_server = hospital_robot.app:main',
            'arduino_bridge = hospital_robot.arduino_bridge_node:main',
            'ultrasonic_node = hospital_robot.ultrasonic_node:main',
            

            'lidar_driver = hospital_robot.lidar_driver:main',
            'cliff_detector = hospital_robot.cliff_detector_node:main',
            'imu_node = hospital_robot.imu_node:main',
            
            
            'encoder_driver = hospital_robot.encoder_driver_node:main',
            'wheel_odometry = hospital_robot.wheel_odometry:main',
            

            'explorer = hospital_robot.explorer:main',
        ],
    },
)