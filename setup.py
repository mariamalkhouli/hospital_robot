from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hospital_robot'

setup(
    name=package_name,
    version='0.0.0',
    # Find packages in the root of the package (where app.py now is)
    packages=find_packages(exclude=['test']), 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copy the templates and static folders directly into the share directory
        (os.path.join('share', package_name, 'templates'), 
         glob('templates/*')),
        (os.path.join('share', package_name, 'static'), 
         glob('static/**/*', recursive=True)),
        # Copy app.py (if it were outside the main package, but it's not needed here as it's installed via 'packages')
    ],
    # Add Flask and Pillow to the required dependencies for deployment
    install_requires=['setuptools', 'flask', 'Pillow'], 
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
            # Update entry point to point directly to the app module
            'web_server = hospital_robot.app:main' 
        ],
    },
)