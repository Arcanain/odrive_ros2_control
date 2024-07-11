from setuptools import find_packages, setup
import os

package_name = 'odrive_ros2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'python_programs.talker',
        'python_programs.listener',
        'python_programs.keyboard_teleop',
        'python_programs.odrive_twist_driver',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/odrive_keyboard_control.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='mtom0809tom0809@gmial.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_talker= python_programs.talker:main',
            'my_listener= python_programs.listener:main',
            'keyboard_teleop= python_programs.keyboard_teleop:main',
            'control_odrive= python_programs.odrive_twist_driver:main',
        ],
    },
)
