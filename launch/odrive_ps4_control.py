#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('odrive_ros2_control')
    rviz = os.path.join(package_dir, 'rviz', 'odom_publish.rviz')

    return LaunchDescription([
        Node(
            package='odrive_ros2_control',
            executable='control_odrive',
            name='odrive_twist_driver',
            output='screen'
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen'
        ),
        Node(
                package='ros2_joy_to_twist',
                executable='joy_to_twist',
                name='joy_to_twist',
                output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=["-d", rviz]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
