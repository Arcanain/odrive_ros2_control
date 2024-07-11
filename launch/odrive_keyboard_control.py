#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_python',
            executable='my_talker',
            name='talker',
            output='screen'
        ),
        Node(
            package='my_python',
            executable='my_listener',
            name='listener',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
