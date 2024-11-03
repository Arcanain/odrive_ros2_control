#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')
    package_dir = get_package_share_directory('odrive_ros2_control')
    rviz = os.path.join(package_dir, 'rviz', 'odom_publish.rviz')
    with open(file_path, 'r') as file:
        robot_description = file.read()
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='both',
            parameters=[{'joint_state_publisher': robot_description}]
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
            package='odrive_ros2_control',
            executable='control_odrive',
            name='odrive_twist_driver',
            output='screen'
        ),
        Node(
            package='arcanain_simulator',
            executable='odrive_odometry_pub',
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
