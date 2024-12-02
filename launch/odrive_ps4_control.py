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
    imu_package = 'adi_imu_tr_driver_ros2'
    with open(file_path, 'r') as file:
        robot_description = file.read()
    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen'
        ),
        Node(
            package=imu_package,
            executable='adis_rcv_csv_node',
            output="screen",
            parameters=[
                {"mode": "Attitude"},
                {"device": "/dev/ttyACM_IMU"},
            ],
        ),
        Node(
                package='ros2_joy_to_twist',
                executable='joy_to_twist',
                name='joy_to_twist',
                output='screen'
        ),
        Node(
            package='odrive_ros2_control',
            executable='control_odrive_and_odom_pub',
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