#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
import tf_transformations
import tf2_ros

import sys
import time
import odrive
from odrive.enums import *
import fibre.libfibre
import math

class OdriveMotorControl(Node):
    def __init__(self):
        super().__init__('odrive_twist_driver')
        self.create_subscription(Twist, 'cmd_vel', self.callback_vel, 10)
        # Connect to Odrive
        self.find_odrive()

        self.tire_tread         = 0.425 #[m]
        self.target_linear_vel  = 0.0   #[m/s]
        self.target_angular_vel = 0.0   #[rad/s]

        self.tire_diameter      = 0.2   #[m]  
        self.tire_circumference = math.pi * self.tire_diameter  

        self.timer = self.create_timer(0.1, self.update) 

    def callback_vel(self, msg):
        self.get_logger().info('Callback received a velocity message.')
        self.get_logger().info('I heard: "%s"' % msg.linear.x)
        self.target_linear_vel = msg.linear.x
        self.target_angular_vel = msg.angular.z

    def find_odrive(self):
        while True:
            self.get_logger().info("Connect to Odrive...")
            self.odrv0 = odrive.find_any()
            if self.odrv0 is not None:
                self.get_logger().info("Connect to Odrive Success!!!")
                break
            else:
                self.get_logger().info("Disconnect to Odrive...")

    def odrive_setup(self):
        self.get_logger().info("start setup...")
        self.get_logger().info("%s" % self.odrv0.vbus_voltage)
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.controller.input_vel = 0
    
    def update(self):
        right_vel, left_vel = self.calc_relative_vel(self.target_linear_vel, self.target_angular_vel)
        self.odrv0.axis0.controller.input_vel = right_vel
        self.odrv0.axis1.controller.input_vel = -left_vel
        #トルクを計算
        # 各軸のモーター電流を取得
        motor_current_0 = self.odrv0.axis0.motor.current_control.Iq_measured
        motor_current_1 = self.odrv0.axis1.motor.current_control.Iq_measured

        # トルク定数を取得
        torque_constant_0 = self.odrv0.axis0.motor.config.torque_constant
        torque_constant_1 = self.odrv0.axis1.motor.config.torque_constant

        # トルクを計算
        torque_0 = motor_current_0 * torque_constant_0
        torque_1 = motor_current_1 * torque_constant_1

        print("軸0のトルク: {} Nm".format(torque_0))
        print("軸1のトルク: {} Nm".format(torque_1))
        
    def calc_relative_vel(self, target_linear_vel, target_angular_vel):
        # Convert to each circumferential velocity
        circumferential_right_vel = target_linear_vel + (self.tire_tread / 2.0) * target_angular_vel #[m/s]
        circumferential_left_vel  = target_linear_vel - (self.tire_tread / 2.0) * target_angular_vel #[m/s]

        # Convert to each rotational velocity
        right_vel = circumferential_right_vel / self.tire_circumference #[turn/s]
        left_vel  = circumferential_left_vel / self.tire_circumference  #[turn/s]

        return right_vel, left_vel
    
    def fini(self):
        self.get_logger().info("shutdown...")
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.controller.input_vel = 0

def main(args=None):
    rclpy.init(args=args)
    Odrive_motor_control = OdriveMotorControl()
    Odrive_motor_control.odrive_setup()
    #Odrive_motor_control.odrive_control()
    try:
        rclpy.spin(Odrive_motor_control)
    finally:
        Odrive_motor_control.fini()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
