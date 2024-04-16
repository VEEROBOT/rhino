#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import time
from math import pi

class DifferentialDriver:
    def __init__(self):
        self.wheel_diameter = 0.10
        self.motor_rpm = 200
        self.max_pwm_val = 170
        self.min_pwm_val = -170
        self.wheel_separation = 0.30
        self.ka= 10.0
        self.kl= 1.2

        rclpy.init()

        self.node = rclpy.create_node('cmdvel_listener')
 
        self.subscription = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

        self.timer = self.node.create_timer(0.01, self.publish_callback)

        self.left_pwm_pub = self.node.create_publisher(Int16, 'pwml', 10)
        self.right_pwm_pub = self.node.create_publisher(Int16, 'pwmr', 10)

        self.left_pwm = Int16()
        self.right_pwm = Int16()

        self.wheel_radius = self.wheel_diameter / 2
        self.circumference_of_wheel = 2 * pi * self.wheel_radius
        self.max_speed = (self.circumference_of_wheel * self.motor_rpm) / 60  # m/sec
        self.right_vel_actual = 0
        self.left_vel_actual = 0
        self.kp = 0.5


    def get_pwm(self, left_speed, right_speed):

        if left_speed and right_speed != 0:

            self.lspeedPWM = max(
                min((left_speed / self.max_speed) * self.max_pwm_val, self.max_pwm_val),
                self.min_pwm_val
            )
            self.rspeedPWM = max(
                min((right_speed / self.max_speed) * self.max_pwm_val, self.max_pwm_val),
                self.min_pwm_val
            )
        elif left_speed <0.0 and right_speed <0.0 and left_speed !=right_speed:


            self.lspeedPWM = -max(
                min((abs(left_speed) / self.max_speed) * self.max_pwm_val, self.max_pwm_val),
                self.min_pwm_val
            )
            self.rspeedPWM = -max(
                min((abs(right_speed) / self.max_speed) * self.max_pwm_val, self.max_pwm_val),
                self.min_pwm_val
            )

        else:
            self.lspeedPWM=0.0
            self.rspeedPWM=0.0

        return self.lspeedPWM, self.rspeedPWM

    def callback(self, data):
        linear_vel = data.linear.x  # Linear Velocity of Robot
        angular_vel = data.angular.z  # Angular Velocity of Robot

        right_vel = linear_vel *self.kl + ((angular_vel * self.wheel_separation) / 2) *self.ka # right wheel velocity
        left_vel = linear_vel *self.kl- ((angular_vel * self.wheel_separation) / 2)* self.ka  # left wheel velocity


        left_pwm_data, right_pwm_data = self.get_pwm(left_vel, right_vel)

        left_rpm = (left_vel / (2 * pi * self.wheel_radius)) * 60
        right_rpm = (right_vel / (2 * pi * self.wheel_radius)) * 60

        # print(" Left Velocity = {} m/s | Left RPM = {} RPM".format(left_vel, left_rpm))
        # print(" Right Velocity = {} m/s | Right RPM = {} RPM".format(right_vel, right_rpm))
        
        self.left_pwm.data = int(left_pwm_data)
        self.right_pwm.data = int(right_pwm_data)


    def publish_callback(self):

        self.left_pwm_pub.publish(self.left_pwm)
        self.right_pwm_pub.publish(self.right_pwm)


if __name__ == '__main__':
    dd = DifferentialDriver()

    dd.node.get_logger().info('Differential Drive Initialized with following Params-')
    dd.node.get_logger().info('Motor Max RPM:\t' + str(dd.motor_rpm) + ' RPM')
    dd.node.get_logger().info('Wheel Diameter:\t' + str(dd.wheel_diameter) + ' m')
    dd.node.get_logger().info('Wheel Separation:\t' + str(dd.wheel_separation) + ' m')
    dd.node.get_logger().info('Robot Max Speed:\t' + str(dd.max_speed) + ' m/sec')
    dd.node.get_logger().info('Robot Max PWM:\t' + str(dd.max_pwm_val))

    rclpy.spin(dd.node)
    rclpy.shutdown()

