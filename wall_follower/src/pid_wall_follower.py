#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from dynamic_reconfigure.server import Server
from wall_follower.cfg import WallFollowerConfig

class PIDWallFollower:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # PID Coefficients
        self.Kp = 1.0  # Proportional gain
        self.Ki = 0.5  # Integral gain
        self.Kd = 0.8  # Derivative gain

        # Error Values
        self.error = 0.0
        self.error_sum = 0.0
        self.prev_error = 0.0

        # Max Limits
        self.max_integral = 10.0  # Maximum integral term
        self.max_linear_velocity = 0.2  # Linear velocity limit (m/s)
        self.max_angular_velocity = 1.5  # Angular velocity limit (rad/s)

        self.cmd_msg = Twist()

        # setup dynamic reconfigure server
        self.srv = Server(WallFollowerConfig, self.reconfigure_callback)

    def reconfigure_callback(self, config, level):
        # update PID parameters when they're changed
        self.kp = config.Kp
        self.ki = config.Ki
        self.kd = config.Kd
        return config

    def scan_callback(self, msg):
        # Considering only a small range in front

        left_arc = msg.ranges[0:10]
        right_arc = msg.ranges[-10:]
        front_ranges = np.concatenate((left_arc[::-1], right_arc[::-1]))
        front_distance = min(front_ranges)

        # Considering a small range on the right
        right_ranges = msg.ranges[260:280]
        right_distance = min(right_ranges)

        # Angle error calculation
        desired_angle = np.arctan2(0.5, right_distance)
        current_angle = np.arctan2(0.5, front_distance)

        self.error = desired_angle - current_angle

        # PID controller
        proportional = self.Kp * self.error
        self.error_sum += self.error
        integral = self.Ki * self.error_sum
        derivative = self.Kd * (self.error - self.prev_error)

        # Saturation and windup handling
        if integral > self.max_integral:
            integral = self.max_integral
            self.error_sum = self.max_integral / self.Ki
        elif integral < -self.max_integral:
            integral = -self.max_integral
            self.error_sum = -self.max_integral / self.Ki

        control = proportional + integral + derivative

        # Velocity saturation
        linear_velocity = 0.2
        angular_velocity = control

        if angular_velocity > self.max_angular_velocity:
            angular_velocity = self.max_angular_velocity
        elif angular_velocity < -self.max_angular_velocity:
            angular_velocity = -self.max_angular_velocity

        # Set the control command
        self.cmd_msg.linear.x = linear_velocity
        self.cmd_msg.angular.z = angular_velocity

        self.prev_error = self.error

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.cmd_msg)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('pid_wall_follower')
    
    # create an instance of PIDWallFollower and run it
    wall_follower = PIDWallFollower()
    wall_follower.run()
