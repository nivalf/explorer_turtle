#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from wall_follower.cfg import WallFollowerConfig

class PIDWallFollower:
    def __init__(self):
        # initialize the node
        rospy.init_node('pid_wall_follower')

        # setup publisher for movement commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # setup subscriber for laser scan messages
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # setup a default movement command
        self.cmd = Twist()

        # PID coefficients
        self.kp = 1
        self.ki = 0.5
        self.kd = 0.8

        # error values
        self.prev_error = 0.0
        self.cum_error = 0.0

        # desired distance from the wall
        self.desired_distance = 1.5

        # setup dynamic reconfigure server
        self.srv = Server(WallFollowerConfig, self.reconfigure_callback)

    def reconfigure_callback(self, config, level):
        # update PID parameters when they're changed
        self.kp = config.Kp
        self.ki = config.Ki
        self.kd = config.Kd
        return config

    def scan_callback(self, msg):
        # this method gets called every time a LaserScan message is received

        # find the minimum range (closest object)
        min_range = min(msg.ranges)

        # compute the error
        error = self.desired_distance - min_range

        # compute the control output (correction needed)
        correction = self.kp*error + self.ki*self.cum_error + self.kd*(error - self.prev_error)

        # apply the correction to the robot's motion command
        self.cmd.linear.x = 0.1  # constant forward speed
        self.cmd.angular.z = correction

        # update error values
        self.prev_error = error
        self.cum_error += error

        # publish the movement command
        self.cmd_pub.publish(self.cmd)

    def run(self):
        # keep the node running until it's shut down
        rospy.spin()

if __name__ == '__main__':
    # create an instance of PIDWallFollower and run it
    PIDWallFollower().run()
