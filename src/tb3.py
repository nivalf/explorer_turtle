#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees
import numpy as np

class Tb3Move(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()
        self.publish()

class Tb3Odometry(object):
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)
    
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)
    
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

class Tb3LaserScan(object):
    # Get the min_distance and the direction
    def get_closest_object_details(self, ranges):

        left_arc = ranges[0:41]
        right_arc = ranges[-41:]
        front_arc = np.concatenate((left_arc[::-1], right_arc[::-1]))

        min_distance = front_arc.min()
        arc_angles = np.arange(-41, 41)
        closest_object_position = arc_angles[np.argmin(front_arc)]

        return (min_distance, closest_object_position)
    
    # Get max distance and direction
    def get_farthest_object_details(self, ranges):

        left_arc = ranges[0:90]
        right_arc = ranges[-90:]
        front_arc = np.concatenate((left_arc[::-1], right_arc[::-1]))

        max_distance = front_arc.max()
        arc_angles = np.arange(-90, 90)
        farthest_object_position = arc_angles[np.argmax(front_arc)]

        return (max_distance, farthest_object_position)

    def laserscan_cb(self, scan_data):
        # Clip the scan data to the valid range of lidar [0.12m, 3.5m]
        ranges = np.clip(scan_data.ranges, 0.12, 3.5)

        (self.min_distance, self.closest_object_position) = self.get_closest_object_details(ranges)
        (self.max_distance, self.farthest_object_position) = self.get_farthest_object_details(ranges)

        sector_angle = int(360/self.n_sectors)

        for i in range(self.n_sectors):
            sector_scan_data = ranges[i*sector_angle:(i+1)*sector_angle]
            min_index = np.argmin(sector_scan_data)
            min_distance = sector_scan_data[min_index]

            angle_increment = scan_data.angle_increment
            min_angle = i*sector_angle + min_index*angle_increment
            self.sector[i] = {"id": i, "min_distance": min_distance, "min_angle": min_angle}
    

    def __init__(self):
        self.n_sectors = 8
        self.sector = [{} for _ in range(self.n_sectors)]

        self.min_distance = 0.0
        self.closest_object_position = 0.0 # degrees
        self.max_distance = 3.5
        self.farthest_object_position = 0.0 # degrees
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb) 
