#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import sin, cos, pi

# Import the tb3 modules from tb3.py
from tb3 import Tb3LaserScan

SECTORS = 2  # left and right sector


class VisualiseMarkers:
    def __init__(self):
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.approach_distance = rospy.get_param("~approach_distance")

        # Initialize LED markers for each sector
        self.led_markers = []
        for i in range(SECTORS):
            led_marker = Marker()
            led_marker.header.frame_id = "base_scan"
            led_marker.id = i
            led_marker.type = led_marker.SPHERE
            led_marker.action = led_marker.ADD
            led_marker.scale.x = 0.05
            led_marker.scale.y = 0.05
            led_marker.scale.z = 0.05
            led_marker.color.a = 0.5
            led_marker.pose.orientation.w = 1.0
            led_marker.pose.position.x = self.approach_distance * 0.75 * cos((i - 0.5) * pi / 6)  # adjust for left and right sector
            led_marker.pose.position.y = self.approach_distance * 0.75 * sin((i - 0.5) * pi / 6)
            led_marker.pose.position.z = 0
            self.led_markers.append(led_marker)

        # Initialize markers for the sector lines
        self.line_markers = []
        for i in range(SECTORS + 1):  # one extra for the front line
            line_marker = Marker()
            line_marker.header.frame_id = "base_scan"
            line_marker.id = len(self.led_markers) + i
            line_marker.type = line_marker.ARROW
            line_marker.action = line_marker.ADD
            line_marker.scale.x = 0.01  # shaft diameter
            line_marker.scale.y = 0.01  # head diameter
            line_marker.scale.z = 0.05  # head length
            line_marker.color.a = 0.5
            line_marker.color.r = 1.0   # White color
            line_marker.color.g = 1.0
            line_marker.color.b = 1.0  
            line_marker.pose.orientation.w = 1.0
            line_marker.points.append(Point())  # start at robot position
            end_point = Point()
            end_point.x = self.approach_distance * cos((i - 1) * pi / 6)  # adjust for front, left, and right lines
            end_point.y = self.approach_distance * sin((i - 1) * pi / 6)
            line_marker.points.append(end_point)
            self.line_markers.append(line_marker)
        
        # Initialize marker for the obstacle detection threshold circle
        circle_marker = Marker()
        circle_marker.header.frame_id = "base_scan"
        circle_marker.id = self.line_markers[-1].id + 1
        circle_marker.type = circle_marker.CYLINDER
        circle_marker.action = circle_marker.ADD
        circle_marker.scale.x = 2 * self.approach_distance
        circle_marker.scale.y = 2 * self.approach_distance
        circle_marker.scale.z = 0.01  # thin cylinder
        circle_marker.color.a = 0.3  # semi-transparent
        circle_marker.color.b = 1.0  # blue color
        circle_marker.pose.orientation.w = 1.0
        circle_marker.pose.position.z = -0.1  # slightly below the robot to not interfere with other markers
        self.circle_marker = circle_marker

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.tb3_lidar = Tb3LaserScan()

        # Handle shut down
        rospy.on_shutdown(self.shutdown_ops)

        rospy.loginfo("The 'visualise_markers' node is active")

    
    def shutdown_ops(self):
        # Add any shutdown actions as necessary
        rospy.logwarn("Received a shutdown request. Shutting down 'visualise_markers' node...")

    def set_led_markers(self):
        obstacle_detected = self.tb3_lidar.min_distance < self.approach_distance

        if(obstacle_detected):
            for i in range(SECTORS):
                if self.tb3_lidar.sector[(7+i)%8]["min_distance"] < self.approach_distance:
                    self.turn_on_led_marker(i)
                else:
                    self.turn_off_led_marker(i)
        else:
            self.turn_off_all_led_markers()

    def publish_marker_data(self):
        # Publish Proximity LED indicator markers
        for led_marker in self.led_markers:
            self.marker_publisher.publish(led_marker)

        # Publish the sector lines
        for line_marker in self.line_markers:
            self.marker_publisher.publish(line_marker)

        # Publish obstacle range circle
        self.marker_publisher.publish(self.circle_marker)

    def turn_off_all_led_markers(self):
        for i in range(SECTORS):
            self.turn_off_led_marker(i)

    # Turn ON the LED : RED color
    def turn_on_led_marker(self, i):
        self.led_markers[i].color.r = 1.0
        self.led_markers[i].color.g = 0.0
        self.led_markers[i].color.b = 0.0
        self.led_markers[i].color.a = 1.0

    # Turn OFF the LED: White
    def turn_off_led_marker(self, i):
        self.led_markers[i].color.r = 1.0
        self.led_markers[i].color.g = 1.0
        self.led_markers[i].color.b = 1.0
        self.led_markers[i].color.a = 0.3


    def run(self):
        try:
            while not rospy.is_shutdown():
                self.set_led_markers()
                self.publish_marker_data()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    rospy.init_node("visualise_markers")
    visualiser = VisualiseMarkers()
    visualiser.run()
