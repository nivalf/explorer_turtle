#! /usr/bin/env python3
# search_action_server.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow


class SearchActionServer():
    feedback = SearchFeedback()
    result = SearchResult()

    def __init__(self):
        # TODO: create a "simple action server" with a callback function, and start it...
        self.actionserver = actionlib.SimpleActionServer(
            "search_action_server", SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        try:
            rate = rospy.Rate(10)

            fwd_velocity = goal.fwd_velocity
            approach_distance = goal.approach_distance


            if not self.is_req_valid(goal):
                # abort the action server if an invalid goal has been requested...
                # DEV: Add a message. Figure out how to do this.
                self.actionserver.set_aborted()
                return

            # TODO: Print a message to indicate that the requested goal was valid
            print(f"\n#####\n"
                f"The 'search_action_server' has been called.\n"
                f"Goal: Move forward with {fwd_velocity:.2f} m/s velocity and stop at {approach_distance:.2f} distance from the object ...\n\n"
                f"Commencing the action...\n"
                f"#####\n")
            
            # Flag for status of the action
            success = True

            # Get the robot's current odometry from the Tb3Odometry() class:
            self.posx0 = self.tb3_odom.posx
            self.posy0 = self.tb3_odom.posy
            # Get information about objects up ahead from the Tb3LaserScan() class:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            # TODO: set the robot's forward velocity (as specified in the "goal")...
            self.vel_controller.set_move_cmd(fwd_velocity, 0.2)

            # TODO: establish a conditional statement so that the
            # while loop continues as long as the distance to the closest object
            # ahead of the robot is always greater than the "approach distance"
            # (as specified in the "goal")...
            while self.tb3_lidar.min_distance > approach_distance:
                # update LaserScan data:
                self.closest_object = self.tb3_lidar.min_distance
                self.closest_object_location = self.tb3_lidar.closest_object_position

                # publish a velocity command to make the robot start moving
                self.vel_controller.publish()

                # check if there has been a request to cancel the action mid-way through:
                if self.actionserver.is_preempt_requested():
                    # TODO: take appropriate action if the action is cancelled (peempted)...
                    rospy.loginfo("Cancelling the search.")
                
                    # DEV: Add a message. Figure out how to do this.
                    self.actionserver.set_preempted()
                    # stop the robot:
                    self.vel_controller.stop()
                    success = False
                    # exit the loop:
                    break

                # determine how far the robot has travelled so far:
                self.distance = sqrt(
                    pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

                # TODO: update all feedback message values and publish a feedback message:
                self.feedback.current_distance_travelled = self.distance
                self.actionserver.publish_feedback(self.feedback)

                # TODO: update all result parameters:
                self.result.total_distance_travelled = self.distance
                self.result.closest_object_distance = self.closest_object
                self.result.closest_object_angle = self.closest_object_location

                rate.sleep()

            if success:
                rospy.loginfo("approach completed successfully.")
                # TODO: Set the action server to "succeeded" and stop the robot...
                self.actionserver.set_succeeded(self.result)
                self.vel_controller.stop()
        except rospy.ROSInterruptException:
            pass

    # Handle invalid requests
    def is_req_valid(self, goal: SearchGoal):
        fwd_velocity = goal.fwd_velocity
        approach_distance = goal.approach_distance
        
        # Implement some checks on the "goal" input parameter(s)
        is_valid = True

        if approach_distance < 0:
            print(f'Invalid approach distance {approach_distance:.2f}m. Please choose a distance between 0.12m and 3.5m.')
            is_valid = False
        elif approach_distance < 0.12:
            print(f'Invalid approach distance {approach_distance:.2f}m. Please choose a distance between 0.12m and 3.5m.')
            is_valid = False
        elif approach_distance >= 3.5:
            print(f'Invalid approach distance {approach_distance:.2f}m. Please choose a distance between 0.12m and 3.5m.')
            is_valid = False
        
        if fwd_velocity < 0:
            print(f'Invalid approach velocity {fwd_velocity:.2f}m/s.')
            is_valid = False
        elif fwd_velocity > 0.26:
            print(f'Invalid approach velocity {fwd_velocity:.2f}m/s. Please choose an approach velocity between 0m/s and 0.26m/s.')
            is_valid = False

        return is_valid


if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
