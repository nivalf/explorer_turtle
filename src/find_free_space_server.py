#! /usr/bin/env python3
# find_free_space_action_server.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from turtlebot_explorer.msg import FindFreeSpaceAction, FindFreeSpaceFeedback, FindFreeSpaceResult, FindFreeSpaceGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow


class FindFreeSpaceActionServer():
    feedback = FindFreeSpaceFeedback()
    result = FindFreeSpaceResult()

    def __init__(self):
        # create a "simple action server" with a callback function, and start it...
        self.actionserver = actionlib.SimpleActionServer(
            "find_free_space_action_server", FindFreeSpaceAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Find Free Space Action Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: FindFreeSpaceGoal):
        try:
            rate = rospy.Rate(10)

            ang_velocity_magnitude = goal.ang_velocity_magnitude
            min_clear_distance = goal.min_clear_distance


            if not self.is_req_valid(goal):
                # abort the action server if an invalid goal has been requested...
                # DEV: Add a message. Figure out how to do this.
                self.actionserver.set_aborted()
                return

            # TODO: Print a message to indicate that the requested goal was valid
            print(f"\n#####\n"
                f"The 'find_free_space_action_server' has been called.\n"
                f"Goal: Find free space with mininum free space of {min_clear_distance:.2f}m while turning with an angular velocity of {ang_velocity_magnitude:.2f} rad/s...\n\n"
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

            # set the robot's angular velocity (as specified in the "goal")...
            self.vel_controller.set_move_cmd(0, ang_velocity_magnitude)

            # establish a conditional statement so that the
            # while loop continues as long as the distance to the closest object
            # ahead of the robot is always less than the "minimum clear distance"
            # (as specified in the "goal")...
            while self.tb3_lidar.min_distance < min_clear_distance:
                # update LaserScan data:
                self.closest_object = self.tb3_lidar.min_distance
                self.closest_object_location = self.tb3_lidar.closest_object_position

                # publish a velocity command to make the robot start turning
                self.vel_controller.publish()

                # check if there has been a request to cancel the action mid-way through:
                if self.actionserver.is_preempt_requested():
                    # take appropriate action if the action is cancelled (peempted)...
                    rospy.loginfo("Cancelling the Search for free space.")
                
                    # DEV: Add a message. Figure out how to do this.
                    self.actionserver.set_preempted()
                    # stop the robot:
                    self.vel_controller.stop()
                    success = False
                    # exit the loop:
                    break

                # determine how far the robot has travelled so far:
                # DEV: change to find the angle turned
                self.distance = sqrt(
                    pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

                # DEV TODO: update all feedback message values and publish a feedback message:
                self.feedback.current_angle_turned = 0
                self.actionserver.publish_feedback(self.feedback)


                rate.sleep()

            if success:
                rospy.loginfo("Found Free Space")

                # DEV TODO: update all result parameters:
                self.result.success = success
                self.result.turned_angle = 0

                # TODO: Set the action server to "succeeded" and stop the robot...
                self.actionserver.set_succeeded(self.result)
                self.vel_controller.stop()
                
        except rospy.ROSInterruptException:
            pass

    # Handle invalid requests
    def is_req_valid(self, goal: FindFreeSpaceGoal):
        ang_velocity_magnitude = goal.ang_velocity_magnitude
        min_clear_distance = goal.min_clear_distance
        
        # Implement some checks on the "goal" input parameter(s)
        is_valid = True

        if min_clear_distance < 0:
            print(f'Invalid minimum clear distance {min_clear_distance:.2f}m. Please choose a distance between 0.12m and 3.5m.')
            is_valid = False
        elif min_clear_distance < 0.12:
            print(f'Invalid minimum clear distance {min_clear_distance:.2f}m. Please choose a distance between 0.12m and 3.5m.')
            is_valid = False
        elif min_clear_distance >= 3.5:
            print(f'Invalid minimum clear distance {min_clear_distance:.2f}m. Please choose a distance between 0.12m and 3.5m.')
            is_valid = False
        
        if not abs(ang_velocity_magnitude) < 1.82:
            print(f'Invalid angular velocity {ang_velocity_magnitude:.2f}m/s. Please choose an angular velocity between -1.82 rad/s and 1.82 rad/s.')
            is_valid = False

        return is_valid


if __name__ == '__main__':
    rospy.init_node("find_free_space_action_server")
    FindFreeSpaceActionServer()
    rospy.spin()

