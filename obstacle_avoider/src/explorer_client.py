#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback
from obstacle_avoider.msg import FindFreeSpaceAction, FindFreeSpaceGoal, FindFreeSpaceFeedback

search_action_server_name = "/search_action_server"
reverse_search_action_server_name = "/reverse_search_action_server"
find_free_space_action_server_name = "/find_free_space_action_server"


class ExplorerActionClient():
    search_goal = SearchGoal()
    reverse_search_goal = SearchGoal()
    find_free_space_goal = FindFreeSpaceGoal()

    def search_feedback_callback(self, feedback_data: SearchFeedback):
        # get the current distance travelled, from the feedback message
        # and assign this to a class variable...
        self.distance = feedback_data.current_distance_travelled
        print(f"FEEDBACK: Current distance travelled: {self.distance}m. ")


    def find_free_space_feedback_cb(self, feedback_data: FindFreeSpaceFeedback):
        # get the current angle turned, from the feedback message
        # and assign this to a class variable...
        self.angle = feedback_data.current_angle_turned
        print(f"FEEDBACK: Current angle turned: {self.angle} degrees. ")



    def __init__(self):
        self.distance = 0.0
        self.angle = 0.0

        self.action_complete = False
        rospy.init_node("explorer_client")
        self.rate = rospy.Rate(1)

        # setup a "simple action client" with a callback function
        # and wait for the server to be available...
        self.search_client = actionlib.SimpleActionClient(search_action_server_name,
                                                   SearchAction)
        self.search_client.wait_for_server()

        # setup a "simple action client" with a callback function
        # and wait for the server to be available...
        self.reverse_search_client = actionlib.SimpleActionClient(reverse_search_action_server_name,
                                                   SearchAction)
        self.reverse_search_client.wait_for_server()

        # Setup client for finding free space
        self.find_free_space_client = actionlib.SimpleActionClient(find_free_space_action_server_name, FindFreeSpaceAction)
        self.find_free_space_client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            # cancel the goal request, if this node is shutdown before the action has completed...
            self.search_client.cancel_goal()
            self.reverse_search_client.cancel_goal()
            self.find_free_space_client.cancel_goal()

            rospy.logwarn("Goal Cancelled...")


        # TODO: Print the result here...
        print(f"RESULT: Search Action State = {self.search_client.get_state()}")
        print(f"RESULT: Total Distance Travelled: {self.distance}m")
        print(f"RESULT: Reverse Search Action State = {self.reverse_search_client.get_state()}")
        print(f"RESULT: Total Distance Travelled: {self.distance}m")
        print(f"RESULT: Find Free Space Action State = {self.find_free_space_client.get_state()}")
        print(f"RESULT: Total Angle Turned: {self.angle} degrees")

        
        self.action_complete = True

    def main_loop(self):
        # assign values to all goal parameters
        # and send the goal to the action server...
        self.search_goal.fwd_velocity = rospy.get_param("~linear_velocity")
        self.search_goal.approach_distance = rospy.get_param("~approach_distance")

        self.reverse_search_goal.fwd_velocity = -rospy.get_param("~linear_velocity")
        self.reverse_search_goal.approach_distance = rospy.get_param("~approach_distance")

        self.find_free_space_goal.ang_velocity_magnitude = rospy.get_param("~ang_velocity_magnitude")
        self.find_free_space_goal.min_clear_distance = rospy.get_param("~approach_distance")

        
        while not rospy.is_shutdown():
            # self.reverse_search_client.send_goal_and_wait(self.reverse_search_goal)
            self.search_client.send_goal_and_wait(self.search_goal)
            self.find_free_space_client.send_goal_and_wait(self.find_free_space_goal)
                


if __name__ == '__main__':
    try:
        # Instantiate the node and call the main_loop() method from it...
        client_instance = ExplorerActionClient()
        client_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
