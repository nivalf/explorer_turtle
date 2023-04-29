#! /usr/bin/env python3
# search_client.py

import rospy
import actionlib

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

move_fwd_action_server_name = "/move_fwd_oa_server"


class ExplorerActionClient():
    goal = SearchGoal()

    def feedback_callback(self, feedback_data: SearchFeedback):
        # TODO: get the current distance travelled, from the feedback message
        # and assign this to a class variable...
        self.distance = feedback_data.current_distance_travelled
        print(f"FEEDBACK: Current distance travelled: {self.distance}m. ")

    def __init__(self):
        self.distance = 0.0

        self.action_complete = False
        rospy.init_node("explorer_action_client")
        self.rate = rospy.Rate(1)

        # TODO: setup a "simple action client" with a callback function
        # and wait for the server to be available...
        self.move_fwd_client = actionlib.SimpleActionClient(move_fwd_action_server_name,
                                                   SearchAction)
        self.move_fwd_client.wait_for_server()

        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            # TODO: cancel the goal request, if this node is shutdown before the action has completed...
            self.move_fwd_client.cancel_goa()

            rospy.logwarn("Goal Cancelled...")

        # TODO: Print the result here...
        print(f"RESULT: Action State = {self.move_fwd_client.get_state()}")
        print(f"RESULT: Total Distance Travelled: {self.distance}m")

    def main_loop(self):
        # TODO: assign values to all goal parameters
        # and send the goal to the action server...
        self.goal.fwd_velocity = 0.2
        self.goal.approach_distance = 0.2

        self.move_fwd_client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        while self.move_fwd_client.get_state() < 2:
            # TODO: Construct an if statement and cancel the goal if the
            # distance travelled exceeds 2 meters...
            if self.distance > 2:
                self.move_fwd_client.cancel_goal()
                print("Goal Cancelled since distance travelled exceeds 2 meters...")
                # break out of the while loop to stop the node:
                break

            self.rate.sleep()

        self.action_complete = True


if __name__ == '__main__':
    # TODO: Instantiate the node and call the main_loop() method from it...
    client_instance = ExplorerActionClient()
    client_instance.main_loop()
