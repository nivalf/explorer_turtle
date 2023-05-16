#!/usr/bin/env python3

import roslaunch
import rospy

class MapSaver():
    def __init__(self):
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        self.rate = rospy.Rate(0.2) # T=5s
        self.map_path = "$(find obstacle_avoider)/maps/explore_map"
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        rospy.loginfo("Shutting down 'map_saver' node")

    def save_map(self):
        while not rospy.is_shutdown():
            print(f"Saving map at time: {rospy.get_time()}...")
            map_node = roslaunch.core.Node(package="map_server",
                                    node_type="map_saver",
                                    args=f"-f {self.map_path}")
            process = self.launch.launch(map_node)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        # Instantiate the node and call the main_loop() method from it...
        rospy.init_node("map_saver", anonymous=True)
        MapSaver().save_map()
    except rospy.ROSInterruptException:
        pass
