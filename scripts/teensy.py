#!/usr/bin/env python

import rospy

RATE = 10

class SimTeensy:
    def __init__(self):
        # gazebo model states to Pose2D
        rospy.init_node(rospy.get_param("ros_serial_node"))

    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    teensy = SimTeensy()
    teensy.run()