#!/usr/bin/env python

import rospy
import numpy as np

from ar_commander.msg import ControllerCmd
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray, Float64
from tf.transformations import euler_from_quaternion

# take model states msg and publish a curr pose in Pose2D format

class SimInterface():
    def __init__(self):
        # gazebo model states to Pose2D
        rospy.init_node('sim_interface')
        self.pub_pose = rospy.Publisher('pose', Pose2D, queue_size=10)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_statesCallback)

        self.pose = Pose2D()
        self.pos = None
        self.theta = None

        # GNC commands to sim joint inputs
        rospy.Subscriber('/controller_cmds', ControllerCmd, self.control_cmdsCallback)

        self.control_cmds = None
        self.W_cmd = None
        self.phi_cmd = None

        self.cmdP1_pub = rospy.Publisher("/robot_0/joint1_position_controller/command",Float64, queue_size=10)
        self.cmdP2_pub = rospy.Publisher("/robot_0/joint2_position_controller/command",Float64, queue_size=10)
        self.cmdP3_pub = rospy.Publisher("/robot_0/joint3_position_controller/command",Float64, queue_size=10)
        self.cmdP4_pub = rospy.Publisher("/robot_0/joint4_position_controller/command",Float64, queue_size=10)
        self.cmdV5_pub = rospy.Publisher("/robot_0/joint5_velocity_controller/command",Float64, queue_size=10)
        self.cmdV6_pub = rospy.Publisher("/robot_0/joint6_velocity_controller/command",Float64, queue_size=10)
        self.cmdV7_pub = rospy.Publisher("/robot_0/joint7_velocity_controller/command",Float64, queue_size=10)
        self.cmdV8_pub = rospy.Publisher("/robot_0/joint8_velocity_controller/command",Float64, queue_size=10)


    def control_cmdsCallback(self, msg):
        self.W_cmd = msg.omega_arr.data
        self.phi_cmd = msg.phi_arr.data

    def modelStates2Pose2D(self):
        self.pose.x = self.pos.x
        self.pose.y = self.pos.y
        self.pose.theta = self.theta

    def model_statesCallback(self, msg):
        if len(msg.pose) >= 2:
            self.pos = msg.pose[1].position
            self.q = msg.pose[1].orientation
            _, _, self.theta = euler_from_quaternion([self.q.x, self.q.y, self.q.z, self.q.w])
            self.modelStates2Pose2D()

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            # Publish pose to GNC
            self.pub_pose.publish(self.pose)

            # Publish GNC cmds to sim joints
            if self.phi_cmd != None and self.W_cmd != None:
                self.cmdP1_pub.publish(self.phi_cmd[2])     # yaw_joint_l1 -> inner on r2
                self.cmdP2_pub.publish(self.phi_cmd[3])     # yaw_joint_l2 -> outer on r2
                self.cmdP3_pub.publish(self.phi_cmd[0])     # yaw_joint_r1 -> inner on r1
                self.cmdP4_pub.publish(self.phi_cmd[1])     # yaw_joint_r2 -> outer on r1
                self.cmdV5_pub.publish(self.W_cmd[0])   # drive_joint_l1
                self.cmdV6_pub.publish(self.W_cmd[1])   # drive_joint_l2
                self.cmdV7_pub.publish(self.W_cmd[2])   # drive_joint_r1
                self.cmdV8_pub.publish(self.W_cmd[3])   # drive_joint_r2

            rate.sleep()


if __name__ == '__main__':
    interface = SimInterface()
    interface.run()
