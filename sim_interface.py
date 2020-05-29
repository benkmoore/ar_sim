#!/usr/bin/env python

import rospy
import numpy as np

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray, Float64
from tf.transformations import euler_from_quaternion

# take model states msg and publish a curr pose in Pose2D format

class SimInterface():
	def __init__(self):
		# gazebo model states to Pose2D
		rospy.init_node('robot_pose')
		self.pub_pose = rospy.Publisher('pose', Pose2D, queue_size=10)

		rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_statesCallback)

		self.pose = Pose2D()
		self.pos = None
		self.theta = None

		# GNC commands to sim joint inputs
		rospy.Subscriber('/controller_cmds', Float64MultiArray, self.control_cmdsCallback)

		self.control_cmds = None
		self.V_cmd = None
		self.phi_cmd = None

		self.cmdP1_pub = rospy.Publisher("/robot_0/joint1_position_controller/command",Float64, queue_size=10)
		self.cmdP2_pub = rospy.Publisher("/robot_0/joint2_position_controller/command",Float64, queue_size=10)
		self.cmdP3_pub = rospy.Publisher("/robot_0/joint3_position_controller/command",Float64, queue_size=10)
		self.cmdP4_pub = rospy.Publisher("/robot_0/joint4_position_controller/command",Float64, queue_size=10)
		self.cmdV5_pub = rospy.Publisher("/robot_0/joint5_velocity_controller/command",Float64, queue_size=10)
		self.cmdV6_pub = rospy.Publisher("/robot_0/joint6_velocity_controller/command",Float64, queue_size=10)
		self.cmdV7_pub = rospy.Publisher("/robot_0/joint7_velocity_controller/command",Float64, queue_size=10)
		self.cmdV8_pub = rospy.Publisher("/robot_0/joint8_velocity_controller/command",Float64, queue_size=10)


	def control_cmds2JointInputs(self):
		self.V_cmd = self.control_cmds[0:4]
		self.phi_cmd = self.control_cmds[4] # phi1 = self.gnc_cmds[4], phi2 = self.gnc_cmds[5]

	def control_cmdsCallback(self, msg):
		self.control_cmds = msg.data
		self.control_cmds2JointInputs()

	def modelStates2Pose2D(self):
		self.pose.x = self.pos.x
		self.pose.y = self.pos.y
		self.pose.theta = self.theta

	def model_statesCallback(self, msg):
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
			if self.phi_cmd != None and self.V_cmd != None:
				self.cmdP1_pub.publish(self.phi_cmd) 	# yaw_joint_l1
				self.cmdP2_pub.publish(self.phi_cmd) 	# yaw_joint_l2
				self.cmdP3_pub.publish(self.phi_cmd) 	# yaw_joint_r1
				self.cmdP4_pub.publish(self.phi_cmd) 	# yaw_joint_r2
				self.cmdV5_pub.publish(self.V_cmd[0]) 	# drive_joint_l1
				self.cmdV6_pub.publish(self.V_cmd[1]) 	# drive_joint_l2
				self.cmdV7_pub.publish(self.V_cmd[2]) 	# drive_joint_r1
				self.cmdV8_pub.publish(self.V_cmd[3]) 	# drive_joint_r2

			rate.sleep()


if __name__ == '__main__':
	interface = SimInterface()
	interface.run()