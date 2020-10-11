#!/usr/bin/env python

import rospy
import rospkg
import sys
import numpy as np
import numpy.random as npr
import numpy.linalg as npl
import math

sys.path.append(rospkg.RosPack().get_path('ar_commander'))

from configs.robotConfig import robotConfig
from ar_commander.msg import ControllerCmd, Decawave, TOF
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

RATE = 10 # Hz
TOF_DEFAULT = 9999
TOF_CONVERSION = 1000 # convert form sim units (m) to sensor units (mm)


class SimInterface:
    def __init__(self):
        # gazebo model states to Pose2D
        rospy.init_node("simInterface")

        # localization variables & covariances
        self.loc = Decawave()
        self.pose = Pose2D()
        self.pos = None
        self.theta = None
        self.cov_pos1 = None
        self.cov_pos2 = None
        self.cov_theta = None

        # measurement noise & robot params
        self.rcfg = robotConfig()
        self.pos_noise = rospy.get_param("position_noise")
        self.theta_noise = rospy.get_param("theta_noise")

        # control cmds
        self.control_cmds = None
        self.W_cmd = None
        self.phi_cmd = None

        # tof sensor data
        self.tof_msg = TOF()

        # publishers
        self.pose_pub = rospy.Publisher("pose", Pose2D, queue_size=10)
        self.decawave_pub = rospy.Publisher("sensor/decawave_measurement", Decawave, queue_size=10)
        self.tof_pub = rospy.Publisher("sensor/tof_data", TOF, queue_size=10)

        self.cmdP1_pub = rospy.Publisher("joint1_position_controller/command", Float64, queue_size=10)
        self.cmdP2_pub = rospy.Publisher("joint2_position_controller/command", Float64, queue_size=10)
        self.cmdP3_pub = rospy.Publisher("joint3_position_controller/command", Float64, queue_size=10)
        self.cmdP4_pub = rospy.Publisher("joint4_position_controller/command", Float64, queue_size=10)
        self.cmdV5_pub = rospy.Publisher("joint5_velocity_controller/command", Float64, queue_size=10)
        self.cmdV6_pub = rospy.Publisher("joint6_velocity_controller/command", Float64, queue_size=10)
        self.cmdV7_pub = rospy.Publisher("joint7_velocity_controller/command", Float64, queue_size=10)
        self.cmdV8_pub = rospy.Publisher("joint8_velocity_controller/command", Float64, queue_size=10)

        # subscribers
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelStatesCallback)
        rospy.Subscriber("controller_cmds", ControllerCmd, self.controlCmdsCallback)
        rospy.Subscriber("sim_sensor/tof1", LaserScan, self.tof1Callback)
        rospy.Subscriber("sim_sensor/tof2", LaserScan, self.tof2Callback)
        rospy.Subscriber("sim_sensor/tof3", LaserScan, self.tof3Callback)

    def tof1Callback(self, msg):
        data = msg.ranges[0] * TOF_CONVERSION
        self.tof_msg.tof1 = data if not math.isinf(data) else TOF_DEFAULT

    def tof2Callback(self, msg):
        data = msg.ranges[0] * TOF_CONVERSION
        self.tof_msg.tof2 = data if not math.isinf(data) else TOF_DEFAULT

    def tof3Callback(self, msg):
        data = msg.ranges[0] * TOF_CONVERSION
        self.tof_msg.tof3 = data if not math.isinf(data) else TOF_DEFAULT

    def controlCmdsCallback(self, msg):
        self.W_cmd = msg.omega_arr.data
        self.phi_cmd = msg.phi_arr.data

    def modelStates2Pose2D(self):
        self.pose.x = self.pos.x
        self.pose.y = self.pos.y
        self.pose.theta = self.theta  # sim outputs [-pi, pi]

    def modelStatesCallback(self, msg):
        try:
            idx = msg.name.index(rospy.get_namespace()[1:-1])
            self.pos = msg.pose[idx].position
            self.q = msg.pose[idx].orientation
            _, _, self.theta = euler_from_quaternion([self.q.x, self.q.y, self.q.z, self.q.w])

            self.modelStates2Pose2D()
            self.simLocalizationData()  # sim noisy localization data

        except ValueError:
            pass

    def simLocalizationData(self):
        # publish noisy localization data and transform to end of robot arms
        self.loc.x1.data = (self.pose.x - self.rcfg.L * np.sin(self.theta) + npr.normal(0.0, self.pos_noise)) # sensor on Y axis arm
        self.loc.y1.data = (self.pose.y + self.rcfg.L * np.cos(self.theta) + npr.normal(0.0, self.pos_noise))
        self.loc.x2.data = (self.pose.x + self.rcfg.L * np.cos(self.theta) + npr.normal(0.0, self.pos_noise)) # sensor on X axis arm
        self.loc.y2.data = (self.pose.y + self.rcfg.L * np.sin(self.theta) + npr.normal(0.0, self.pos_noise))
        self.loc.theta.data = self.pose.theta + npr.normal(0.0, self.theta_noise) # theta measurement

        # covariances: normal distribution: standard deviation^2
        self.loc.cov1.data = ((self.pos_noise ** 2) * np.eye(2)).reshape(-1)
        self.loc.cov2.data = ((self.pos_noise ** 2) * np.eye(2)).reshape(-1)
        self.loc.cov_theta.data = self.theta_noise ** 2

        # new measurement flags
        # self.loc.new_meas1.data = True
        # self.loc.new_meas2.data = True

    def publishSimMsgs(self):
        # Publish pose, tof and noisy localization data to GNC
        self.pose_pub.publish(self.pose)
        self.tof_pub.publish(self.tof_msg)
        if self.pos is not None:
            self.decawave_pub.publish(self.loc)

        # Publish cmds to motors
        if self.phi_cmd != None and self.W_cmd != None:
            self.cmdP1_pub.publish(self.phi_cmd[2])  # yaw_joint_l1 -> inner on X axis arm
            self.cmdP2_pub.publish(self.phi_cmd[3])  # yaw_joint_l2 -> outer on X axis arm
            self.cmdP3_pub.publish(self.phi_cmd[0])  # yaw_joint_r1 -> inner on Y axis arm
            self.cmdP4_pub.publish(self.phi_cmd[1])  # yaw_joint_r2 -> outer on Y axis arm
            self.cmdV5_pub.publish(self.W_cmd[2])  # drive_joint_l1 -> inner on X axis arm
            self.cmdV6_pub.publish(self.W_cmd[3])  # drive_joint_l2 -> outer on X axis arm
            self.cmdV7_pub.publish(self.W_cmd[0])  # drive_joint_r1 -> inner on Y axis arm
            self.cmdV8_pub.publish(self.W_cmd[1])  # drive_joint_r2 -> outer on Y axis arm

    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.publishSimMsgs()
            rate.sleep()


if __name__ == "__main__":
    interface = SimInterface()
    interface.run()
