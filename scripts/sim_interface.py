#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as npl
import sys

from ar_commander.msg import ControllerCmd, Decawave
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

sys.path.append(rospy.get_param("AR_SIM_DIR"))
sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

import config.sim_interface_params as params
import configs.robot_v1 as rcfg

# take model states msg and publish a curr pose in Pose2D format


class SimInterface:
    def __init__(self):
        # gazebo model states to Pose2D
        rospy.init_node("sim_interface")

        # localization variables & covariances
        self.pose = Pose2D()
        self.pos = None
        self.theta = None
        self.cov_pos1 = None
        self.cov_pos2 = None
        self.cov_theta = None

        # control cmds
        self.control_cmds = None
        self.W_cmd = None
        self.phi_cmd = None

        # measurement noise params
        self.pos_noise = params.position_noise
        self.theta_noise = params.theta_noise

        # publishers
        self.pose_pub = rospy.Publisher("pose", Pose2D, queue_size=10)
        self.decawave_pub = rospy.Publisher(
            "sensor/decawave_measurement", Decawave, queue_size=10
        )

        self.cmdP1_pub = rospy.Publisher(
            "joint1_position_controller/command", Float64, queue_size=10
        )
        self.cmdP2_pub = rospy.Publisher(
            "joint2_position_controller/command", Float64, queue_size=10
        )
        self.cmdP3_pub = rospy.Publisher(
            "joint3_position_controller/command", Float64, queue_size=10
        )
        self.cmdP4_pub = rospy.Publisher(
            "joint4_position_controller/command", Float64, queue_size=10
        )
        self.cmdV5_pub = rospy.Publisher(
            "joint5_velocity_controller/command", Float64, queue_size=10
        )
        self.cmdV6_pub = rospy.Publisher(
            "joint6_velocity_controller/command", Float64, queue_size=10
        )
        self.cmdV7_pub = rospy.Publisher(
            "joint7_velocity_controller/command", Float64, queue_size=10
        )
        self.cmdV8_pub = rospy.Publisher(
            "joint8_velocity_controller/command", Float64, queue_size=10
        )

        # subscribers
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_statesCallback)
        rospy.Subscriber("controller_cmds", ControllerCmd, self.control_cmdsCallback)

    def control_cmdsCallback(self, msg):
        self.W_cmd = msg.omega_arr.data
        self.phi_cmd = msg.phi_arr.data

    def modelStates2Pose2D(self):
        self.pose.x = self.pos.x
        self.pose.y = self.pos.y
        self.pose.theta = self.theta  # sim outputs [-pi, pi]

    def model_statesCallback(self, msg):
        try:
            idx = msg.name.index(rospy.get_namespace()[1:-1])
            self.pos = msg.pose[idx].position
            self.q = msg.pose[idx].orientation
            _, _, self.theta = euler_from_quaternion(
                [self.q.x, self.q.y, self.q.z, self.q.w]
            )
            self.modelStates2Pose2D()

            # sim noisy localization data from decawave sensors
            loc_data = self.getLocalizationData()
            self.decawave_pub.publish(loc_data)
        except ValueError:
            pass

    def getLocalizationData(self):
        loc = Decawave()
        # sensor on Y axis arm
        loc.x1.data = (
            self.pose.x
            - rcfg.L * np.sin(self.theta)
            + np.random.uniform(-self.pos_noise, self.pos_noise)
        )
        loc.y1.data = (
            self.pose.y
            + rcfg.L * np.cos(self.theta)
            + np.random.uniform(-self.pos_noise, self.pos_noise)
        )
        # sensor on X axis arm
        loc.x2.data = (
            self.pose.x
            + rcfg.L * np.cos(self.theta)
            + np.random.uniform(-self.pos_noise, self.pos_noise)
        )
        loc.y2.data = (
            self.pose.y
            + rcfg.L * np.sin(self.theta)
            + np.random.uniform(-self.pos_noise, self.pos_noise)
        )
        # theta
        loc.theta.data = self.pose.theta + np.random.uniform(
            -self.theta_noise, self.theta_noise
        )

        # covariances: normal distribution: standard deviation^2
        loc.cov1.data = ((self.pos_noise ** 2) * np.eye(2)).reshape(-1)
        loc.cov2.data = ((self.pos_noise ** 2) * np.eye(2)).reshape(-1)
        loc.cov_theta.data = self.theta_noise ** 2

        return loc

    def publishSimMsgs(self):
        # Publish pose to GNC
        self.pose_pub.publish(self.pose)

        # Publish GNC cmds to sim joints
        if self.phi_cmd != None and self.W_cmd != None:
            self.cmdP1_pub.publish(
                self.phi_cmd[2]
            )  # yaw_joint_l1 -> inner on X axis arm
            self.cmdP2_pub.publish(
                self.phi_cmd[3]
            )  # yaw_joint_l2 -> outer on X axis arm
            self.cmdP3_pub.publish(
                self.phi_cmd[0]
            )  # yaw_joint_r1 -> inner on Y axis arm
            self.cmdP4_pub.publish(
                self.phi_cmd[1]
            )  # yaw_joint_r2 -> outer on Y axis arm
            self.cmdV5_pub.publish(
                self.W_cmd[2]
            )  # drive_joint_l1 -> inner on X axis arm
            self.cmdV6_pub.publish(
                self.W_cmd[3]
            )  # drive_joint_l2 -> outer on X axis arm
            self.cmdV7_pub.publish(
                self.W_cmd[0]
            )  # drive_joint_r1 -> inner on Y axis arm
            self.cmdV8_pub.publish(
                self.W_cmd[1]
            )  # drive_joint_r2 -> outer on Y axis arm

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publishSimMsgs()
            rate.sleep()


if __name__ == "__main__":
    interface = SimInterface()
    interface.run()
