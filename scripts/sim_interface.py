#!/usr/bin/env python

import rospy
import numpy as np
import numpy.random as npr
import numpy.linalg as npl
import sys

from ar_commander.msg import MotorCmd, Decawave
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

sys.path.append(rospy.get_param("AR_SIM_DIR"))
sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

import config.sim_interface_params as params
import configs.robot_v1 as rcfg

RATE = 10



class SimInterface:
    """
    Simulation interface node
    Handles data transfer to and from the autononmy stack.
    Actuates motors based on data from 'motor_cmds' message.
    Simulates noisy localization data based on parameters located
    in 'configs/sim_interface_params'. Published at rate defined
    globally in script.

    Data in:
        - Motor commands

    Data out:
        - Noisy localization

    """

    def __init__(self):
        # gazebo model states to Pose2D
        rospy.init_node("sim_interface")

        # localization variables & covariances
        self.loc = Decawave()
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
        self.decawave_pub = rospy.Publisher("sensor/decawave_measurement", Decawave, queue_size=10)

        self.cmdP1_pub = rospy.Publisher("joint1_position_controller/command", Float64, queue_size=10) # postion publisher: stepper motor
        self.cmdP2_pub = rospy.Publisher("joint2_position_controller/command", Float64, queue_size=10)
        self.cmdP3_pub = rospy.Publisher("joint3_position_controller/command", Float64, queue_size=10)
        self.cmdP4_pub = rospy.Publisher("joint4_position_controller/command", Float64, queue_size=10)

        self.cmdV5_pub = rospy.Publisher("joint5_velocity_controller/command", Float64, queue_size=10) # velocity publisher: dc motor
        self.cmdV6_pub = rospy.Publisher("joint6_velocity_controller/command", Float64, queue_size=10)
        self.cmdV7_pub = rospy.Publisher("joint7_velocity_controller/command", Float64, queue_size=10)
        self.cmdV8_pub = rospy.Publisher("joint8_velocity_controller/command", Float64, queue_size=10)

        # subscribers
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelStatesCallback)
        rospy.Subscriber("motor_cmds", MotorCmd, self.motorCmdsCallback)

    def motorCmdsCallback(self, msg):
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
            _, _, self.theta = euler_from_quaternion(
                [self.q.x, self.q.y, self.q.z, self.q.w]
            )

            self.modelStates2Pose2D()
            self.simLocalizationData()  # sim noisy localization data

        except ValueError:
            pass

    def simLocalizationData(self):
        # publish noisy localization data and transform to end of robot arms
        self.loc.x1.data = (self.pose.x - rcfg.L * np.sin(self.theta) + npr.normal(0.0, self.pos_noise)) # sensor on Y axis arm
        self.loc.y1.data = (self.pose.y + rcfg.L * np.cos(self.theta) + npr.normal(0.0, self.pos_noise))
        self.loc.x2.data = (self.pose.x + rcfg.L * np.cos(self.theta) + npr.normal(0.0, self.pos_noise)) # sensor on X axis arm
        self.loc.y2.data = (self.pose.y + rcfg.L * np.sin(self.theta) + npr.normal(0.0, self.pos_noise))
        self.loc.theta.data = self.pose.theta + npr.normal(0.0, self.theta_noise) # theta measurement

        # covariances
        self.loc.cov1.data = ((self.pos_noise ** 2) * np.eye(2)).reshape(-1)
        self.loc.cov2.data = ((self.pos_noise ** 2) * np.eye(2)).reshape(-1)
        self.loc.cov_theta.data = self.theta_noise ** 2

        # new measurement flags
        self.loc.new_meas1.data = True
        self.loc.new_meas2.data = True

    def publishSimMsgs(self):
        # Publish pose and noisy localization data to GNC
        self.pose_pub.publish(self.pose)
        if self.pos is not None:
            self.decawave_pub.publish(self.loc)

        # Publish cmds to motors
        if self.phi_cmd != None and self.W_cmd != None:
            self.cmdP1_pub.publish(self.phi_cmd[2])  # yaw_joint_l1 -> inner on X axis arm
            self.cmdP2_pub.publish(self.phi_cmd[3])  # yaw_joint_l2 -> outer on X axis arm
            self.cmdP3_pub.publish(self.phi_cmd[0])  # yaw_joint_r1 -> inner on Y axis arm
            self.cmdP4_pub.publish(self.phi_cmd[1])  # yaw_joint_r2 -> outer on Y axis arm
            self.cmdV5_pub.publish(self.W_cmd[2])    # drive_joint_l1 -> inner on X axis arm
            self.cmdV6_pub.publish(self.W_cmd[3])    # drive_joint_l2 -> outer on X axis arm
            self.cmdV7_pub.publish(self.W_cmd[0])    # drive_joint_r1 -> inner on Y axis arm
            self.cmdV8_pub.publish(self.W_cmd[1])    # drive_joint_r2 -> outer on Y axis arm

    def run(self):
        rate = rospy.Rate(RATE)  
        while not rospy.is_shutdown():
            self.publishSimMsgs()
            rate.sleep()


if __name__ == "__main__":
    interface = SimInterface()
    interface.run()
