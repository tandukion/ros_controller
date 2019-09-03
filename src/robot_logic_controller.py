#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import copy
import busio

from board import SCL, SDA              # import from adafruit_blinka
from adafruit_pca9685 import PCA9685    # import from adafruit-circuitpython-pca9685

from src.robot_servo import RobotServo
from src.robot_state_machine import *
from src.robot_ros_comm import RobotStateServer, JointStreamerServer
from src.motion_controller import *

# dummy
joint_pos_dummy = [0, 0, 0, 0, 0, 0]
robot_status_dummy = [1, 0, 0, 0, 0, 1, 0]
ROBOT_DOF = 6

class RobotLogicController:
    def __init__(self):
        # Create Robot Servo
        i2c = busio.I2C(SCL, SDA)
        pca = PCA9685(i2c)
        pca.frequency = 50

        self.robot_servo = []
        for i in range(ROBOT_DOF):
            self.robot_servo.append(RobotServo(pca.channels[0]))

        # TODO: make joint position, robot state class
        # Create Motion Controller for Joint Position
        # self.joint_pos = copy.deepcopy(joint_pos_dummy)
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        initial_joint_state = [0]*len(self.joint_names)
        self.joint_pos = initial_joint_state
        self.motion_control = MotionController(initial_joint_state, robot_servo=self.robot_servo)

        # Create Robot Status class
        # self.robot_status = copy.deepcopy(robot_status_dummy)
        self.goal_joint_pos = []
        for i in range(ROBOT_DOF):
            self.goal_joint_pos.append(0)

        self.robot_status = RobotStatus()



        # Start State Machine
        self._state_machine = RobotStateMachine(model=self)

        # Create Communication Server
        self._robot_state_pub = RobotStateServer(controller=self)
        self._robot_state_pub.start_server()
        self._joint_streamer_pub = JointStreamerServer(controller=self)
        self._joint_streamer_pub.start_server()

        self.trig_initialized()

    """
    State callback handlers
    """
    def _on_state_enter(self):
        """
        General callback on entering any state
        """
        print("Entering state: ", self.state)

    def _on_state_exit(self):
        """
        General callback on exiting any state
        """
        pass

    def _on_state_initialized(self):
        """
        Callback on entering initialized state
        """
        print("Initialized.")
        self.trig_standby()

    def _on_state_in_motion(self):
        """
        Callback in motion
        """
        # TODO: How to move robot here
        print("Moving robot")
        for i in range(len(self.joint_pos)):
            self.joint_pos[i] = self.goal_joint_pos[i]

        self.trig_motion_completed()

    def _on_state_at_goal(self):
        """
        Callback on arriving at goal point
        """
        print("Reach goal trajectory point")
        self.trig_standby()

    """
    Communication callback handlers
    """
    def get_joint_pos(self):
        self.joint_pos = self.motion_control.get_joint_position()
        return self.joint_pos

    def get_robot_status(self):
        return self.robot_status.get_robot_status()

    def move_robot(self, goal_angle):
        for i in range(ROBOT_DOF):
            self.goal_joint_pos[i] = goal_angle[i]
        self.trig_motion()
