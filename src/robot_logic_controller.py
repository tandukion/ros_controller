#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import copy
from src.robot_state_machine import RobotStateMachine
from src.robot_ros_comm import RobotStateServer, JointStreamerServer

# dummy
joint_pos_dummy = [0, 0, 0, 0, 0, 0]
robot_status_dummy = [1, 0, 0, 0, 0, 1, 0]


class RobotLogicController:
    def __init__(self):
        # TODO: make joint position, robot state class
        self.joint_pos = copy.deepcopy(joint_pos_dummy)
        self.robot_status = copy.deepcopy(robot_status_dummy)
        self.goal_joint_pos = []

        # Starting State Machine
        self._state_machine = RobotStateMachine(model=self)

        # Creating Communication Server
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
    """
    def get_joint_pos(self):
        return self.joint_pos

    def get_robot_status(self):
        return self.robot_status

    def move_robot(self, goal_angle):
        self.goal_joint_pos = copy.deepcopy(goal_angle)
        self.trig_motion()
