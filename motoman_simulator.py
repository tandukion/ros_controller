#!/usr/bin/env python
#
# Copyright (c) 2020, Dwindra Sulistyoutomo
#

import sys
from scripts.robot_logic_controller import RobotLogicController


class MotomanRobotSimulator:
    def __init__(self, dof=6, home_pos=None):
        # Basic configuration
        self.robot_dof = dof
        self.home_pos = home_pos if home_pos else [0] * self.robot_dof

        # Start the Robot Logic Controller
        self.robot = RobotLogicController(robot="motoman", dof=dof, home_pos=self.home_pos)

    def simulate_io(self, address, value):
        self.robot.io_interface_server.set_io_port(address, value)


if __name__ == '__main__':
    # Edit this for initial position. Simulation will make the robot start from home position
    home_pos = [0, 0, 0, 0, 0, 0]
    robot = MotomanRobotSimulator(home_pos=home_pos)
    while 1:
        continue
