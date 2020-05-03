#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

from scripts.robot_logic_controller import RobotLogicController


robot = RobotLogicController(sim=True, robot="default", dof=6, home_pos=[0, 0, 0, 0, 0, 0])

while 1:
    continue
