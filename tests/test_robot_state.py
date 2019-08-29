#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#
import sys
from src.robot_ros_comm import *
from time import sleep

robot_state_publisher = RobotStateServer()
robot_state_publisher.start_server()

while 1:
    for i in range(60):
        joint_pos[0] += 1
        sleep(0.01)

    sleep(3)
    for i in range(120):
        joint_pos[0] -= 1
        sleep(0.01)

    sleep(3)
    for i in range(60):
        joint_pos[0] += 1
        sleep(0.01)

    sleep(3)