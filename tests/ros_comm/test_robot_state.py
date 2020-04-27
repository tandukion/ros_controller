#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#
from scripts.ros_comm.robot_state_server import *
from time import sleep

robot_state_publisher = RobotStateServer()
robot_state_publisher.start_server()

while 1:
    for i in range(60):
        joint_pos_dummy[0] += 1
        sleep(0.01)

    sleep(3)
    for i in range(120):
        joint_pos_dummy[0] -= 1
        sleep(0.01)

    sleep(3)
    for i in range(60):
        joint_pos_dummy[0] += 1
        sleep(0.01)

    sleep(3)
