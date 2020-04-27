#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#
from scripts.ros_comm.ros_comm import *

joint_streamer_publisher = JointStreamerServer()
joint_streamer_publisher.start_server()

robot_state_publisher = RobotStateServer()
robot_state_publisher.start_server()

while 1:
    continue
