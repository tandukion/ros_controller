#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#
from scripts.ros_comm.joint_streamer_server import JointStreamerServer
from scripts.ros_comm.robot_state_server import RobotStateServer

joint_streamer_publisher = JointStreamerServer()
joint_streamer_publisher.start_server()

robot_state_publisher = RobotStateServer()
robot_state_publisher.start_server()

while 1:
    continue
