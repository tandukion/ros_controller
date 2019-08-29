#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#
import sys
from src.robot_ros_comm import *
from time import sleep

joint_streamer_publisher = JointStreamerServer()
joint_streamer_publisher.start_server()

robot_state_publisher = RobotStateServer()
robot_state_publisher.start_server()
