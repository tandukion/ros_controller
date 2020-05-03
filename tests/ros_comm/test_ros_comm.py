#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import unittest
from ...scripts.ros_comm.joint_streamer_server import JointStreamerServer
from ...scripts.ros_comm.robot_state_server import RobotStateServer


class TestRosComm(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        joint_streamer_publisher = JointStreamerServer(port=11000)
        joint_streamer_publisher.start_server()

        robot_state_publisher = RobotStateServer(port=11002)
        robot_state_publisher.start_server()

    def test_run(self):
        while 1:
            continue


if __name__ == '__main__':
    unittest.main()

