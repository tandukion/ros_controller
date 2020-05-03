#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

# Created to test as a client from Fanuc Roboguide to know the simple message sent from controller

import unittest
from ..scripts.ros_comm.ros_comm import *


class TestRoboguideClient(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robotclient = ClientSocket(ip_addr="192.168.4.61", port=11002)
        cls.robotclient.start()

    def test_run(self):
        while 1:
            if self.robotclient.message.msg_type == JOINT_POSITION:
                if self.robotclient.message.comm_type == 1:
                        str = ''
                        for d in self.robotclient.message.data:
                            str += "%.2f, " %d
                        print("Current Joint State:", str, end="\r", flush=True)


if __name__ == '__main__':
    unittest.main()
