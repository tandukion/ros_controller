#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import unittest
from ..scripts.robot_logic_controller import RobotLogicController


class TestRobotLogic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = RobotLogicController(sim=True, robot="default", dof=6, home_pos=[0, 0, 0, 0, 0, 0])

    def test_run(self):
        while 1:
            continue


if __name__ == '__main__':
    unittest.main()
