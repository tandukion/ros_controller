#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import unittest
from ..scripts.robot_state_machine import RobotStateMachine
from time import sleep


class StateMachineTester(RobotStateMachine):
    def __init__(self):
        # Initialize the state machine
        RobotStateMachine.__init__(self)

    def _on_state_initialized(self):
        print("Initializing")

    def _on_state_in_motion(self):
        print("In Motion")

    def _on_state_at_goal(self):
        print("Reach Goal")

    def _on_state_enter(self):
        print("Entering state: ", self.state)

    def _on_state_exit(self):
        pass


class TestStateMachine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.state_machine = StateMachineTester()

    def test_1_initialize(self):
        sleep(1)
        self.state_machine.trig_initialized()

    def test_2_standby(self):
        sleep(1)
        self.state_machine.trig_standby()

    def test_3_motion_success(self):
        sleep(1)
        self.state_machine.trig_motion()
        sleep(1)
        self.state_machine.trig_motion_completed()
        sleep(1)
        self.state_machine.trig_standby()

    def test_4_motion_error(self):
        sleep(1)
        self.state_machine.trig_motion()
        sleep(1)
        self.state_machine.trig_motion_error()


if __name__ == '__main__':
    unittest.main()
