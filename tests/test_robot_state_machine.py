#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

from scripts.robot_state_machine import *
from time import sleep


class StateMachineTester:
    def __init__(self):
        self._state_machine = RobotStateMachine(model=self)

    def _on_state_initialized(self):
        print("Initializing")

    def _on_state_enter(self):
        print("Entering state: ", self.state)

    def _on_state_exit(self):
        pass


tester = StateMachineTester()

sleep(2)
tester.trig_initialized()

sleep(2)
tester.trig_standby()

sleep(2)
tester.trig_motion()

sleep(2)
tester.trig_motion_completed()

sleep(2)
tester.trig_standby()

sleep(2)
tester.trig_motion()

sleep(2)
tester.trig_motion_error()
