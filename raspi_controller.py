#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import threading
import time
import busio

from board import SCL, SDA              # import from adafruit_blinka
from adafruit_motor.servo import Servo  # import from adafruit-circuitpython-motor
from adafruit_pca9685 import PCA9685    # import from adafruit-circuitpython-pca9685


class RobotServo (Servo):
    """
    Default value defined for MG996 servo
    """

    def __init__(self,
                 channel,
                 min_pulse=500,
                 max_pulse=2500,
                 zero_pulse=1500,
                 actuation_range=180,
                 **kwargs):
        super(RobotServo, self).__init__(channel, actuation_range=actuation_range,
                                        min_pulse=min_pulse, max_pulse=max_pulse)

        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        self.upper_angle_limit = actuation_range / 2
        self.lower_angle_limit = - (actuation_range) / 2
        self._current_angle = 0

    def setAngle(self, set_angle):
        # remap the range from 0-180 deg to -90-90 deg
        self.angle = set_angle + 90
        self._current_angle = set_angle
        return

    def getAngle(self):
        return self._current_angle


if __name__ == '__main__':
    i2c = busio.I2C(SCL, SDA)

    # Create a simple PCA9685 class instance.
    pca = PCA9685(i2c)
    pca.frequency = 50


    # Test RobotServoclass
    servo = RobotServo(pca.channels[0])
    angle = 0

    for i in range(90):
        angle = angle + 1
        servo.setAngle(angle)
        print(servo.getAngle())

    time.sleep(2)
    for i in range(180):
        angle = angle - 1
        servo.setAngle(angle)
        print(servo.getAngle())

    time.sleep(2)
    for i in range(90):
        angle = angle + 1
        servo.setAngle(angle)
        print(servo.getAngle())
