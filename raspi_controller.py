#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import threading
import time
import busio

from board import SCL, SDA              # import from adafruit_blinka
from adafruit_motor import servo        # import from adafruit-circuitpython-motor
from adafruit_pca9685 import PCA9685    # import from adafruit-circuitpython-pca9685


class MG996Servo (Servo):
    """
    Define for MG996 servo
    """

    def __init__(self,
                 channel,
                 min_pulse=500,
                 max_pulse=2500,
                 **kwargs):
        super(MG996, self).__init__(channel, min_pulse, max_pulse)

        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = 1500
        self.upper_angle_limit = 90
        self.lower_angle_limit = -90

        self._current_angle = self.zero_pulse

    def setAngle(self, angle_deg):
        if angle_deg > 0:
            pulse = self.zero_pulse + angle_deg * (self.max_pulse - self.zero_pulse) / self.upper_angle_limit
        else:
            pulse = self.zero_pulse + angle_deg * (self.min_pulse - self.zero_pulse) / self.lower_angle_limit

        self._current_angle = angle_deg

    def getAngle(self):
        return self._current_angle

if __name__ == '__main__':
    i2c = busio.I2C(SCL, SDA)

    # Create a simple PCA9685 class instance.
    pca = PCA9685(i2c)
    pca.frequency = 50
