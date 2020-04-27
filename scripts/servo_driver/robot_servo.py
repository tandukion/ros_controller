#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

from adafruit_motor.servo import Servo  # import from adafruit-circuitpython-motor


class RobotServo (Servo):
    """
    Class that serves as servo for the robot.

    This class handles servo commands with range from -min_value to +max_value, not from 0 to actuation range.
    With assumption that servo can achieve the target angle with given load, this class returns the current angle of
    the servo based on last target angle

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
        """
        Set the angle of the servo
        :param set_angle: target angle to set
        :return:
        """
        # remap the range from 0-180 deg to -90-90 deg
        self.angle = set_angle + 90
        self._current_angle = set_angle
        return

    def getAngle(self):
        """
        Returns the current angle of the servo
        :return: current angle
        """
        return self._current_angle
