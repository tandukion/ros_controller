#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import time
import busio

from board import SCL, SDA              # import from adafruit_blinka
from adafruit_pca9685 import PCA9685    # import from adafruit-circuitpython-pca9685

from src.robot_servo import RobotServo

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
