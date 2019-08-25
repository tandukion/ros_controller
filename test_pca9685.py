from board import SCL, SDA
import busio
import time

# https://github.com/adafruit/Adafruit_CircuitPython_Motor
from adafruit_motor import servo
# Import the PCA9685 module.
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 50

servo = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)

for i in range(180):
    servo.angle = i

time.sleep(2)
for i in range(180):
    servo.angle = 180 - i

pca.deinit()
