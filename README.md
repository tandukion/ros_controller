# servorobot_raspi-controller
This repository acts as a controller for servo motors using Raspberry Pi.

## Hardware
The controller controls the servos using PCA9685 servo driver.

I2C setup is needed to be enabled on Raspberry Pi.

## Library dependencies
In order to run the program, libraries for using servo with Raspberry Pi are needed.

#### RPi.GPIO library 
    sudo pip3 install RPI.GPIO 
#### Adafruit blinka library 
    sudo pip3 install adafruit-blinka 
#### PCA9685 library 
    sudo pip3 install adafruit-circuitpython-pca9685 
#### Servo and Motor library 
    sudo pip3 install adafruit-circuitpython-motor 
