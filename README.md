# ros_controller
Robot Controller which accept Joint Trajectory commands from ROS via TCP/IP using [Simple Message](http://wiki.ros.org/simple_message) protocol.

The controller accept the following interfaces:
- Joint Streaming
- Robot Status
- IO Interface

This controller can be used as robot simulator and real robot controller
> :warning: Currently support only Python 3

## Robot Simulator

#### `Yaskawa Motoman robot simulator`
Run the motoman simulator to run the simulator
~~~
python motoman_simulator.py
~~~
> Edit the script to add home position or additional IO simulation

## Robot Controller
Currently available real robot controller:
#### `6 DOF Servo robot arm`
The servo robot is defined at: [https://github.com/tandukion/servorobot](https://github.com/tandukion/servorobot)
> Please use the actual servorobot_controller which includes both the servo driver and this controller. <br>
[https://github.com/tandukion/servorobot_controller](https://github.com/tandukion/servorobot_controller)