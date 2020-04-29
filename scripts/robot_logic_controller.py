#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

try:
    import busio

    from board import SCL, SDA              # import from adafruit_blinka
    from adafruit_pca9685 import PCA9685    # import from adafruit-circuitpython-pca9685

    from scripts.servo_driver.robot_servo import RobotServo
except ImportError:
    print("Not using real board")

from scripts.robot_state_machine import *
from scripts.ros_comm.simple_message import *
from scripts.ros_comm.joint_streamer_server import JointStreamerServer
from scripts.ros_comm.robot_state_server import RobotStateServer
from scripts.ros_comm.io_interface_server import IoInterfaceServer
from scripts.motion_controller.motion_controller import *

# dummy
joint_pos_dummy = [0, 0, 0, 0, 0, 0]
robot_status_dummy = [1, 0, 0, 0, 0, 1, 0]

# CONFIG
ROBOT_DOF = 6
# HOME_POSITION = [0, 30, 0, 0, 30, 0]
HOME_POSITION = [0, -30, -30, 0, -90, 0]
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

JOINT_STREAM_PORT = 11000
ROBOT_STATE_PORT = 11002
IO_INTERFACE_PORT = 11003
MOTOMAN_JOINT_STREAM_PORT = 50240
MOTOMAN_ROBOT_STATE_PORT = 50241
MOTOMAN_IO_INTERFACE_PORT = 50242


class RobotLogicController:
    def __init__(self, sim=False):
        # Create Robot Servo
        if not sim:
            i2c = busio.I2C(SCL, SDA)
            pca = PCA9685(i2c)
            pca.frequency = 50

            self.robot_servo = []
            for i in range(ROBOT_DOF):
                self.robot_servo.append(RobotServo(pca.channels[i]))

        # TODO: make joint position, robot state class
        # Create Motion Controller for Joint Position
        # self.joint_pos = copy.deepcopy(joint_pos_dummy)
        self.joint_names = JOINT_NAMES
        initial_joint_pos = [0]*len(self.joint_names)
        self.joint_pos = initial_joint_pos
        self.home_pos = HOME_POSITION

        if not sim:
            self.motion_controller = MotionController(initial_joint_pos, self.home_pos, robot_servo=self.robot_servo)
        else:
            self.motion_controller = MotionController(initial_joint_pos, self.home_pos)

        # Create Robot Status class
        # self.robot_status = copy.deepcopy(robot_status_dummy)
        self.goal_joint_pos = []
        for i in range(ROBOT_DOF):
            self.goal_joint_pos.append(0)

        self.robot_status = RobotStatus()

        # Start State Machine
        self._state_machine = RobotStateMachine(model=self)

        # Start Communication Server
        self.server_shutdown = False
        self.servers = []
        self.start_servers()

        self.trig_initialized()

    def start_servers(self):
        """
        Start the servers to receive any Simple Message
        """
        self.joint_streamer_server = JointStreamerServer(controller=self, port=JOINT_STREAM_PORT)
        # self.joint_streamer_server = JointStreamerServer(controller=self, port=MOTOMAN_JOINT_STREAM_PORT)
        self.joint_streamer_server.start_server()
        self.robot_state_server = RobotStateServer(controller=self, port=ROBOT_STATE_PORT)
        # self.robot_state_server = RobotStateServer(controller=self, port=MOTOMAN_ROBOT_STATE_PORT)
        self.robot_state_server.start_server()
        self.io_interface_server = IoInterfaceServer(controller=self, port=IO_INTERFACE_PORT)
        # self.io_interface_server = IoInterfaceServer(controller=self, port=MOTOMAN_IO_INTERFACE_PORT)
        self.io_interface_server.start_server()

        self.servers.append(self.joint_streamer_server)
        self.servers.append(self.robot_state_server)
        self.servers.append(self.io_interface_server)

        # Start thread for controlling the servers
        t = threading.Thread(target=self.server_controller, args=())
        t.setDaemon(True)
        t.start()

    def server_controller(self):
        """
        Control the connection of the servers
        """
        while not self.server_shutdown:
            client_disconnected = False
            # Check if any server is disconnected from its client.
            for server in self.servers:
                if server.server.is_shutdown:
                    client_disconnected = True
                    break

            #  Restart any server connected which is still not disconnected
            if client_disconnected:
                for server in self.servers:
                    # Close socket if it still connected
                    # Usually blocked by socket.recv() even if the client is already disconnected
                    if server.server.is_connected:
                        server.server.close_socket()

                    # Restart any shutdown server
                    if server.server.is_shutdown:
                        server.server.restart_server()

    """
    State callback handlers
    """
    def _on_state_enter(self):
        """
        General callback on entering any state
        """
        # print("Entering state: ", self.state)

    def _on_state_exit(self):
        """
        General callback on exiting any state
        """
        pass

    def _on_state_initialized(self):
        """
        Callback on entering initialized state
        """
        print("Initialized.")
        print("Moving robot to HOME position")
        self.motion_controller.move_to_home_from_uninitialized()
        print("Robot at HOME")
        self.trig_standby()

    def _on_state_in_motion(self):
        """
        Callback in motion
        """
        # TODO: How to move robot here
        # print("Moving robot")
        # for i in range(len(self.joint_pos)):
        #     self.joint_pos[i] = self.goal_joint_pos[i]

        # Set the robot state into in_motion
        self.robot_status.set_motion(TRUE)

        self.trig_motion_completed()

    def _on_state_at_goal(self):
        """
        Callback on arriving at goal point
        """
        # print("Reach goal trajectory point")

        # Set the robot state into NOT in_motion
        self.robot_status.set_motion(FALSE)

        self.trig_standby()

    """
    Communication callback handlers
    """
    def get_joint_pos(self):
        self.joint_pos = self.motion_controller.get_joint_position()
        return self.joint_pos

    def get_robot_status(self):
        return self.robot_status.get_robot_status()

    def set_robot_motion_possible(self, value=True):
        self.robot_status.set_motion_possible(value)

    """
    Motion signal handlers
    """
    def stop_motion(self):
        self.motion_controller.stop()

    def move_robot(self, stream_message):
        # for i in range(ROBOT_DOF):
        #     self.goal_joint_pos[i] = goal_angle[i]

        # Simple Joint Position
        if stream_message.msg_type == JOINT_POSITION or stream_message.msg_type == JOINT_TRAJ_PT:
            # check if it a new trajectory
            if stream_message.seq_num == 0:
                print("Got new trajectory")
                self.motion_controller.trigger_new_trajectory()
            else:
                self.motion_controller.add_motion_waypoint(stream_message)

        elif stream_message.msg_type == JOINT_TRAJ_PT_FULL:
            self.motion_controller.add_motion_waypoint(stream_message)

        self.trig_motion()
