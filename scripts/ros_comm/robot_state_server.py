#!/usr/bin/env python
#
# Copyright (c) 2020, Dwindra Sulistyoutomo
#
# Currently only support Python 3.x

import time
import sched
from math import degrees, radians

from .simple_message import *
from .ros_comm import write_messages, BytesIO
from .message_server import MessageServer

# dummy
joint_pos_dummy = [0, 0, 0, 0, 0, 0]


class RobotStateServer (MessageServer):
    """
    This class acts as server for sending robot state (joint position + robot status)
    """
    def __init__(self, port=11002, loop_rate=40, stat_loop=10, controller=None):
        """
        :param port: TCP port for Robot State. By default = 11002 defined from ROS-Industrial
        :param loop_rate: frequency for sending robot state
        :param stat_loop: Robot Status frequency based on Joint Position messages
        :param controller: robot controller which handles the joint position
        :type  controller: RobotLogicController
        """
        super(RobotStateServer, self).__init__(port)
        self.controller = controller
        self.port = port
        self.inbound_handler = self.publish_handler
        self.loop_rate = loop_rate
        self.seq = 0
        self.handle = True
        self.stat_count = 0  # counter for sending robot status every N=stat_loop joint status
        self.stat_loop = stat_loop

        # Create scheduler
        self.pub_period = 1/loop_rate
        self.scheduler = sched.scheduler(time.time, time.sleep)

        # Create message classes to send
        self.joint_pos_message = SimpleMessage()
        self.joint_pos_message.set_header(JOINT_POSITION, TOPIC, 0)
        self.robot_status_message = SimpleMessage()
        self.robot_status_message.set_header(STATUS, TOPIC, 0)

    def publish_handler(self, sock):
        """
        Main handler for publishing Joint Position and Robot State.
        Each handler run on schedule based on publish rate
        :param sock: client socket, given from ServerSocket
        """
        self.handle = True
        while self.handle:
            self.scheduler.enter(self.pub_period, 1, self.joint_position_publisher(sock))
            self.scheduler.enter(self.pub_period * self.stat_loop, 1, self.robot_status_publisher(sock))

    def joint_position_publisher(self, sock):
        """
        Handler to publish Joint Position message
        """
        # Create Joint Position message  #TODO: create how get the current joint position here
        # Get current Joint Position
        if self.controller:
            # get Joint Position from main logic controller
            joint_pos = self.controller.get_joint_pos()
        else:
            # if not using logic controller, just return dummy (for testing)
            joint_pos = copy.deepcopy(joint_pos_dummy)

        # Convert to radians
        joint_pos_rad = list(map(radians, joint_pos))
        self.joint_pos_message.assign_data(joint_pos_rad)
        # set seq_num to 0 for Joint Position Topic
        self.seq = 0

        # Serialize + sending message
        self.handle = write_messages(BytesIO(), sock, self.joint_pos_message, self.seq)

    def robot_status_publisher(self, sock):
        """
        Handler to publish Robot State message
        """
        # Create Robot Status message  #TODO: create how get the current robot status here
        # Get current Robot Status
        if self.controller:
            # get robot status from logic controller
            robot_status = self.controller.get_robot_status()
        else:
            # if not using logic controller, just return dummy (for testing)
            dummy_status = [1, 0, 0, 0, 0, 1, 0]
            robot_status = dummy_status

        self.robot_status_message.assign_data(robot_status)

        # Serialize + sending message, no need to use seq for Robot Status
        self.handle = write_messages(BytesIO(), sock, self.robot_status_message)