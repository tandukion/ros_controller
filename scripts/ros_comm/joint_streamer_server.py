#!/usr/bin/env python
#
# Copyright (c) 2020, Dwindra Sulistyoutomo
#
# Currently only support Python 3.x

import time
from math import degrees, radians

from scripts.ros_comm.simple_message import *
from scripts.ros_comm.ros_comm import read_messages, write_messages, BytesIO
from scripts.ros_comm.message_server import MessageServer

# dummy
joint_pos_dummy = [0, 0, 0, 0, 0, 0]


class JointStreamerServer (MessageServer):
    """
    This class acts as server for processing Joint Stream motion control from ROS
    """
    def __init__(self, controller=None, port=11000, loop_rate=42):
        """
        :param port: TCP port for Joint Stream. By default = 11000 defined from ROS-Industrial
        :param loop_rate: frequency for sending processing packet
        """
        super(JointStreamerServer, self).__init__(port)
        self.controller = controller
        self.port = port
        self.inbound_handler = self.joint_streamer_handler
        self.loop_rate = loop_rate
        self.handle_done = False

    def joint_streamer_handler(self, sock):
        """
        Handler for Joint Streamer messages from ROS.
        Read the Joint Stream Trajectory Point message and reply with Joint-Position-type message
        """
        handle_done = False
        buff_size = 4096

        # create incoming packet and out coming packet
        joint_stream_message = SimpleMessage()
        reply_message = SimpleMessage()
        # reply with JOINT_POSITION packets
        reply_message.create_empty(JOINT_POSITION)
        reply_message.set_header(JOINT_POSITION, RESPONSE, SUCCESS)

        while not handle_done:
            # Receive the packet
            read_messages(BytesIO(), sock, buff_size, joint_stream_message)

            # print("Got joint stream:")
            # print_str = ''
            # print_str += "[%d] " % joint_stream_message.seq_num
            # for d in joint_stream_message.data:
            #     print_str += "%.2f, " % d
            # print(print_str, end="\t")
            # print("")

            # Check sequence number
            if joint_stream_message.seq_num >= 0:
                # Request new trajectory node
                reply_message.set_seq_num(joint_stream_message.seq_num)
                reply_message.set_reply_code(SUCCESS)

                write_messages(BytesIO(), sock, reply_message)

                # Execute the motion
                self._handle_command(joint_stream_message)

    def _handle_command(self, stream_message):
        """
        Process the command message to update Joint Position with motion
        :param stream_message: command message from ROS
        :type  stream_message: SimpleMessage
        """
        # print ("Set joint position based on command")
        # Convert to degrees
        # set_angle = list(map(degrees, command.data))

        # TODO: how to set robot joint position here
        if self.controller:
            # Call function from main logic controller
            # self.controller.move_robot(set_angle)
            self.controller.move_robot(stream_message)
        else:
            # if not using logic controller, just update dummy (for testing)
            set_angle = list(map(degrees, stream_message.data))
            time.sleep(0.01)
            for i in range(len(joint_pos_dummy)):
                joint_pos_dummy[i] = set_angle[i]
