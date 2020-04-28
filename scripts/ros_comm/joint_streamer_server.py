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
        socket_connected = True
        buff_size = 4096

        # create incoming packet and out coming packet
        joint_stream_message = SimpleMessage()
        reply_message = SimpleMessage()
        # reply with JOINT_POSITION packets
        reply_message.create_empty(JOINT_POSITION)
        reply_message.set_header(JOINT_POSITION, RESPONSE, SUCCESS)

        while socket_connected:
            # Receive the packet
            (socket_connected, message_ready) = read_messages(BytesIO(), sock, buff_size, joint_stream_message)
            if not socket_connected:
                break

            if message_ready:
                # print("Got joint stream:")
                # print_str = ''
                # print_str += "[%d " % joint_stream_message.msg_type
                # print_str += "%d %d] " % (joint_stream_message.comm_type, joint_stream_message.reply_code)
                # for d in joint_stream_message.data:
                #     print_str += "%.2f, " % d
                # print(print_str, end="\t")
                # print("")

                # Default General Robot
                if joint_stream_message.msg_type == JOINT_POSITION or joint_stream_message.msg_type == JOINT_TRAJ_PT:
                    # Check sequence number
                    if joint_stream_message.seq_num >= 0:
                        # Request new trajectory node
                        reply_message.set_seq_num(joint_stream_message.seq_num)
                        reply_message.set_reply_code(SUCCESS)

                        write_messages(BytesIO(), sock, reply_message)

                        # Execute the motion
                        self._handle_command(joint_stream_message)

                # Yaskawa Motoman
                elif joint_stream_message.msg_type == MOTOMAN_MOTION_CTRL:
                    # Check the command
                    command = joint_stream_message.ctrl_cmd
                    if command == MOTOMAN_CMD_STOP_MOTION or command == MOTOMAN_CMD_STOP_TRAJ_MODE:
                        # Disconnect the connection on stopping motion
                        print("Stopping joint trajectory streaming")
                        socket_connected = False

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
