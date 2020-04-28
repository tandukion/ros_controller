#!/usr/bin/env python
#
# Copyright (c) 2020, Dwindra Sulistyoutomo
#
# Currently only support Python 3.x

import time

from scripts.ros_comm.simple_message import *
from scripts.ros_comm.ros_comm import read_messages, write_messages, BytesIO
from scripts.ros_comm.message_server import MessageServer


class IoInterfaceServer (MessageServer):
    """
    This class acts as server for processing Joint Stream motion control from ROS
    """
    def __init__(self, controller=None, port=11003, loop_rate=40):
        """
        :param port: TCP port for IO Interface. By default = 11003 defined from ROS-Industrial
        :param loop_rate: frequency for sending processing packet
        """
        super(IoInterfaceServer, self).__init__(port)
        self.controller = controller
        self.port = port
        self.inbound_handler = self.io_interface_handler
        self.loop_rate = loop_rate
        self.handle_done = False

        # IO port
        self.io_ports = {}

    def assign_io_number(self, number, value):
        self.io_ports[str(number)] = value

    def io_interface_handler(self, sock):
        socket_connected = True
        buff_size = 4096

        # create incoming packet and out coming packet
        io_message = SimpleMessage()

        while socket_connected:
            # Receive the packet
            socket_connected = read_messages(BytesIO(), sock, buff_size, io_message)
            if not socket_connected:
                break

            # Read the data
            io_command = io_message.data

            # print("Got io stream:")
            # print_str = ''
            # print_str += "[%d] " % io_stream_message.msg_type
            # print_str += "[%d] [%d] " % (io_stream_message.comm_type, io_stream_message.reply_code)
            # for d in io_stream_message.data:
            #     print_str += "%d, " % d
            # print(print_str, end="\t")
            # print("")

            # Process the command
            # FIXME: The command is configured according to Yaskawa IO configuration for now
            # Reply for Read IO
            if io_message.msg_type == ROS_MSG_MOTO_READ_IO_BIT or io_message.msg_type == ROS_MSG_MOTO_READ_IO_GROUP:
                # Create the reply message
                reply_message = SimpleMessage()
                reply_message.set_header(io_message.msg_type+1, RESPONSE, SUCCESS)

                # Add value
                # FIXME: change this dummy value
                reply_message.data.append(0)
                # Add result_code
                reply_message.data.append(MOTOMAN_IO_SUCCESS)

                # Serialize + sending message
                write_messages(BytesIO(), sock, reply_message)

            # Reply for Write IO
            if io_message.msg_type == ROS_MSG_MOTO_WRITE_IO_BIT or io_message.msg_type == ROS_MSG_MOTO_WRITE_IO_GROUP:
                # Create the reply message
                reply_message = SimpleMessage()
                reply_message.set_header(io_message.msg_type+1, RESPONSE, SUCCESS)

                # Add result_code
                reply_message.data.append(MOTOMAN_IO_SUCCESS)

                # Serialize + sending message
                write_messages(BytesIO(), sock, reply_message)
