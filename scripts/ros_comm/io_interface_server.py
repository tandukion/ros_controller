#!/usr/bin/env python
#
# Copyright (c) 2020, Dwindra Sulistyoutomo
#
# Currently only support Python 3.x

import time

from .simple_message import *
from .ros_comm import read_messages, write_messages, BytesIO
from .message_server import MessageServer


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

    def set_io_port(self, address, value):
        self.io_ports[str(address)] = value

    def get_io_port(self, address):
        return self.io_ports[str(address)]

    def io_interface_handler(self, sock):
        socket_connected = True
        buff_size = 4096

        # create incoming packet and out coming packet
        io_message = SimpleMessage()

        while socket_connected:
            # Receive the packet
            (socket_connected, message_ready) = read_messages(BytesIO(), sock, buff_size, io_message)
            if not socket_connected:
                break

            # print("Got io stream:")
            # print_str = ''
            # print_str += "[%d] " % io_stream_message.msg_type
            # print_str += "[%d] [%d] " % (io_stream_message.comm_type, io_stream_message.reply_code)
            # for d in io_stream_message.data:
            #     print_str += "%d, " % d
            # print(print_str, end="\t")
            # print("")

            # FIXME: The command is configured according to Yaskawa IO configuration for now
            # Reply for Read IO
            if io_message.msg_type == ROS_MSG_MOTO_READ_IO_BIT or io_message.msg_type == ROS_MSG_MOTO_READ_IO_GROUP:
                # Process the message
                address = io_message.data[0]

                # Read the IO port
                # Single read
                if io_message.msg_type == ROS_MSG_MOTO_READ_IO_BIT:
                    value = self.get_io_port(address)
                # Group read
                else:
                    value = 0
                    for i in range(8):
                        read_bit = self.get_io_port(address+i)

                        # add the bit to the return value
                        value += read_bit << i

                # Create the reply message
                reply_message = SimpleMessage()
                reply_message.set_header(io_message.msg_type+1, RESPONSE, SUCCESS)

                # Add value
                reply_message.data.append(value)
                # Add result_code
                reply_message.data.append(MOTOMAN_IO_SUCCESS)

                # Serialize + sending message
                write_messages(BytesIO(), sock, reply_message)

            # Reply for Write IO
            if io_message.msg_type == ROS_MSG_MOTO_WRITE_IO_BIT or io_message.msg_type == ROS_MSG_MOTO_WRITE_IO_GROUP:
                # Process the message
                address = io_message.data[0]
                value = io_message.data[1]

                # Assign the IO port
                # Single write
                if io_message.msg_type == ROS_MSG_MOTO_WRITE_IO_BIT:
                    self.set_io_port(address, value)
                # Group write
                else:
                    base_address = address * 10

                    # Check based on I/O type on first number
                    if base_address >= 10000:
                        # Loop on 8 bit of IO group
                        for i in range(8):
                            # Set the IO for the specific IO port
                            self.set_io_port(base_address+i, (value & (1 << i)))

                # Create the reply message
                reply_message = SimpleMessage()
                reply_message.set_header(io_message.msg_type+1, RESPONSE, SUCCESS)

                # Add result_code
                reply_message.data.append(MOTOMAN_IO_SUCCESS)

                # Serialize + sending message
                write_messages(BytesIO(), sock, reply_message)
