#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import struct
import copy

# MSG_TYPE
JOINT_POSITION = 10
JOINT_TRAJ_PT = 11
JOINT_TRAJ = 12
STATUS = 13
JOINT_TRAJ_PT_FULL = 14
JOINT_FEEDBACK = 15

# COMM_TYPE
TOPIC = 1
REQUEST = 2
RESPONSE = 3

# REPLY_CODE
INVALID = 0
SUCCESS = 1
FAILURE = 2

# DATA LIMIT
MAX_JOINT_NUM = 10
ROBOT_STATUS_DATA = 7

# MOTOMAN SIMPLE MESSAGE
MOTOMAN_IO_FALURE = 0
MOTOMAN_IO_SUCCESS = 1

MOTOMAN_MOTION_CTRL = 2001
MOTOMAN_MOTION_REPLY = 2002
ROS_MSG_MOTO_READ_IO_BIT = 2003
ROS_MSG_MOTO_READ_IO_BIT_REPLY = 2004
ROS_MSG_MOTO_WRITE_IO_BIT = 2005
ROS_MSG_MOTO_WRITE_IO_BIT_REPLY = 2006
ROS_MSG_MOTO_READ_IO_GROUP = 2007
ROS_MSG_MOTO_READ_IO_GROUP_REPLY = 2008
ROS_MSG_MOTO_WRITE_IO_GROUP = 2009
ROS_MSG_MOTO_WRITE_IO_GROUP_REPLY = 2010
ROS_MSG_MOTO_IOCTRL_REPLY = 2011

ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX = 2016
ROS_MSG_MOTO_JOINT_FEEDBACK_EX = 2017
ROS_MSG_MOTO_SELECT_TOOL = 2018
ROS_MSG_MOTO_GET_DH_PARAMETERS = 2020

# MOTOMAN CONTROL COMMAND
MOTOMAN_CMD_CHECK_MOTION_READY = 200101
MOTOMAN_CMD_CHECK_QUEUE_CNT = 200102
MOTOMAN_CMD_STOP_MOTION = 200111
MOTOMAN_CMD_START_TRAJ_MODE = 200121
MOTOMAN_CMD_STOP_TRAJ_MODE = 200122


class SimpleMessage(object):
    """
    Class for simple message for ROS-I communication
    Structure: MSG_TYPE, COMM_TYPE, REPLY_CODE, DATA
    """
    def __init__(self):
        self.msg_type = None
        self.comm_type = None
        self.reply_code = None
        self.seq_num = None
        self.data = []

        # Robot Vendor Specific Messages
        self.robot_id = None
        self.ctrl_cmd = None

    def set_header(self, msg_type, comm_type, reply_code):
        """
        Set the header value of the message
        :param msg_type: value for MSG_TYPE
        :param comm_type: value for COMM_TYPE
        :param reply_code: value for REPLY_CODE
        """
        self.msg_type = msg_type
        self.comm_type = comm_type
        self.reply_code = reply_code

    def assign_data(self, data_list):
        """
        Assign the data to the message list
        :param data_list: list of data from the message
        """
        self.data = copy.deepcopy(data_list)

    def set_topic(self):
        self.comm_type = 1

    def set_response(self):
        self.comm_type = 3

    def set_reply_code(self, code):
        self.reply_code = code

    def set_seq_num(self, seq):
        self.seq_num = seq

    def set_robot_id(self, value):
        self.robot_id = value

    def set_ctrl_cmd(self, value):
        self.ctrl_cmd = value

    def create_empty(self, type):
        self.msg_type = 0
        self.comm_type = 0
        self.reply_code = 0
        self.seq_num = 0

        data_range = 0
        if type == JOINT_POSITION:
            data_range = MAX_JOINT_NUM
        elif type == STATUS:
            data_range = ROBOT_STATUS_DATA
        elif type == JOINT_TRAJ_PT:
            data_range = MAX_JOINT_NUM + 2  # for velocity and duration

        if len(self.data) > 0:
            for i in range(data_range):
                self.data[i] = 0
        else:
            for i in range(data_range):
                self.data.append(0)

def serialize_messages(b, seq, msg):
    """
    Serialize the simple message to the buffer

    :param b: buffer to write to
    :type  b: StringIO or BytesIO
    :param seq: current sequence number
    :param msg: message to write
    :type  msg: SimpleMessage
    """
    start = b.tell()    # b.tell() return 0 since it is empty
    b.seek(start + 4)   # reserve 4-bytes for length

    # serialize the message data
    try:
        # msg.serialize(b)

        # Write the MSG_TYPE of the message
        b.write(struct.pack('<I', msg.msg_type))

        # Write the COMM_TYPE  of the message
        b.write(struct.pack('<I', msg.comm_type))

        # Write the REPLY_CODE  of the message
        b.write(struct.pack('<I', msg.reply_code))

        # Write the DATA
        if msg.msg_type == JOINT_POSITION:
            # Sequence Number
            b.write(struct.pack('<I', seq))

            # Joint Position data in rad
            for d in msg.data:
                b.write(struct.pack('<f', d))

            # Fill the remaining unused DOF with zeros
            for i in range(10-len(msg.data)):
                b.write(struct.pack('<f', 0))

        elif msg.msg_type == STATUS:
            # Robot Status
            for d in msg.data:
                b.write(struct.pack('<I', d))

        else:
            # Any message with integer data
            for d in msg.data:
                b.write(struct.pack('<I', d))

    except struct.error as e:
        raise Exception(e)

    # write 4-byte packet length on the first 4 bytes
    end = b.tell()
    size = end - start - 4  # do not include LENGTH packet
    b.seek(start)
    b.write(struct.pack('<I', size))
    b.seek(end)


def deserialize_messages(b, msg, max_size=68):
    """
    Deserialize message and unpack the simple message

    :param b: buffer to read
    :type  b: StringIO or BytesIO
    :param msg: message read from the buffer
    :type  msg: SimpleMessage
    :param max_size: max size of incoming message in bytes
    """
    try:
        start = 0
        pos = start             # starting position
        btell = b.tell()        # last position, b.tell() return full size of incoming bytes
        left = btell - pos

        # check to see if we even have a message
        if left >= 4:
            # set position to start
            b.seek(pos)

            # ------ PREFIX ------
            # Read the LENGTH of the message
            (message_length,) = struct.unpack('<I', b.read(4))
            pos += 4

            if message_length > 1:
                # ------ HEADER ------
                # Read the MSG_TYPE of the message
                (msg_type,) = struct.unpack('<I', b.read(4))
                pos += 4

                # print("btell: ", btell, "LENGTH: ", message_length, "\t MSG_TYPE: ", msg_type)

                # Read the COMM_TYPE  of the message
                (comm_type,) = struct.unpack('<I', b.read(4))
                pos += 4

                # Read the REPLY_CODE  of the message
                (reply_code,) = struct.unpack('<I', b.read(4))
                pos += 4

                # ------ BODY ------
                # Read the DATA on the rest of the message based on message type
                if msg_type == JOINT_POSITION:
                    # Sequence Number
                    (seq_num,) = struct.unpack('<I', b.read(4))
                    pos += 4

                    # Joint Position data in rad
                    data = []
                    for i in range(MAX_JOINT_NUM):
                        (d,) = struct.unpack('<f', b.read(4))
                        data.append(d)
                        pos += 4

                elif msg_type == JOINT_TRAJ_PT:
                    # Sequence Number
                    (seq_num,) = struct.unpack('<I', b.read(4))
                    pos += 4

                    # Joint Position data in rad
                    data = []
                    for i in range(MAX_JOINT_NUM + 2):
                        (d,) = struct.unpack('<f', b.read(4))
                        data.append(d)
                        pos += 4

                elif msg_type == STATUS:
                    data = []
                    for i in range(ROBOT_STATUS_DATA):
                        (d,) = struct.unpack('<I', b.read(4))
                        data.append(d)
                        pos += 4

                # ROBOT Vendor Specific Messages
                elif msg_type == MOTOMAN_MOTION_CTRL:
                    # Robot ID Number
                    (robot_id,) = struct.unpack('<I', b.read(4))
                    pos += 4

                    # Sequence Number
                    (seq_num,) = struct.unpack('<I', b.read(4))
                    pos += 4

                    # Control Command Number
                    (ctrl_cmd,) = struct.unpack('<I', b.read(4))
                    pos += 4

                    # Joint Position data in rad
                    data = []
                    for i in range(MAX_JOINT_NUM):
                        (d,) = struct.unpack('<f', b.read(4))
                        data.append(d)
                        pos += 4

                # default with unsigned int
                else:
                    data = []
                    while pos < btell and pos < max_size:
                        (d,) = struct.unpack('<I', b.read(4))
                        data.append(d)
                        pos += 4

                msg.set_header(msg_type, comm_type, reply_code)
                msg.assign_data(data)

                # Set sequence number on message with seq_num
                if msg_type == JOINT_POSITION or msg_type == JOINT_TRAJ_PT:
                    msg.set_seq_num(seq_num)

                # ROBOT Vendor Specific Messages
                elif msg_type == MOTOMAN_MOTION_CTRL:
                    msg.set_robot_id(robot_id)
                    msg.set_seq_num(seq_num)
                    msg.set_ctrl_cmd(ctrl_cmd)

        # Update the buffer back to its correct position for writing
        if btell == pos:
            #common case: no leftover data, reset the buffer
            b.seek(start)
            b.truncate(start)
        else:
            if pos != start:
                #next packet is stuck in our buffer, copy it to the
                #beginning of our buffer to keep things simple
                b.seek(pos)
                leftovers = b.read(btell-pos)
                b.truncate(start + len(leftovers))
                b.seek(start)
                b.write(leftovers)
            else:
                b.seek(btell)
    except Exception as e:
        print("\nbtell: ", btell, "LENGTH: ", message_length, "MSG_TYPE: ", msg_type)
        raise Exception(e)
