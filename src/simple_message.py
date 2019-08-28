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


class SimpleMessage(object):
    """
    Class for simple message for ROS-I communication
    Structure: MSG_TYPE, COMM_TYPE, REPLY_CODE, DATA
    """
    def __init__(self):
        self.msg_type = None
        self.comm_type = None
        self.reply_code = None
        self.data = []

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


def serialize_message(b, seq, msg):
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
                b.write(struct.pack('<I', d))


    except struct.error as e:
        raise Exception(e)

    # write 4-byte packet length on the first 4 bytes
    end = b.tell()
    size = end - start - 4  # do not include LENGTH packet
    b.seek(start)
    b.write(struct.pack('<I', size))
    b.seek(end)


def deserialize_messages(b, msg):
    """
    Deserialize message and unpack the simple message

    :param b: buffer to read
    :type  b: StringIO or BytesIO
    :param msg: message read from the buffer
    :type  msg: SimpleMessage
    """
    try:
        start = 0
        pos = start             # starting position
        btell = b.tell()        # last position, b.tell() return full size of incoming bytes
        left = btell - pos

        # check to see if we even have a message
        if left < 4:
            return

        # set position to start
        b.seek(pos)

        # Read the LENGTH of the message
        (message_length,) = struct.unpack('<I', b.read(4))
        left -= 4

        # Read the MSG_TYPE of the message
        (msg_type,) = struct.unpack('<I', b.read(4))
        left -= 4

        # Read the COMM_TYPE  of the message
        (comm_type,) = struct.unpack('<I', b.read(4))
        left -= 4

        # Read the REPLY_CODE  of the message
        (reply_code,) = struct.unpack('<I', b.read(4))
        left -= 4

        # Read the DATA on the rest of the message based on message type
        if msg_type == JOINT_POSITION:
            # Sequence Number
            (seq_num,) = struct.unpack('<I', b.read(4))
            left -= 4

            # Joint Position data in rad
            data = []
            while left > 0:
                (d,) = struct.unpack('<f', b.read(4))
                data.append(d)
                left -= 4

        elif msg_type == JOINT_TRAJ_PT:
            # Sequence Number
            (seq_num,) = struct.unpack('<I', b.read(4))
            left -= 4

            # Joint Position data in rad
            data = []
            while left > 0:
                (d,) = struct.unpack('<f', b.read(4))
                data.append(d)
                left -= 4

        elif msg_type == STATUS:
            data = []
            while left > 0:
                (d,) = struct.unpack('<I', b.read(4))
                data.append(d)
                left -= 4

        # default with unsigned int
        else:
            data = []
            while left > 0:
                (d,) = struct.unpack('<I', b.read(4))
                data.append(d)
                left -= 4

        msg.set_header(msg_type, comm_type, reply_code)
        msg.assign_data(data)

        # print("LENGTH: %s   MSG_TYPE: %s    COMM_TYPE: %s   REPLY_CODE: %s" % (message_length, msg_type, comm_type, reply_code))
        # print(data)
        # test print
        if msg_type == JOINT_POSITION:
            if comm_type == 1:
                str = ''
                for d in data:
                    str += "%.2f, " %d
                print("Current Joint State:", str)

        # Update the buffer back to its correct position for writing
        b.seek(start)
        b.truncate(start)
    except Exception as e:
        raise Exception(e)
