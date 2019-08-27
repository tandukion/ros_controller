#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import struct
import genpy


def args_kwds_to_message(data_class, args, kwds):
    """
    Given a data class, take in the args and kwds of a function call and return the appropriate
    data_class instance.

    If kwds are specified, a new data_class instance will be created with keyword-style init.

    If there is only a single arg and it is of the correct type, it
    will be returned. AnyMsg is considered to match all data_class
    types.

    Otherwise, args will be used as args for a new message instance.

    @param data_class: Message class that will be used to instantiate new instances, if necessary.
    @type  data_class: Message class
    @param args: function args
    @type  args: sequence
    @param kwds: function kwds
    @type  kwds: dict
    @raise TypeError: if args and kwds are both specified
    """
    if args and kwds:
        raise TypeError("publish() can be called with arguments or keywords, but not both.")
    elif kwds:
        return data_class(**kwds)
    else:
        if len(args) == 1:
            arg = args[0]
            # #2584: have to compare on md5sum as isinstance check can fail in dyngen case
            if hasattr(arg, '_md5sum') and (arg._md5sum == data_class._md5sum or isinstance(arg, AnyMsg)):
                return arg
            # If the argument is a message, make sure that it matches
            # the type of the first field. This means that the
            # data_class has fields and that the type matches.  This
            # branch isn't necessary but provides more useful
            # information to users
            elif isinstance(arg, genpy.Message):
                if len(data_class._slot_types) == 0:
                    raise TypeError("expected [] but got [%s]"%arg._type)
                elif arg._type != data_class._slot_types[0]:
                    raise TypeError("expected [%s] but got [%s]"%(data_class._slot_types[0], arg._type))
            return data_class(*args)
        else:
            return data_class(*args)


def serialize_message(b, seq, msg):
    """
    Serialize the message to the buffer
    @param b: buffer to write to. WARNING: buffer will be reset after call
    @type  b: StringIO
    @param msg: message to write
    @type  msg: Message
    @param seq: current sequence number (for headers)
    @type  seq: int: current sequence number (for headers)
    @raise ROSSerializationException: if unable to serialize
    message. This is usually due to a type error with one of the
    fields.
    """
    start = b.tell()
    b.seek(start + 4)  # reserve 4-bytes for length

    # update Header object in top-level message
    if getattr(msg.__class__, "_has_header", False):
        header = msg.header
        header.seq = seq
        # default value for frame_id is '0'
        if header.frame_id is None:
            header.frame_id = "0"

    # serialize the message data
    try:
        msg.serialize(b)

    except struct.error as e:
        raise Exception(e)

    # write 4-byte packet length
    # -4 don't include size of length header
    end = b.tell()
    size = end - 4 - start
    b.seek(start)
    b.write(struct.pack('<I', size))
    b.seek(end)


def deserialize_messages(b, msg_queue, data_class, queue_size=None, max_msgs=None, start=0):
    """
    Read all messages off the buffer

    @param b: buffer to read data from
    @type  b: StringIO
    @param msg_queue: queue to append deserialized data to
    @type  msg_queue: list
    @param data_class: message deserialization class
    @type  data_class: Message class
    @param queue_size: message queue size. all but the last
    queue_size messages are discarded if this parameter is specified.
    @type  queue_size: int
    @param start: starting position to read in b
    @type  start: int
    @param max_msgs: maximum number of messages to deserialize or None
    @type  max_msgs: int
    @raise genpy.DeserializationError: if an error/exception occurs during deserialization
    """
    try:
        pos = start
        btell = b.tell()
        left = btell - pos

        # check to see if we even have a message
        if left < 4:
            return

        # read in each message from the buffer as a string. each
        # message is preceded by a 4-byte integer length. the
        # serialized messages are appended to buff.
        b.seek(pos)
        buffs = []
        # size of message
        size = -1
        while (size < 0 and left >= 4) or (size > -1 and left >= size):
            # - read in the packet length
            #   NOTE: size is not inclusive of itself.
            if size < 0 and left >= 4:
                (size,) = struct.unpack('<I', b.read(4))
                left -= 4
            # - deserialize the complete buffer
            if size > -1 and left >= size:
                buffs.append(b.read(size))
                pos += size + 4
                left = btell - pos
                size = -1
                if max_msgs and len(buffs) >= max_msgs:
                    break

        # Before we deserialize, prune our buffers baed on the
        # queue_size rules.
        if queue_size is not None:
            buffs = buffs[-queue_size:]

        # Deserialize the messages into msg_queue, then trim the
        # msg_queue as specified.
        for q in buffs:
            data = data_class()
            msg_queue.append(data.deserialize(q))
        if queue_size is not None:
            del msg_queue[:-queue_size]

        # update buffer b to its correct write position.
        if btell == pos:
            # common case: no leftover data, reset the buffer
            b.seek(start)
            b.truncate(start)
        else:
            if pos != start:
                # next packet is stuck in our buffer, copy it to the
                # beginning of our buffer to keep things simple
                b.seek(pos)
                leftovers = b.read(btell - pos)
                b.truncate(start + len(leftovers))
                b.seek(start)
                b.write(leftovers)
            else:
                b.seek(btell)
    except Exception as e:
        raise Exception(e)

