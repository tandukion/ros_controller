#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#
# Currently only support Python 3.x

import socket
import threading
import time
import sched
from math import degrees, radians
from src.simple_message import *

try:
    from cStringIO import StringIO  # Python 2.x
    python3 = 0
except ImportError:
    from io import StringIO, BytesIO  # Python 3.x
    python3 = 1

# dummy
joint_pos = [0, 0, 0, 0, 0, 0]


"""
Basic functions for communication.

These functions handle the reading and writing message in Simple Message
"""
def read_messages(b, sock, buff_size, msg):
    """
    Read the simple message from socket connection.
    Note that the connection should be already established.

    :param b: buffer
    :type  b: StringIO or BytesIO
    :param sock: socket to read from
    :param buff_size: buffer size
    :param msg: message to read
    :type  msg: SimpleMessage
    """
    # Read from socket
    message_read = False
    while not message_read:
        try:
            d = sock.recv(buff_size)
            b.write(d)

            btell = b.tell()
            if btell > 4:
                message_read = True

        except (socket.timeout, Exception):
            pass

    # Deserialize the message
    deserialize_messages(b, msg)

    # reset if everything was done
    if b.tell() == 1:
        b.seek(0)


def write_messages(b, sock, msg, seq=0):
    """
    Write the simple message to socket connection.

    :param b: buffer
    :type  b: StringIO or BytesIO
    :param sock: socket to read from
    :param msg: message to read
    :type  msg: SimpleMessage
    :param seq: sequence number of the message
    """
    # Serialize the message
    serialize_messages(b, seq, msg)

    # Send to socket
    try:
        sock.sendall(b.getvalue())
    except socket.error as e:
        raise Exception(e)

    # clearing buffer
    b.truncate(0)


"""
Client classes
    - ClientSocket

Classes here are used when the program acts as a Client of the socket communication
Mainly used only to check and verify the communication and testing functions
"""
class ClientSocket(object):
    """
    Simple Client socket class.
    This class is used only for testing the communication and verifying the simple message.
    Mainly to test read_messages and deserialize_message.
    """
    def __init__(self,
                 ip_addr,
                 port):
        """
        :param ip_addr: IP address of server
        :type  ip_addr: str
        :param port: TCP port
        """
        self.port = port
        self.host = socket.gethostbyname(ip_addr)
        self.is_shutdown = False

        # Creating client socket
        self.client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.message = SimpleMessage()

    def start(self):
        """
        Run the loop in separated thread
        """
        t = threading.Thread(target=self.run, args=())
        t.setDaemon(True)
        t.start()

    def run(self):
        """
        Main TCP receive loop. Use start() to do this automatically.
        """
        self.is_shutdown = False
        while not self.is_shutdown:
            try:
                # Connect to Server
                self.client_sock.connect((self.host, self.port))
            except socket.timeout:
                print('socket timeout')
                continue

            try:
                handle_done = False
                buff_size = 4096

                while not handle_done:
                    read_messages(BytesIO(), self.client_sock, buff_size, self.message)

            except socket.error as e:
                if not self.is_shutdown:
                    print("Failed to handle inbound connection due to socket error: %s" % e)


"""
Server classes
    - ServerSocket          : socket connection as a Server 
    - MessageServer         : base class for handling the message as server
    - RobotStateServer      : server for sending robot state (joint position + robot status)
    - JointStreamerServer   : server for processing joint stream

Classes here are used when the program acts as a Server of the socket communication.
These classes are the main classes, if the program runs as robot controller.
"""
class ServerSocket(object):
    """
    Simple server class that accepts inbound TCP/IP connections.
    This class handles the creation of server socket until accepting the connection.
    After the connection established, it jumps to the handler defined by the caller of this class.
    """
    def __init__(self,
                 inbound_handler,
                 port=0):
        """
        :param inbound_handler: handler method after connection with client is established
        :param port: TCP port
        """
        self.port = port
        self.addr = ''
        self.is_shutdown = False
        self.inbound_handler = inbound_handler

        # Creating server socket
        try:
            self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # print('Socket created for Port %s' % self.port)
            self.server_sock.bind((self.addr, self.port))
            # print('Socket bind complete')
            self.server_sock.listen(5)
            # print('Socket now listening')
        except socket.error as e:
            print("Bind failed. Error : %s " % e)

    def start(self):
        """
        Run the loop in separated thread
        """
        t = threading.Thread(target=self.run, args=())
        t.setDaemon(True)
        t.start()

    def run(self):
        """
        Main TCP receive loop. Use start() to do this automatically.
        """
        self.is_shutdown = False
        if not self.server_sock:
            raise Exception("%s did not connect" % self.__class__.__name__)
        while not self.is_shutdown:
            try:
                print('Waiting for connection')
                (client_sock, client_addr) = self.server_sock.accept()
            except socket.timeout:
                print('socket timeout')
                continue
            except IOError as e:
                print('IO error')
                (errno, msg) = e.args
                if errno == 4:  # interrupted system call
                    continue
                raise

            print('Connected with ' + client_addr[0] + ':' + str(client_addr[1]))
            try:
                # leave threading decisions up to inbound_handler
                print('Running connection handler')
                self.inbound_handler(client_sock)
            except socket.error as e:
                if not self.is_shutdown:
                    print("Failed to handle inbound connection due to socket error: %s" % e)


class MessageServer(object):
    """
    Base class for handling the message as server.
    Specific message should inherit from this class.
    """
    def __init__(self, inbound_handler, port=0):
        """
        :param inbound_handler: handler method after connection with client is established
        :param port: TCP port
        """
        self.inbound_handler = inbound_handler
        self.port = port
        self.server = None
        self.lock = threading.Lock()

    def start_server(self):
        """
        Start the socket server and passing the inbound handler
        """
        if self.server:
            return
        with self.lock:
            try:
                if not self.server:
                    # Set the handler as the process after connection is established
                    self.server = ServerSocket(self.inbound_handler, self.port)
                    self.server.start()
            except Exception as e:
                self.server = None
                return 0, "unable to establish socket server: %s" % e, []


class RobotStateServer (MessageServer):
    """
    This class acts as server for sending robot state (joint position + robot status)
    """
    def __init__(self, port=11002, loop_rate=40, stat_loop=10):
        """
        :param port: TCP port for Robot State. By default = 11002 defined from ROS-Industrial
        :param loop_rate: frequency for sending robot state
        :param stat_loop: Robot Status frequency based on Joint Position messages
        """
        super(RobotStateServer, self).__init__(port)
        self.port = port
        self.inbound_handler = self.publish_handler
        self.loop_rate = loop_rate
        self.seq = 0
        self.handle_done = False
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
        self.handle_done = False
        while not self.handle_done:
            self.scheduler.enter(self.pub_period, 1, self.joint_position_publisher(sock))
            self.scheduler.enter(self.pub_period * self.stat_loop, 1, self.robot_state_publisher(sock))

    def joint_position_publisher(self, sock):
        """
        Handler to publish Joint Position message
        """
        # create Joint Position message  #TODO: create how get the current joint position here
        # convert to radians
        joint_pos_rad = list(map(radians, joint_pos))
        self.joint_pos_message.assign_data(joint_pos_rad)
        # set seq_num to 0 for Joint Position Topic
        self.seq = 0

        # Serialize + sending message
        write_messages(BytesIO(), sock, self.joint_pos_message, self.seq)

    def robot_state_publisher(self, sock):
        """
        Handler to publish Robot State message
        """
        # create Robot Status message  #TODO: create how get the current robot status here
        dummy_status = [1, 0, 0, 0, 0, 1, 0]
        robot_status = dummy_status
        self.robot_status_message.assign_data(robot_status)
        # Serialize + sending message, no need to use seq for Robot Status
        write_messages(BytesIO(), sock, self.robot_status_message)


class JointStreamerServer (MessageServer):
    """
    This class acts as server for processing Joint Stream motion control from ROS
    """
    def __init__(self, port=11000, loop_rate=42):
        """
        :param port: TCP port for Joint Stream. By default = 11000 defined from ROS-Industrial
        :param loop_rate: frequency for sending processing packet
        """
        super(JointStreamerServer, self).__init__(port)
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

        # create incoming packet and outcoming packet
        joint_stream_message = SimpleMessage()
        reply_message = SimpleMessage()
        # reply with JOINT_POSITION packets
        reply_message.create_empty(JOINT_POSITION)
        reply_message.set_header(JOINT_POSITION, RESPONSE, SUCCESS)

        while not handle_done:
            # Recevie the packet
            read_messages(BytesIO(), sock, buff_size, joint_stream_message)

            # print("Got joint stream:")
            print_str = ''
            print_str += "[%d] " % joint_stream_message.seq_num
            for d in joint_stream_message.data:
                print_str += "%.2f, " % d
            print(print_str, end="\t")
            print("")

            # Check sequence number
            if joint_stream_message.seq_num >= 0:
                # Request new trajectory node
                reply_message.set_seq_num(joint_stream_message.seq_num)
                reply_message.set_reply_code(SUCCESS)

                write_messages(BytesIO(), sock, reply_message)

                # Execute the motion
                self._handle_command(joint_stream_message)

    def _handle_command(self, command):
        """
        Process the command message to update Joint Position with motion
        :param command: command message from ROS
        :type  command: SimpleMessage
        """
        # print ("Set joint position based on command")
        # Convert to degrees
        set_angle = list(map(degrees, command.data))

        # TODO: how to set robot joint position here
        for i in range(len(joint_pos)):
            joint_pos[i] = set_angle[i]
