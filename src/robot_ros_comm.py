#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import socket
import struct
import sys
import threading
import copy

from src.simple_message import *

import select
try:
    from cStringIO import StringIO #Python 2.x
    python3 = 0
except ImportError:
    from io import StringIO, BytesIO #Python 3.x
    python3 = 1


#dummy
joint_angle = [0, 0, 0, 0, 0, 0]


def read_messages(b, sock, buff_size, msg):
    """
    Read the message from socket connection.
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
        # t.setDaemon(True)
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





























class ServerSocket(object):
    """
    Simple server class that accept inbound TCP/IP connections.
    This class handle the creation of server socket until accepting the connection.
    After the connection established, it jumps to the handler defined by the caller of this class.
    """
    def __init__(self,
                 inbound_handler,
                 port=0):
        self.port = port
        self.addr = ''
        self.is_shutdown = False
        self.inbound_handler = inbound_handler

        # Creating server socket
        try:
            self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            print('Socket created for Port %s' % self.port)
            self.server_sock.bind((self.addr, self.port))
            print('Socket bind complete')
            self.server_sock.listen(5)
            print('Socket now listening')
        except socket.error as e:
            print("Bind failed. Error Code : %s  Message %s" %(str(e[0]), e[1]))

    def start(self):
        """
        Run the loop in separated thread
        """
        t = threading.Thread(target=self.run, args=())
        # t.setDaemon(True)
        t.start()

    def run(self):
        """
        Main TCP receive loop. Use start() to do this automatically.
        """
        self.is_shutdown = False
        if not self.server_sock:
            raise Exception("%s did not connect"%self.__class__.__name__)
        while not self.is_shutdown:
            try:
                print('Trying to accept connection')
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
            if self.is_shutdown:
                break

            try:
                # leave threading decisions up to inbound_handler
                self.inbound_handler(client_sock, client_addr)
            except socket.error as e:
                if not self.is_shutdown:
                    print("Failed to handle inbound connection due to socket error: %s" % e)


class MessageServer(object):
    """
    Base Server
    """
    def __init__(self, inbound_handler, port=11000):
        """

        :param port:
        """
        self.inbound_handler = inbound_handler
        self.port = port
        self.server = None
        self.lock = threading.Lock()

        if python3 == 0: # Python 2.x
            self.read_buff = StringIO()
            self.write_buff = StringIO()
        else: # Python 3.x
            self.read_buff = BytesIO()
            self.write_buff = BytesIO()

        #STATS
        self.stat_bytes = 0
        # Number of messages that have passed through this transport
        self.stat_num_msg = 0

    def start_server(self):
        """
        Start the socket server and passing the inbound handler
        """
        if self.server:
            return
        with self.lock:
            try:
                if not self.server:
                    self.server = ServerSocket(self.inbound_handler, self.port)
                    self.server.start()
            except Exception as e:
                self.server = None
                return 0, "unable to establish socket server: %s"%e, []


class JointStreamerServer (MessageServer):
    """

    """
    def __init__(self, port=11000):
        super(JointStreamerServer,self).__init__(port)
        self.port = port
        self.inbound_handler = self.joint_streamer_handler

    def joint_streamer_handler(self, sock, client_addr):
        handle_done = False
        buff_size = 4096

        # Getting the header message
        print('Getting header')
        if python3 == 0:
            header = read_header(sock, StringIO(), buff_size)
        else:
            header = read_header(sock, BytesIO(), buff_size)

        # Receving the command
        while not handle_done:
            try:
                b = self.read_buff
                msg_queue = []

                try:
                    sock.setblocking(1)
                    while not msg_queue:
                        if b.tell() >= 4:
                            # Read message + deserialize message
                            deserialize_messages(b, msg_queue, data_class=int, queue_size=65536)

                        if not msg_queue:
                            d = sock.recv(buff_size)
                            if d:
                                b.write(d)
                            self.stat_bytes += len(d)

                    self.stat_num_msg += len(msg_queue) #STATS
                    # set the _connection_header field
                    for m in msg_queue:
                        m._connection_header = header
                except Exception as e:
                    raise Exception(e)

                commands = msg_queue

                for command in commands:
                    self._handle_command(command)
                handle_done = True

            except:
                continue

    def _handle_command(self, command):
        print("Joint command: ")
        print(command)


class RobotStateServer (MessageServer):
    """

    """
    def __init__(self, port=11002):
        super(RobotStateServer,self).__init__(port)
        self.port = port
        self.inbound_handler = self.robot_state_handler
        self.seq = 0

    def robot_state_handler(self, sock, client_addr):
        # create the message
        self.seq += 1
        messages = joint_angle

        # Serialize + sending message
        # serialize_message(self.write_buff, self.seq, message)

        encoded_message = []
        for m in messages:
            encoded_message.append(str(m))

        s = b''.join([struct.pack('<I', len(m)) + m for m in encoded_message])
        self.write_buff.write(s)

        # Write data
        try:
            sock.sendall(self.write_buff)
        except socket.error as se:
            pass


class RobotROSCommunication(object):
    """

    """
    def __init__(self):
        _joint_state_thread = threading.Thread(target=self.joint_state_thread)
        _joint_state_thread.setDaemon(True)
        _joint_state_thread.start()


    # def joint_state_thread(self):
    #     continue
