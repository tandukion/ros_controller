#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#
# Currently only support Python 3.x

import socket
import threading
from .simple_message import *

try:
    from cStringIO import StringIO  # Python 2.x
    python3 = 0
except ImportError:
    from io import StringIO, BytesIO  # Python 3.x
    python3 = 1

# dummy
joint_pos_dummy = [0, 0, 0, 0, 0, 0]


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
    socket_status = True
    message_ready = False
    sock.settimeout(5)
    while not message_ready:
        try:
            d = sock.recv(buff_size)
            b.write(d)

            btell = b.tell()
            if btell > 4:
                message_ready = True

        except socket.timeout:
            break
        except socket.error as e:
            # sockname = sock.getsockname()
            # print("[%d] Socket error. " % sockname[1], e)
            socket_status = False
            break

    if message_ready:
        # Deserialize the message
        deserialize_messages(b, msg)

    # reset if everything was done
    if b.tell() == 1:
        b.seek(0)

    return socket_status, message_ready


def write_messages(b, sock, msg, seq=0):
    """
    Write the simple message to socket connection.

    :param b: buffer
    :type  b: StringIO or BytesIO
    :param sock: socket to read from
    :param sock: socket.socket
    :param msg: message to read
    :type  msg: SimpleMessage
    :param seq: sequence number of the message
    :type  seq: int
    """
    # Serialize the message
    serialize_messages(b, seq, msg)

    write_success = True
    # Send to socket
    try:
        sock.sendall(b.getvalue())
    except socket.error as e:
        # raise Exception(e)
        # sockname = sock.getsockname()
        # print("[%d] Socket error. " % sockname[1], e)
        write_success = False

    # clearing buffer
    b.truncate(0)
    return write_success


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
        self.is_connected = False
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

    def close_socket(self):
        """ Close the client socket connection """
        self.client_sock.close()
        self.is_shutdown = True

    def restart_server(self):
        """ Restart the thread """
        self.start()

    def run(self):
        """
        Main TCP receive loop. Use start() to do this automatically.
        """
        self.is_shutdown = False
        if not self.server_sock:
            raise Exception("%s did not connect" % self.__class__.__name__)
        while not self.is_shutdown:
            try:
                print('[%d] Waiting for connection' % self.port)
                (self.client_sock, client_addr) = self.server_sock.accept()
            except socket.timeout:
                print('socket timeout')
                continue
            except IOError as e:
                print('IO error')
                (errno, msg) = e.args
                if errno == 4:  # interrupted system call
                    continue
                break

            # Set flag for started connection
            self.is_connected = True
            print('[%d]' % self.port + ' Connected with ' + client_addr[0] + ':' + str(client_addr[1]))

            # leave threading decisions up to inbound_handler
            # print('Running connection handler')
            self.inbound_handler(self.client_sock)

            # Set flag for disconnected client after returning from inbound_handler
            # Returning means socket error and client is disconnected
            # print('[%d] Disconnected' % self.port)
            self.is_connected = False
            self.is_shutdown = True
