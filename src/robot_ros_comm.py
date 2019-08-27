#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import socket
import struct
import sys
import threading

from src.msg import serialize_message, deserialize_messages

import select
try:
    from cStringIO import StringIO #Python 2.x
    python3 = 0
except ImportError:
    from io import StringIO, BytesIO #Python 3.x
    python3 = 1


#dummy
joint_angle = [0, 0, 0, 0, 0, 0]

def decode_simple_message_header(header_str):
    """

    :param header_str:
    :return:
    """
    (size, ) = struct.unpack('<I', header_str[0:4])
    size += 4 # add in 4 to include size of size field
    header_len = len(header_str)
    if size > header_len:
        raise Exception("Incomplete header. Expected %s bytes but only have %s"%((size+4), header_len))

    d = {}
    start = 4
    while start < size:
        (field_size, ) = struct.unpack('<I', header_str[start:start+4])
        if field_size == 0:
            raise Exception("Invalid 0-length handshake header field")
        start += field_size + 4
        if start > size:
            raise Exception("Invalid line length in handshake header: %s"%size)
        line = header_str[start-field_size:start]

        #python3 compatibility
        if python3 == 1:
            line = line.decode()

        idx = line.find("=")
        if idx < 0:
            raise Exception("Invalid line in handshake header: [%s]"%line)
        key = line[:idx]
        value = line[idx+1:]
        d[key.strip()] = value
    return d

def read_header(sock, b , buff_size):
    """

    :param sock:
    :param b:
    :param buff_size:
    :return:
    """
    header_str = None
    while not header_str:
        try:
            d = sock.recv(buff_size)
            if not d:
                raise Exception("connection from sender terminated before handshake header received. %s bytes were received. Please check sender for additional details."%b.tell())
            b.write(d)
            btell = b.tell()
            if btell > 4:
                # most likely we will get the full header in the first recv, so
                # not worth tiny optimizations possible here
                bval = b.getvalue()
                print(bval)
                (size,) = struct.unpack('<I', bval[0:4])
                if btell - 4 >= size:
                    header_str = bval

                    # memmove the remnants of the buffer back to the start
                    leftovers = bval[size+4:]
                    b.truncate(len(leftovers))
                    b.seek(0)
                    b.write(leftovers)
                    header_recvd = True
        except (socket.timeout, Exception):
            pass

    # process the header
    return decode_simple_message_header(bval)


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
