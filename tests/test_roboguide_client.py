#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

# Created to test as a client from Fanuc Roboguide to know the simple message sent from controller

import socket
from src.robot_ros_comm import *

from io import StringIO, BytesIO #Python 3.x

HOST = ''   # Symbolic name, meaning all available interfaces
host = socket.gethostbyname("192.168.4.61")
PORT = 11002 # Arbitrary non-privileged port

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_state = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket created')

sock.connect((host, PORT))
print('Socket connected')

buff_size = 4096

while(1):
    # d = sock.recv(buff_size)
    # print(repr(d))
    msg_queue = []

    read_messages(BytesIO(), sock, buff_size)

    # deserialize_messages(b, msg_queue, data_class=float, queue_size=65536)

    # print(msg_queue)

