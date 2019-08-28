#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

# Created to test as a client from Fanuc Roboguide to know the simple message sent from controller

import socket

HOST = ''   # Symbolic name, meaning all available interfaces
host = socket.gethostbyname("192.168.4.61")
PORT = 11002 # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_state = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket created')

s.connect((host, PORT))
print('Socket connected')

while(1):
    s.sendall(b'Hello, world')
    data = s.recv(1024)

    # print(repr(data))
    print(data)

