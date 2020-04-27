#!/usr/bin/env python
#
# Copyright (c) 2020, Dwindra Sulistyoutomo
#
# Currently only support Python 3.x

import threading
from scripts.ros_comm.ros_comm import ServerSocket


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