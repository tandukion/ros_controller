#!/usr/bin/env python
#
# Copyright (c) 2020, Dwindra Sulistyoutomo
#

# defined variable for Robot Status
# RobotMode Message
UNKNOWN = -1    # Unknown or unavailable
MANUAL  = 1     # Teach OR manual mode
AUTO    = 2     # Automatic mode

# TriState Message
# High state
TRUE    = 1
ON      = 1
ENABLED = 1
HIGH    = 1
CLOSED  = 1

# Low state
FALSE   = 0
OFF     = 0
DISABLED= 0
LOW     = 0
OPEN    = 0


class RobotStatus(object):
    """
    Class that handles Robot Status message which contains low-level status information
    """
    def __init__(self):
        # Set default Robot Status
        self.drives_powered = TRUE
        self.e_stopped = FALSE
        self.error_code = 0
        self.in_error = FALSE
        self.in_motion = FALSE
        self.mode = AUTO                # by default, robot running with ROS should be running with AUTO mode
        self.motion_possible = TRUE

    def set_drives_power(self):
        self.drives_powered = TRUE

    def release_drives_power(self):
        self.drives_powered = FALSE

    def set_e_stop(self):
        self.e_stopped = TRUE

    def release_e_stop(self):
        self.e_stopped = FALSE

    def set_error(self, value):
        self.in_error = value

    def set_motion(self, value):
        self.in_motion = value

    def set_error_code(self, e_code):
        self.error_code = e_code

    def set_mode(self, mode):
        self.mode = mode

    def set_motion_possible(self, value=True):
        if value:
            self.motion_possible = TRUE
        else:
            self.motion_possible = FALSE

    def get_robot_status(self):
        """
        Create a list of Robot Status and return it
        :return: list of status
        """
        status = []
        status.append(self.drives_powered)
        status.append(self.e_stopped)
        status.append(self.error_code)
        status.append(self.in_error)
        status.append(self.in_motion)
        status.append(self.mode)
        status.append(self.motion_possible)
        return status
