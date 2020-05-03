#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import os
import json
import yaml
from transitions.extensions.markup import MarkupMachine


class RobotStateMachine(MarkupMachine):
    """
    Class that handle State Machine for the robot.
    Using MarkupMachine to handle the transition with trigger functions
    """
    def __init__(self,
                 model=None,
                 config_dict_file="robot_state_machine_config.yaml"):
        """
        :param model: object with callback handler when entering and exiting state
        :param config_dict_file: JSON file containing state machine dictionary to read
        """
        # Load json config file
        _config_dict_file = os.path.abspath(os.path.join(os.path.dirname(__file__), 'config', config_dict_file))

        # Check file extension
        if config_dict_file.endswith(".json"):
            with open(_config_dict_file, 'rt') as cfg_file:
                self._config = json.load(cfg_file)
        elif config_dict_file.endswith(".yaml"):
            with open(_config_dict_file) as cfg_file:
                self._config = (yaml.load(cfg_file, Loader=yaml.FullLoader))

        # Initialize the state machine
        MarkupMachine.__init__(self,
                               model=model,
                               states=self._config["states"],
                               transitions=self._config["transitions"],
                               initial=self._config["initial"])


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
