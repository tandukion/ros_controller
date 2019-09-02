#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#

import os
import json
from transitions.extensions.markup import MarkupMachine


class RobotStateMachine(MarkupMachine):
    def __init__(self,
                 model=None,
                 config_dict_file="robot_state_machine_config.json"):

        # Load json config file
        _config_dict_file = os.path.abspath(os.path.join(os.path.dirname(__file__), config_dict_file))
        with open(_config_dict_file, 'rt') as cfg_file:
            self._config = json.load(cfg_file)

        # Initialize the state machine
        MarkupMachine.__init__(self,
                               model=model,
                               states=self._config["states"],
                               transitions=self._config["transitions"],
                               initial=self._config["initial"])
