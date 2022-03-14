#!/usr/bin/env python3
# _run.py
"""Run node for trajectory optimization using ng_trajectory.
"""
######################
# Imports & Globals
######################

from autopsy.node import Node
from autopsy.reconfigure import ParameterServer

# For reading the configuration
import os

import ng_trajectory


######################
# RunNode
######################

class RunNode(Node):

    # Node variables
    P = ParameterServer()

    # Parameters
    # Note: Not sure whether this is the right way, but when
    #       we have this option... :)
    P.config_file = {"default": "",
        "description": "Path to the configuration file."
    }


    def __init__(self):
        super(Node, self).__init__("ng_trajectory_ros")


    def load_config(self):
        if os.path.isfile(self.P.config_file.value):
            if os.access(self.P.config_file.value, os.R_OK):
                ng_trajectory.configurationLoad(self.P.config_file.value)
            else:
                raise IOError("File '%s' is not readable." % self.P.config_file.value)
        else:
            raise IOError("File '%s' does not exist." % self.P.config_file.value)
