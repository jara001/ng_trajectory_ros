#!/usr/bin/env python
# _run.py
"""Run node for trajectory optimization using ng_trajectory.
"""
######################
# Imports & Globals
######################

from autopsy.node import Node
from autopsy.reconfigure import ParameterServer


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
