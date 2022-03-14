#!/usr/bin/env python
# _run.py
"""Run node for trajectory optimization using ng_trajectory.
"""
######################
# Imports & Globals
######################

from autopsy.node import Node


######################
# RunNode
######################

class RunNode(Node):

    def __init__(self):
        super(Node, self).__init__("ng_trajectory_ros")
