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

import numpy
import ng_trajectory


# Messages
from nav_msgs.msg import Path
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


######################
# RunNode
######################

class RunNode(Node):

    # Node variables
    start_points = None
    valid_points = None
    P = None


    def __init__(self):
        super(Node, self).__init__("ng_trajectory_ros")

        self.sub_path = self.Subscriber("path", Path, self.callback_path)
        self.sub_validarea = self.Subscriber("validarea", GridCells, self.callback_validarea)

        self.pub_path = self.Publisher("npath", Path, queue_size = 1, latch = True)


        # Parameters
        self.P = ParameterServer()

        # Note: Not sure whether this is the right way, but when
        #       we have this option... :)
        self.P.config_file = {"default": "",
            "description": "Path to the configuration file.",
            "callback": self.reconf_config_file
        }


    # Callbacks
    def callback_path(self, msg):
        """Obtain Path and store it for later usage.

        This path is expected to be sorted. It is used as
        a baseline for the genetic algorithm.

        Only part of the points from the path is selected.
        """

        self.start_points = numpy.asarray([ [_p.pose.position.x, _p.pose.position.y ] for _p in msg.poses ])

        print ("Racingline received.")

        if self.valid_points is not None:
            self.start_optimization(msg.header)


    def callback_validarea(self, msg):
        """Obtain valid grid points and start the optimization.

        Group of all valid points (within the track) is split into groups
        by the distance to the selected points.

        From each "bin" only one point is selected in the genetic algorithm.
        """

        self.valid_points = numpy.asarray([ [_p.x, _p.y ] for _p in msg.cells ])

        print ("Valid points received.")

        if self.start_points is not None:
            self.start_optimization(msg.header)


    # Reconfigure callbacks
    def reconf_config_file(self, new_value):
        """Callback on changing the value of the config file."""

        self.P.config_file = new_value

        try:
            self.load_config()
        except Exception as e:
            print (e)

        return new_value


    # Optimization
    def load_config(self):
        """Load the configuration for ng_trajectory."""

        if os.path.isfile(self.P.config_file.value):
            if os.access(self.P.config_file.value, os.R_OK):
                ng_trajectory.configurationLoad(self.P.config_file.value)
            else:
                raise IOError("File '%s' is not readable." % self.P.config_file.value)
        else:
            raise IOError("File '%s' does not exist." % self.P.config_file.value)


    def start_optimization(self, header):
        """Start the optimization and publish its results.

        Arguments:
        header -- Header class from the messages
        """

        fitness, rcandidate, tcandidate, result = ng_trajectory.execute(self.start_points, self.valid_points)

        self.pub_path.publish(
            Path(
                header = header,
                poses = [
                    PoseStamped(
                        pose = Pose(
                            position = Point(
                                point[0], point[1], 0
                            ),
                            orientation = Quaternion(
                                0, 0, 0, 1
                            ),
                        )
                    )
                    for i, point in enumerate(result)
                ],
            )
        )
