#!/usr/bin/env python3
# run.py
"""ROS1 Entrypoint for 'ng_trajectory_ros'.
"""

import rospy

from ng_trajectory_ros.module._run import RunNode


def main():
    """Starts a ROS node, registers the callbacks."""

    node = RunNode()

    # Update parameters
    if rospy.has_param("~"):
        node.P.update(rospy.get_param("~"))

    node.P.reconfigure()


    # Function spin() simply keeps python from exiting until this node is stopped.
    rospy.spin()


if __name__ == '__main__':
    main()
