#!/bin/sh
""":"
if type python3 >/dev/null 2>/dev/null; then
    subversion=$(python3 --version | cut -d. -f2)
    if test "$subversion" -ge 6; then
        exec python3 "$0" "$@"
    else
        echo "Default Python version is lower than Python 3.6. Trying to force the version." >&2
        for avail_subversion in $(whereis python3 | grep -o [0-9]\.[0-9] | sort | uniq | cut -d. -f2); do
            if test "$avail_subversion" -ge 6; then
                if type python3."$avail_subversion" >/dev/null 2>/dev/null; then
                    echo "Forcing execution using Python 3.$avail_subversion."
                    exec python3."$avail_subversion" "$0" "$@"
                fi
            fi
        done
        echo "No suitable version found. Exiting." >&2
        exit 2;
    fi
else
    echo "Unable to find Python 3. Exiting." >&2
    exit 1;
fi
"""
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


    # Load configuration
    try:
        node.load_config()
    except Exception as e:
        print (e)


    # Function spin() simply keeps python from exiting until this node is stopped.
    rospy.spin()


if __name__ == '__main__':
    main()
