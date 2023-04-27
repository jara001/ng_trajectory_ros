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

import math
import numpy
import ng_trajectory


# Messages
from nav_msgs.msg import Path
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from plan_msgs.msg import Trajectory as TrajectoryP
from autoware_auto_msgs.msg import Trajectory as TrajectoryA
from autoware_auto_msgs.msg import TrajectoryPoint


# Global variables
_lf = 0.191         # distance from the center of mass to the front axle
_lr = 0.139         # distance from the center of mass to the rear axle


######################
# Utilities
######################

def euler_to_quaternion(roll, pitch, yaw):
    """Taken from 'profile_trajectory2'."""

    qx = numpy.sin(roll/2) * numpy.cos(pitch/2) * numpy.cos(yaw/2) - numpy.cos(roll/2) * numpy.sin(pitch/2) * numpy.sin(yaw/2)
    qy = numpy.cos(roll/2) * numpy.sin(pitch/2) * numpy.cos(yaw/2) + numpy.sin(roll/2) * numpy.cos(pitch/2) * numpy.sin(yaw/2)
    qz = numpy.cos(roll/2) * numpy.cos(pitch/2) * numpy.sin(yaw/2) - numpy.sin(roll/2) * numpy.sin(pitch/2) * numpy.cos(yaw/2)
    qw = numpy.cos(roll/2) * numpy.cos(pitch/2) * numpy.cos(yaw/2) + numpy.sin(roll/2) * numpy.sin(pitch/2) * numpy.sin(yaw/2)

    return [qx, qy, qz, qw]


######################
# RunNode
######################

class RunNode(Node):

    # Node variables
    start_points = None
    valid_points = None
    P = None
    header = None
    configuration_loaded = None

    # Result
    fitness = None
    rcandidate = None
    tcandidate = None
    result = None
    _v = None
    _a = None
    _t = None


    def __init__(self):
        super(Node, self).__init__("ng_trajectory_ros")

        self.sub_path = self.Subscriber("path", Path, self.callback_path)
        self.sub_validarea = self.Subscriber("validarea", GridCells, self.callback_validarea)

        self.pub_path = self.Publisher("npath", Path, queue_size = 1, latch = True)
        self.pub_traj = self.Publisher("trajectory", TrajectoryP, queue_size = 1, latch = True)
        self.pub_traj_autoware = self.Publisher("trajectory/autoware", TrajectoryA, queue_size = 1, latch = True)


        # Parameters
        self.P = ParameterServer()

        # Note: Not sure whether this is the right way, but when
        #       we have this option... :)

        # Autopsy == 0.7 calls the callbacks when using reconfigure().
        # This is required, as otherwise the callbacks would not be called when setting the first value from params (in opposition to ROS1).
        # Maybe it is an issue of autopsy-ROS2 relation, but who knows.

        # So we reverse the order so the trajectory is published only once.
        self.P.use_autoware = {"default": False,
            "description": "When True, trajectory is published as 'autoware_auto_msgs/Trajectory'.",
            "callback": self.reconf_use_autoware
        }

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

        self.header = msg.header

        print ("Racingline received.")

        if self.valid_points is not None:
            self.start_optimization()


    def callback_validarea(self, msg):
        """Obtain valid grid points and start the optimization.

        Group of all valid points (within the track) is split into groups
        by the distance to the selected points.

        From each "bin" only one point is selected in the genetic algorithm.
        """

        self.valid_points = numpy.asarray([ [_p.x, _p.y ] for _p in msg.cells ])

        self.header = msg.header

        print ("Valid points received.")

        if self.start_points is not None:
            self.start_optimization()


    # Reconfigure callbacks
    def reconf_config_file(self, new_value):
        """Callback on changing the value of the config file."""

        self.P.config_file = new_value

        try:
            self.load_config()
        except Exception as e:
            print (e)

        return new_value


    def reconf_use_autoware(self, new_value):
        """Callback on changing the message type for Trajectory."""
        #print("Reconf", new_value)
        #self.pub_traj.unregister()

        #if new_value:
        #    self.pub_traj = self.Publisher("trajectory", TrajectoryA, queue_size = 1, latch = True)
        #else:
        #    self.pub_traj = self.Publisher("trajectory", TrajectoryP, queue_size = 1, latch = True)

        self.publish_trajectory(new_value)

        return new_value


    def publish_trajectory(self, use_autoware = None):
        """Publish the trajectory."""

        _autoware = use_autoware if use_autoware is not None else self.P.use_autoware.value
        #print("Pub:", _autoware)
        if _autoware:
            if self._t is not None:
                self.publish_trajectory_autoware()
        else:
            if self._t is not None:
                self.publish_trajectory_plan()


    # Optimization
    def load_config(self):
        """Load the configuration for ng_trajectory."""

        if os.path.isfile(self.P.config_file.value):
            if os.access(self.P.config_file.value, os.R_OK):
                ng_trajectory.configurationLoad(self.P.config_file.value)
                self.configuration_loaded = True

                if self.start_points is not None and self.valid_points is not None:
                    self.start_optimization()
            else:
                raise IOError("File '%s' is not readable." % self.P.config_file.value)
        else:
            raise IOError("File '%s' does not exist." % self.P.config_file.value)


    def start_optimization(self):
        """Start the optimization and publish its results."""

        if not self.configuration_loaded:
            return

        self._t = None

        self.fitness, self.rcandidate, self.tcandidate, self.result = ng_trajectory.execute(self.start_points, self.valid_points)

        # Trajectory -- currently profile only
        _criterion = ng_trajectory.criterions.__getattribute__("profile")

        # Rebuild the configuration
        _alg = {**ng_trajectory.main.CONFIGURATION}

        for level in _alg.get("cascade", [{}]):
            _alg = {**_alg, **level}

        _criterion.init(**{**_alg, **_alg.get("criterion_init", {})})
        self._v, self._a, self._t = _criterion.profiler.profileCompute(points = self.result, overlap = {**_alg, **_alg.get("criterion_args", {})}.get("overlap", 0))


        # Publish path
        self.pub_path.publish(
            Path(
                header = self.header,
                poses = [
                    PoseStamped(
                        pose = Pose(
                            position = Point(
                                x = point[0], y = point[1], z = 0.0
                            ),
                            orientation = Quaternion(
                                x = 0.0, y = 0.0, z = 0.0, w = 1.0
                            ),
                        )
                    )
                    for i, point in enumerate(self.result)
                ],
            )
        )

        self.publish_trajectory()


    def publish_trajectory_plan(self):
        """Publish the trajectory as 'plan_msgs/Trajectory'."""

        if self._t is None:
            return

        # Publish trajectory
        # Taken from 'profile_trajectory2'
        msg = TrajectoryP()
        msg.header = self.header

        # Poses
        msg.poses = []
        # Taken from 'car_trajectory_generator.py' by David Kopecky
        for i, p in enumerate(self.result):
            if i < len(self.result) - 1:
                dx = self.result[i + 1, 0] - p[0]
                dy = self.result[i + 1, 1] - p[1]
            else:
                dx = self.result[1, 0] - p[0]
                dy = self.result[1, 1] - p[1]

            # print("dx {} dy {}".format(dx, dy))
            angle = math.atan2(dy, dx)
            q = euler_to_quaternion(0, 0, angle)

            ps = Pose()
            ps.position.x = p[0]
            ps.position.y = p[1]

            ps.orientation.z = q[2]
            ps.orientation.w = q[3]
            msg.poses.append(ps)

        # Curvature
        msg.curvatures = self.result[:, -1].ravel().tolist()#tolist()

        # Velocity
        msg.velocities = self._v.ravel().tolist()#T.tolist()[0]

        # Acceleration
        msg.accelerations = self._a.ravel().tolist()#T.tolist()[0]

        self.pub_traj.publish(msg)

        #rospy.loginfo("Profile trajectory finished with predicted lap time %ss." % ("%f" % _t[-1]).rstrip("0").rstrip("."))


    def publish_trajectory_autoware(self):
        """Publish the trajectory as 'plan_msgs/Trajectory'."""

        if self._t is None:
            return

        # Publish trajectory
        # Taken from 'profile_trajectory2'
        msg2 = TrajectoryA()
        msg2.header = self.header

        # Poses
        poses = []
        # Taken from 'car_trajectory_generator.py' by David Kopecky
        for i, p in enumerate(self.result):
            if i < len(self.result) - 1:
                dx = self.result[i + 1, 0] - p[0]
                dy = self.result[i + 1, 1] - p[1]
            else:
                dx = self.result[1, 0] - p[0]
                dy = self.result[1, 1] - p[1]

            # print("dx {} dy {}".format(dx, dy))
            angle = math.atan2(dy, dx)
            q = euler_to_quaternion(0, 0, angle)

            ps = Pose()
            ps.position.x = p[0]
            ps.position.y = p[1]

            ps.orientation.z = q[2]
            ps.orientation.w = q[3]
            poses.append(ps)

        msg2.points = []
        for _i in range(len(poses)):
            _tp = TrajectoryPoint()

            try:
                _tp.time_from_start.secs = int(self._t[_i])
                _tp.time_from_start.nsecs = int((self._t[_i] - int(self._t[_i])) * 1e9)
            except:
                _tp.time_from_start.sec = int(self._t[_i])
                _tp.time_from_start.nanosec = int((self._t[_i] - int(self._t[_i])) * 1e9)

            _tp.pose = poses[_i]

            #_tp.longitudinal_velocity_mps = _v[_i]
            #_tp.lateral_velocity_mps = 0 # Yes, this is wrong.

            #_tp.acceleration_mps2 = _a[_i]

            #_tp.front_wheel_angle_rad = (1 / _ipol[_i, -1]) if _ipol[_i, -1] != 0 else 999999999

            # Slight talk about the following lines
            """
            Our velocity is probably the magnitude of the final velocity, i.e., of the vector.

                     y
                     ^      _> v
                     |   __/
                -----|--/-
               |     |_/  |
               |     O------> x
               |          |
                ----------

            The angle between the x(car) and v should be Beta [1]:

            $$
            Beta = atan( (l_r / ( l_f + l_r ) ) * tan(delta) ),
            $$

            where delta is steering (angle of the front wheels).

            An approximation (small-angle assumption) [2]:

            $$
            Beta = (l_r / ( l_f + l_r ) ) * delta
            $$


            Then, the longitudinal and lateral velocity should be obtained from [1] or [2, p. 28]:

            $$
            x\dot = v_long = v * cos(Beta)
            y\dot = v_lat = v * sin(Beta)
            $$

            Last thing that is missing is heading rate. According to [2, p. 7] this is obtained via:

            $$
            psi\dot = v * cos(Beta) * tan(delta) / (l_f + l_r) = v * cos(Beta) * kappa
            $$

            where kappa is the curvature of the circle that we would drive with steering delta.
            More useful is maybe the radius of the circle:

            $$
            R = 1 / kappa
            $$

            and

            $$
            tan(delta) = (l_f + l_r) / R = L / R = L * kappa
            $$


            So the final set of the equations should be:

            $$
            delta = atan(L * kappa)
            Beta = atan( (l_r / ( l_f + l_r ) ) * tan(delta) ) = atan(l_r * kappa)
            v_long = v * cos(Beta)
            v_lat = v * sin(Beta)
            psi\dot = v_long * kappa
            $$


            [1]: J. Kong, M. Pfeiffer, G. Schildbach and F. Borrelli,
                 "Kinematic and dynamic vehicle models for autonomous driving control design,"
                 2015 IEEE Intelligent Vehicles Symposium (IV), 2015, pp. 1094-1099,
                 doi: 10.1109/IVS.2015.7225830.
            [2]: J. Filip, 'Trajectory Tracking for Autonomous Vehicles',
                 Czech Technical University in Prague, Faculty of Electrical Engineering, 2018.
                 doi: 10.13140/RG.2.2.36089.93288.

            Note: CARE that both references probably use a sligtly different notation, e.g.,
                  'v' is not the same and position of the velocity vector is also not the same.

            """

            _L = _lf + _lr
            _kappa = self.result[_i, -1]
            _delta = math.atan(_L * _kappa)
            _beta = math.atan(_lr * _kappa)

            _tp.longitudinal_velocity_mps = float(self._v[_i] * math.cos(_beta))
            _tp.lateral_velocity_mps = float(self._v[_i] * math.sin(_beta))

            _tp.acceleration_mps2 = float(self._a[_i])

            _tp.heading_rate_rps = float(_tp.longitudinal_velocity_mps * _kappa)

            _tp.front_wheel_angle_rad = float(_delta)
            _tp.rear_wheel_angle_rad = 0.0

            msg2.points.append(_tp)

        self.pub_traj_autoware.publish(msg2)
