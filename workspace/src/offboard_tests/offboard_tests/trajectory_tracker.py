import pdb

import copy

import numpy

import rospy

import communications.ros
import geometry.conversions
import geometry.shapes
import geodesy.conversions
import geodesy.calculations
import robot_control.path_following

import aircraft_resources.message_trackers

import std_msgs.msg
import geometry_msgs.msg
import avia_fixed_wing_msgs.msg

from aircraft_navigation.trajectory_tracker import TrajectoryTracker
from spirograph_navigation.spirograph_controller import SpirographController

class SpirographTracker(TrajectoryTracker):
    """Track a trajectory

    This inherit from the normal TrajectoryTracker so that we get all its stuff
    but we're going to lay the ability to track a spirograph on top of it.
    """
    def __init__(self):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        self._tracker = None
        self._autonomy_state = ''
        self._arrival_time = None

        self._spirograph_parameters = communications.ros.wait_for_param(
            'test/spirograph', rospy.get_name())
        self._spirograph_center = numpy.array(
            self._spirograph_parameters['lla'], ndmin=2)
        r = (
            self._spirograph_parameters['bound'] /
            2.0 /
            (self._spirograph_parameters['n_petals'] - 1))
        d = self._spirograph_parameters['bound'] / 2.0
        R = self._spirograph_parameters['bound'] + r - d
        self._direction = self._spirograph_parameters['direction']
        # so that we can use numpy.pi, theta_0 is defined in fractions of pi
        self._theta_0 = self._spirograph_parameters['theta_0'] * numpy.pi

        self._spirograph = geometry.shapes.Hypotrochoid(
            R, r, d, self._direction)
        self._spirograph_control = SpirographController(
            self._spirograph_center,
            self._spirograph_parameters['bound'],
            self._spirograph_parameters['n_petals'],
            self._direction,
            50.0, #TODO: look up or get somewhere
            is_ned=False,
            theta_0=self._theta_0)

        self._maccready_publish_timer = rospy.Timer(
            rospy.Duration(5.0), self._publish_maccready)

        super(SpirographTracker, self).__init__()

    def _init_subscribers(self, namespace=''):
        """Start all of the subscribers

        The state subscribers all live in a message tracker, we need to start
        their subscribers up too

        Arguments:
            namespace: optional namespace to put this stuff under

        Returns:
            no returns
        """
        self._autonomy_state_subsciber = rospy.Subscriber(
            namespace + '/autonomy/state',
            avia_fixed_wing_msgs.msg.AutonomyState,
            self._autonomy_state_calback,
            queue_size=1)

        super(SpirographTracker, self)._init_subscribers(namespace)

    def _init_publishers(self, namespace=''):
        """Start publishers

        This initializes a maccready publisher and then initializes the base
        publishers

        Arguments:
            namespace: optional namespace to put stuff under

        Returns:
            no returns
        """
        self._maccready_publisher = rospy.Publisher(
            'estimators/maccready',
            avia_fixed_wing_msgs.msg.MacCready,
            queue_size=1)

        super(SpirographTracker, self)._init_publishers(namespace)

    def _autonomy_state_calback(self, msg):
        """Callback for autonomy state message

        Arguments:
            msg: avia_fixed_wing_msgs.AutonomyState message

        Returns:
            no returns
        """
        if msg.state == 'Spirograph':
            if self._autonomy_state != msg.state:
                self._tracker = SpirographController(
                    self._spirograph_center,
                    self._spirograph_parameters['bound'],
                    self._spirograph_parameters['n_petals'],
                    self._direction,
                    50.0, #TODO: look up or get somewhere
                    is_ned=False,
                    theta_0=self._theta_0)
                theta_end = (
                    numpy.pi * 3.0 +
                    numpy.pi / self._spirograph_parameters['n_petals'])
                distance_remaining = self._spirograph.path_length(
                    self._tracker.theta, theta_end)
                self._arrival_time = (
                    distance_remaining / self._spirograph_parameters['speed'] +
                    msg.header.stamp.to_sec())
            self._running = True
        self._autonomy_state = msg.state

    def _service_tracker(self, event_data=None):
        super(SpirographTracker, self)._service_tracker(event_data)

    def _publish_maccready(self, event_data):
        """Publish the current maccready setting

        Arguments:
            event_data: information about the ros timer event calling this

        Returns:
            no returns
        """
        v_mc = 0.0
        if (
            isinstance(self._tracker, SpirographController) and
            self._arrival_time):
            theta_end = (
                numpy.pi * 3.0 +
                numpy.pi / self._spirograph_parameters['n_petals'])
            distance_remaining = self._spirograph.path_length(
                self._tracker.theta, theta_end)
            time_remaining = (
                self._arrival_time - event_data.current_real.to_sec())
            v_required = distance_remaining / time_remaining
            sigma = self.aircraft.sigma()
            v_mc = self.aircraft.performance.MC_for_speed(
                v_required,
                self.aircraft.crosswind,
                self.aircraft.headwind,
                sigma=sigma)

        msg = avia_fixed_wing_msgs.msg.MacCready()
        msg.header.stamp = event_data.current_real
        msg.v_mc = v_mc
        self._maccready_publisher.publish(msg)

    def _check_progress(self):
        """Check our progress on the current waypoint

        Arguments:
            no arguments

        Returns:
            is_segment_completed: True if we have reached the end of this
                flight segment, either because we are within the acceptance
                radius of the waypoint, or because we have completed the orbit
                time
        """
        if isinstance(self._tracker, SpirographController):
            theta_end = (
                numpy.pi * 3.0 +
                numpy.pi / self._spirograph_parameters['n_petals'])
            return self._tracker.theta > theta_end
        else:
            return super(SpirographTracker, self)._check_progress()
