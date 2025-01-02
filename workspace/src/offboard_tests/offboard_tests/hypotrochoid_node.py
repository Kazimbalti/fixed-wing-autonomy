#!/usr/bin/env python3

import copy
import os
import pdb
import sys
import uuid

import numpy
import yaml

import rclpy
from rclpy.node import Node

import geometry.conversions
import geometry.shapes
import geodesy.conversions
import geodesy.calculations
import robot_control.path_following

import std_msgs.msg
import px4_msgs.msg

from hypotrochoid_test.hypotrochoid_controller import SpirographController

class SpirographTracker(Node):
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
        super().__init__('hypotrochoid_controller')

        self._tracker = None
        self._state = 'pre-start'
        self._px4_time_offset = int(0)
        self._offboard_enabled = False

        save_id = 'hypotrochoid_abort_' + str(uuid.uuid4())
        print('experiment id: {}'.format(save_id))
        base_path = '/opt/autonomy/test_data'
        self._save_path = os.path.join(
            base_path, '{}.csv'.format(save_id))

        self._attitude = None
        self._h_cmd = 100.0
        self._hdot_cmd = 0.0
        self._phi_cmd = 0.0
        self._V = None
        self._lla = None
        self._v_ias = None

        L = 30.0

        self._center_point = numpy.deg2rad([40.144235, -105.241073, 0.0])
        self._spirograph_parameters = {
            'lla': self._center_point,
            'bound': 100.0,
            'n_petals': 6,
            'theta_0': 1.0,
            'direction': -1,
            'speed': 17.0,
            }
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

        metadata = {
            'hypotrochoid': copy.deepcopy(self._spirograph_parameters),
            'controller': L
            }
        metadata['hypotrochoid']['lla'] = numpy.rad2deg(self._center_point).tolist()
        metadata_path = os.path.join(base_path, '{}.yaml'.format(save_id))
        with open(metadata_path, 'w') as yfile:
            yaml.dump(metadata, yfile)

        X_start_enu = self._spirograph.X(self._theta_0)
        X_start_ned = geodesy.conversions.enu_to_ned(
            numpy.hstack((X_start_enu, 0.0)))
        self._start_point = numpy.array(
            geodesy.conversions.ned_to_lla(X_start_ned, self._spirograph_center),
            ndmin=2)

        self._spirograph_control = SpirographController(
            self._spirograph_center,
            self._spirograph_parameters['bound'],
            self._spirograph_parameters['n_petals'],
            self._direction,
            L, #TODO: look up or get somewhere
            is_ned=False,
            theta_0=self._theta_0)
        self._initial_control = robot_control.path_following.ParkController(
            self._start_point,
            50.0,
            is_ned=False)

        self._init_publishers()
        self._init_subscribers()
        self._init_timers()

    def _init_subscribers(self):
        """Start all of the subscribers

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self._autonomy_state_subsciber = self.create_subscription(
            std_msgs.msg.String,
            '/autonomy/state',
            self._autonomy_state_callback,
            10)

        self._position_subscriber = self.create_subscription(
            px4_msgs.msg.VehicleLocalPosition,
            '/VehicleLocalPosition_PubSubTopic',
            self._position_callback,
            10)
        self._attitude_subscriber = self.create_subscription(
            px4_msgs.msg.VehicleAttitude,
            '/VehicleAttitude_PubSubTopic',
            self._attitude_callback,
            10)
        self._time_sync_subscriber = self.create_subscription(
            px4_msgs.msg.Timesync,
            '/Timesync_PubSubTopic',
            self._time_sync_callback,
            10)
        self._airspeed_subscriber = self.create_subscription(
            px4_msgs.msg.Airspeed,
            '/Airspeed_PubSubTopic',
            self._airspeed_callback,
            10)
        self._control_mode_subscriber = self.create_subscription(
            px4_msgs.msg.VehicleControlMode,
            '/VehicleControlMode_PubSubTopic',
            self._control_mode_callback,
            10)

    def _init_publishers(self):
        """Start publishers

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self._fw_offboard_publisher = self.create_publisher(
            px4_msgs.msg.FixedWingOffboard,
            '/FixedWingOffboard_PubSubTopic',
            10)
        self._offboard_publisher = self.create_publisher(
            px4_msgs.msg.OffboardControlMode,
            '/OffboardControlMode_PubSubTopic',
            10)

    def _init_timers(self):
        self._control_timer = self.create_timer(
            0.05, self._compute_control)

        self._offboard_timer = self.create_timer(
            0.25, self._offboard_control)

        self._data_save_timer = self.create_timer(
            0.05, self._save_data)

    def now_to_us(self):
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return int((s + ns / 1.e9) * 1.e6)

    @property
    def px4_time(self):
        trim = int(0.0001 * 1e6)
        trim = int(-15.e4)
        return self.now_to_us() + self._px4_time_offset + trim

    def _time_sync_callback(self, msg):
        self._px4_time_offset = int(msg.timestamp - self.now_to_us())

    def _autonomy_state_callback(self, msg):
        """Callback for autonomy state message

        Arguments:
            msg: std_msgs/String message

        Returns:
            no returns
        """
        if msg.data == 'start':
            print('flying to start point')
            self._tracker = self._initial_control
            self._state = msg.data

    def _position_callback(self, msg):
        lla_ref = numpy.array(
            [
                numpy.deg2rad(msg.ref_lat),
                numpy.deg2rad(msg.ref_lon),
                msg.ref_alt],
            ndmin=2)
        ned = numpy.array([msg.x, msg.y, msg.z])
        self._lla = geodesy.conversions.ned_to_lla(ned, lla_ref)
        self._V = numpy.array([msg.vx, msg.vy, msg.vz])

    def _attitude_callback(self, msg):
        self._attitude = geometry.quaternion.Quaternion(x=msg.q)

    def _airspeed_callback(self, msg):
        self._v_ias = msg.indicated_airspeed_m_s

    def _control_mode_callback(self, msg):
        self._offboard_enabled = msg.flag_control_offboard_enabled

    def _compute_control(self):
        if self._state == 'pre-start':
            return

        if self._V is None or self._lla is None:
            return

        self._tracker.update_state(self._lla[0], self._V)

        if self._state == 'start':
            if self._check_progress():
                print('starting run')
                self._state = 'running'
                self._tracker = self._spirograph_control
                self._compute_control()
                return
            a_cmd = self._tracker.command(from_current_location=True)

        if self._state == 'running':
            a_cmd = self._tracker.command()

        # numerical drama ensues if the velocity is zero. If our velocity is
        # too small then just hold zero bank angle
        if numpy.linalg.norm(self._V) < 1.0:
            self._phi_cmd = 0.0

        yaw_rotation = geometry.rotations.zrot(-self._attitude.psi)
        inertial_direction = yaw_rotation.dot(geometry.helpers.unit_vector(0))

        acceleration_normal_to_velocity = numpy.cross(
            numpy.cross(inertial_direction, a_cmd),
            inertial_direction)
        acceleration_body_frame = numpy.dot(
            yaw_rotation.T,
            acceleration_normal_to_velocity)
        acceleration_body_frame += (geometry.helpers.unit_vector(2) * 9.806)
        self._phi_cmd = numpy.arctan2(
            acceleration_body_frame[1], acceleration_body_frame[2])

        delta_h = self._h_cmd - self._lla[0, 2]
        self._hdot_cmd = numpy.clip(delta_h / 10.0, -2.0, 2.0)

        sp_msg = px4_msgs.msg.FixedWingOffboard()
        sp_msg.timestamp = self.px4_time
        sp_msg.valid = True
        sp_msg.type = sp_msg.SETPOINT_TYPE_HIGH_LEVEL_PILOTAGE
        sp_msg.phi = self._phi_cmd
        sp_msg.ias = self._spirograph_parameters['speed']
        sp_msg.vertical_rate = self._hdot_cmd
        self._fw_offboard_publisher.publish(sp_msg)

        if self._check_progress():
            print('finished')
            sys.exit()

    def _offboard_control(self):
        offboard_msg = px4_msgs.msg.OffboardControlMode()
        offboard_msg.timestamp = self.px4_time
        offboard_msg.ignore_thrust = True
        offboard_msg.ignore_attitude = True
        offboard_msg.ignore_bodyrate_x = True
        offboard_msg.ignore_bodyrate_y = True
        offboard_msg.ignore_bodyrate_z = True
        offboard_msg.ignore_position = False
        offboard_msg.ignore_velocity = True
        offboard_msg.ignore_acceleration_force = True
        offboard_msg.ignore_alt_hold = False
        self._offboard_publisher.publish(offboard_msg)

    def _save_data(self):
        if self._V is None or self._attitude is None or self._v_ias is None:
            return
        mode_dict = {
            'pre-start': 0,
            'start': 1,
            'running': 2}
        if self._state not in mode_dict:
            mode = -1
        else:
            mode = mode_dict[self._state]
        if self._state == 'running':
            theta = self._tracker.theta
        else:
            theta = 0.0
        data_to_save = numpy.array(numpy.hstack((
            self.now_to_us(),
            self._lla[0],
            self._V,
            self._attitude.x,
            self._v_ias,
            self._phi_cmd,
            self._hdot_cmd,
            self._spirograph_parameters['speed'],
            theta,
            mode,
            self._offboard_enabled)), ndmin=2)
        with open(self._save_path, 'ba') as logfile:
            numpy.savetxt(logfile, data_to_save, delimiter=',')

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
        if isinstance(self._tracker, SpirographController) and self._state == 'running':
            theta_end = (
                numpy.pi * 3.0 +
                numpy.pi / self._spirograph_parameters['n_petals'])
            return self._tracker.theta > theta_end
        elif self._state == 'start':
            to_go = geodesy.conversions.lla_to_ned(self._lla, self._start_point)
            return numpy.linalg.norm(to_go[0, 0:2]) < 25.0
        else:
            return False

def main():
    rclpy.init()
    this_node = SpirographTracker()
    rclpy.spin(this_node)
    this_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
