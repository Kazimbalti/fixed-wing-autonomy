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
import geometry.rotations
import geometry.shapes
import geodesy.conversions
import geodesy.calculations
import robot_control.path_following
from simulation.unicycle import ThreeDUnicycle

import std_msgs.msg
import px4_msgs.msg

class TrackerTracker(Node):
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
        super().__init__('guided_tracker_node')

        self._done = False
        self._tracker = None
        self._px4_time_offset = int(0)

        save_id = 'guided_' + str(uuid.uuid4())
        print('experiment id: {}'.format(save_id))
        base_path = '/opt/autonomy/test_data'
        self._save_path = os.path.join(
            base_path, '{}.csv'.format(save_id))

        self._attitude = None
        self._h_cmd = 100.0
        self._psi_cmd = 0.0
        self._V = None
        self._lla = None
        self._v_ias = None

        self._target_target_heading = 0.0

        self._target_parameters = {
            'integration_interval': 0.01,
            'heading_change_interval': 60.0,
            'heading_change_magnitude': 90.0,
            'heading_rate': 30.0,
            'speed': 15.0,
            }

        ref_pt = [40.142337, -105.238617, 0.0]
        self._ref_pt = numpy.deg2rad(numpy.array(ref_pt, ndmin=2))

        self._metadata = {
            'target': self._target_parameters,
            'ref_pt': ref_pt,
            'v_cmd': 18.0,
            }
        X0_target = numpy.array([0.0, 0.0, 0.0, 0.0])
        self._target = ThreeDUnicycle(
            X0_target,
            self._target_parameters['integration_interval']
            )
        metadata_path = os.path.join(base_path, '{}.yaml'.format(save_id))
        with open(metadata_path, 'w') as yfile:
            yaml.dump(self._metadata, yfile)

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

        self._data_save_timer = self.create_timer(
            0.05, self._save_data)

        self._integration_timer = self.create_timer(
            self._target_parameters['integration_interval'],
            self._integrate)

        self._heading_change_timer = self.create_timer(
            self._target_parameters['heading_change_interval'],
            self._heading_change)

        self._completion_timer = self.create_timer(
            600.0, self._done_callback)

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

    def _integrate(self):
        delta_heading = geometry.rotations.angle_difference(
            self._target.psi, self._target_target_heading)
        heading_rate = numpy.clip(
            delta_heading / self._target_parameters['integration_interval'],
            -self._target_parameters['heading_rate'],
            self._target_parameters['heading_rate'])
        target_input = numpy.array([
            heading_rate,
            self._target_parameters['speed'],
            0.0])
        self._target.rk4(target_input)

    def _heading_change(self):
        heading_change = (
            (2.0 * numpy.random.rand() - 1.0) *
            numpy.deg2rad(self._target_parameters['heading_change_magnitude'])
            )
        self._target_target_heading += heading_change

    def _compute_control(self):
        if self._V is None or self._lla is None:
            return

        ac_to_target = (
            self._target.X -
            geodesy.conversions.lla_to_ned(self._lla, self._ref_pt)[0]
            )
        self._psi_cmd = numpy.arctan2(ac_to_target[1], ac_to_target[0])
        print(numpy.hstack([ac_to_target[0:2], self._target.velocity[0:2] - self._V[0:2]]))

        sp_msg = px4_msgs.msg.FixedWingOffboard()
        sp_msg.timestamp = self.px4_time
        sp_msg.valid = True
        sp_msg.type = sp_msg.SETPOINT_TYPE_NAVIGATION
        sp_msg.psi = self._psi_cmd
        sp_msg.ias = self._metadata['v_cmd']
        sp_msg.h = self._h_cmd
        self._fw_offboard_publisher.publish(sp_msg)

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
        target_lla = geodesy.conversions.ned_to_lla(
            self._target.X, self._ref_pt)[0]
        data_to_save = numpy.array(numpy.hstack((
            self.now_to_us(),
            self._lla[0],
            self._V,
            self._attitude.x,
            self._v_ias,
            self._psi_cmd,
            self._h_cmd,
            self._metadata['v_cmd'],
            target_lla,
            self._target.psi,
            self._target.velocity)), ndmin=2)
        with open(self._save_path, 'ba') as logfile:
            numpy.savetxt(logfile, data_to_save, delimiter=',')

        if self._done:
            print('finished')
            sys.exit()


    def _done_callback(self):
        print('done!')
        self._done = True

def main():
    rclpy.init()
    this_node = TrackerTracker()
    rclpy.spin(this_node)
    this_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

