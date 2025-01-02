import pdb

import numpy

import rclpy
from rclpy.node import Node

import px4_msgs.msg
import std_msgs.msg

def time_to_s(time):
    s, ns = time.seconds_nanoseconds()
    return float(s + ns / 1.e9)

class OffboardTestPublisher(Node):
    def __init__(self):
        super().__init__('offboard_test')
        self._px4_time_offset = int(0)

        self.start_time = self.get_clock().now()

        self._offboard_setpoint_mode = 'fixed_wing_hlp'
        self._offboard_setpoint_mode = 'fixed_wing_nav'

        self._offboard_publisher = self.create_publisher(
            px4_msgs.msg.OffboardControlMode,
            '/OffboardControlMode_PubSubTopic',
            10)
        self._estimator_offboard_publisher = self.create_publisher(
            px4_msgs.msg.EstimatorOffboard,
            '/EstimatorOffboard_PubSubTopic',
            10)
        self._attitude_setpoint_publisher = self.create_publisher(
            px4_msgs.msg.VehicleAttitudeSetpoint,
            '/VehicleAttitudeSetpoint_PubSubTopic',
            10)
        self._position_setpoint_publisher = self.create_publisher(
            px4_msgs.msg.VehicleLocalPositionSetpoint,
            '/VehicleLocalPositionSetpoint_PubSubTopic',
            10)
        self._control_mode_publisher = self.create_publisher(
            px4_msgs.msg.VehicleControlMode,
            '/VehicleControlMode_PubSubTopic',
            10)
        self._sp_publisher = self.create_publisher(
            px4_msgs.msg.PositionSetpointTriplet,
            '/PositionSetpointTriplet_PubSubTopic',
            10)
        self._fw_offboard_publisher = self.create_publisher(
            px4_msgs.msg.FixedWingOffboard,
            '/FixedWingOffboard_PubSubTopic',
            10)
        self._time_sync_subscriber = self.create_subscription(
            px4_msgs.msg.Timesync,
            '/Timesync_PubSubTopic',
            self._time_sync_callback,
            10)
        self._offboard_timer = self.create_timer(0.05, self._offboard_callback)
        self._pos_setpoint_timer = self.create_timer(0.05, self._position_setpoint_callback)
        self._att_setpoint_timer = self.create_timer(0.05, self._attitude_setpoint_callback)
        self._fw_offboard_timer = self.create_timer(0.05, self._fixed_wing_setpoint_callback)
        self._estimator_offboard_timer = self.create_timer(0.05, self._estimator_offboard_callback)

    def now_to_us(self):
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return int((s + ns / 1.e9) * 1.e6)

    @property
    def px4_time(self):
        return self.now_to_us() + self._px4_time_offset

    def _time_sync_callback(self, msg):
        self._px4_time_offset = int(msg.ts1 / 1.e3 - self.now_to_us())

    def _offboard_callback(self):
        offboard_msg = px4_msgs.msg.OffboardControlMode()
        if self._offboard_setpoint_mode == 'attitude':
            offboard_msg.timestamp = self.px4_time
            offboard_msg.ignore_thrust = False
            offboard_msg.ignore_attitude = False
            offboard_msg.ignore_bodyrate_x = False
            offboard_msg.ignore_bodyrate_y = False
            offboard_msg.ignore_bodyrate_z = False
            offboard_msg.ignore_position = True
            offboard_msg.ignore_velocity = True
            offboard_msg.ignore_acceleration_force = False
            offboard_msg.ignore_alt_hold = True
        else:
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

        vehicle_control_msg = px4_msgs.msg.VehicleControlMode()
        vehicle_control_msg.timestamp = self.px4_time
        vehicle_control_msg.flag_armed = False
        vehicle_control_msg.flag_control_offboard_enabled = True
        vehicle_control_msg.flag_control_auto_enabled = True
        vehicle_control_msg.flag_control_rates_enabled = True
        vehicle_control_msg.flag_control_attitude_enabled = True
        vehicle_control_msg.flag_control_velocity_enabled = True
        vehicle_control_msg.flag_control_position_enabled = True
        vehicle_control_msg.flag_control_altitude_enabled = True
        vehicle_control_msg.flag_control_climb_rate_enabled = True
        #self._control_mode_publisher.publish(vehicle_control_msg)

    def _estimator_offboard_callback(self):
        estimator_offboard_msg = px4_msgs.msg.EstimatorOffboard()
        estimator_offboard_msg.timestamp = self.px4_time
        estimator_offboard_msg.valid = True

        estimator_offboard_msg.quaternion = numpy.array(
            [1.0, 0.0, 0.0, 0.0], dtype=numpy.float32)
        estimator_offboard_msg.update_quaternion = True

        self._estimator_offboard_publisher.publish(estimator_offboard_msg)

    def _attitude_setpoint_callback(self):
        sp_msg = px4_msgs.msg.VehicleAttitudeSetpoint()
        sp_msg.timestamp = self.px4_time
        sp_msg.roll_body = -0.5
        sp_msg.thrust_body = [0.0, 0.0, -1.0]

        if self._offboard_setpoint_mode == 'attitude':
            self._attitude_setpoint_publisher.publish(sp_msg)

    def _position_setpoint_callback(self):
        if self._offboard_setpoint_mode != 'position_setpoint':
            return
        sp_msg = px4_msgs.msg.PositionSetpointTriplet()

        this_sp = px4_msgs.msg.PositionSetpoint()
        this_sp.type = this_sp.SETPOINT_TYPE_OFFBOARD

        this_sp.x = 20.0
        this_sp.y = -10.0
        this_sp.z = -10.0
        this_sp.valid = True
        this_sp.position_valid = True

        this_sp.vx = 0.0
        this_sp.vy = 0.0
        this_sp.vz = 0.0
        this_sp.velocity_valid = False
        this_sp.alt_valid = True

        this_sp.lat = 0.0
        this_sp.lon = 0.0
        this_sp.alt = 10.0
        this_sp.yaw = 0.0
        this_sp.yaw_valid = False

        this_sp.yawspeed = 0.0
        this_sp.yawspeed_valid = False

        this_sp.a_x = 0.0
        this_sp.a_z = 0.0
        this_sp.a_y = 0.0
        this_sp.acceleration_valid = False
        this_sp.acceleration_is_force = False

        sp_msg.previous = this_sp
        sp_msg.current = this_sp
        sp_msg.next = this_sp
        self._sp_publisher.publish(sp_msg)
        return

        sp_msg = px4_msgs.msg.VehicleLocalPositionSetpoint()
        sp_msg.timestamp = self.px4_time
        sp_msg.x = 10.0
        sp_msg.y = 10.0
        sp_msg.z = -100.0

        self._position_setpoint_publisher.publish(sp_msg)

    def _fixed_wing_setpoint_callback(self):
        setpoint_generators = {
            'fixed_wing_hlp': self._hlp_sp_gen,
            'fixed_wing_nav': self._nav_sp_gen,
            }
        if self._offboard_setpoint_mode not in setpoint_generators:
            return

        sp_msg = setpoint_generators[self._offboard_setpoint_mode]()
        self._fw_offboard_publisher.publish(sp_msg)

    def _hlp_sp_gen(self):
        vrate = numpy.cos(
            2.0 * numpy.pi * time_to_s(self.get_clock().now()) / 30.0)

        sp_msg = px4_msgs.msg.FixedWingOffboard()
        sp_msg.timestamp = self.px4_time
        sp_msg.valid = True
        sp_msg.type = sp_msg.SETPOINT_TYPE_HIGH_LEVEL_PILOTAGE
        sp_msg.phi = -0.5
        sp_msg.ias = 15.0
        sp_msg.vertical_rate = vrate
        return sp_msg

    def _nav_sp_gen(self):
        h = 610.0
        heading = numpy.deg2rad(270.0)

        sp_msg = px4_msgs.msg.FixedWingOffboard()
        sp_msg.timestamp = self.px4_time
        sp_msg.valid = True
        sp_msg.type = sp_msg.SETPOINT_TYPE_NAVIGATION
        sp_msg.psi = heading
        sp_msg.ias = 15.0
        sp_msg.h = h
        return sp_msg

    def _control_mode_callback(self, msg):
        print('offboard mode: {}'.format(msg.flag_control_offboard_enabled))

def main(args=None):
    rclpy.init(args=args)
    node = OffboardTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



