import pdb

import numpy

import uuid
import yaml
import os

import geometry.quaternion

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
        self._attitude = None
        self._offboard_enabled = False

        save_id = 'estimator_test_' + str(uuid.uuid4())
        print('experiment id: {}'.format(save_id))
        base_path = '/opt/autonomy/test_data'
        self._save_path = os.path.join(
            base_path, '{}.csv'.format(save_id))

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

        self._time_sync_subscriber = self.create_subscription(
            px4_msgs.msg.Timesync,
            '/Timesync_PubSubTopic',
            self._time_sync_callback,
            10)
        self._attitude_subscriber = self.create_subscription(
            px4_msgs.msg.VehicleAttitude,
            '/VehicleAttitude_PubSubTopic',
            self._attitude_callback,
            10)
        self._control_mode_subscriber = self.create_subscription(
            px4_msgs.msg.VehicleControlMode,
            '/VehicleControlMode_PubSubTopic',
            self._control_mode_callback,
            10)

        self._offboard_timer = self.create_timer(
            0.25, self._offboard_callback)
        self._estimator_offboard_timer = self.create_timer(
            0.05, self._estimator_offboard_callback)
        self._data_save_timer = self.create_timer(
            0.05, self._save_data)

    def now_to_us(self):
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return int((s + ns / 1.e9) * 1.e6)

    @property
    def px4_time(self):
        return self.now_to_us() + self._px4_time_offset

    def _time_sync_callback(self, msg):
        self._px4_time_offset = int(msg.timestamp - self.now_to_us())

    def _offboard_callback(self):
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

    def _estimator_offboard_callback(self):
        estimator_offboard_msg = px4_msgs.msg.EstimatorOffboard()
        estimator_offboard_msg.timestamp = self.px4_time
        estimator_offboard_msg.valid = True

        x = numpy.sqrt(2.0) / 2.0
        estimator_offboard_msg.quaternion = numpy.array(
            [x, x, 0.0, 0.0], dtype=numpy.float32)
        estimator_offboard_msg.update_quaternion = True

        self._estimator_offboard_publisher.publish(estimator_offboard_msg)

    def _control_mode_callback(self, msg):
        self._offboard_enabled = msg.flag_control_offboard_enabled

    def _attitude_callback(self, msg):
        self._attitude = geometry.quaternion.Quaternion(x=msg.q)

    def _save_data(self):
        if self._attitude is None or self._offboard_enabled is None:
            return
        data_to_save = numpy.array(numpy.hstack((
            self.now_to_us(),
            self._attitude.x,
            self._offboard_enabled)), ndmin=2)
        with open(self._save_path, 'ba') as logfile:
            numpy.savetxt(logfile, data_to_save, delimiter=',')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



