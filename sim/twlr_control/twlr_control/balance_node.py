#!/usr/bin/env python3
"""TWLR balance controller (stage 1: legs fixed, cascade PID).

Subscribes : /imu/data (sensor_msgs/Imu), /joint_states (sensor_msgs/JointState),
             /cmd_vel (geometry_msgs/Twist)
Publishes  : /wheel_L/cmd_torque, /wheel_R/cmd_torque (std_msgs/Float64, N m, + = forward)
             /balance/debug (std_msgs/Float64MultiArray)
               [0] pitch deg  [1] pitch_ref deg  [2] speed m/s  [3] speed_ref m/s
               [4] tau_L N m  [5] tau_R N m      [6] yaw rate rad/s  [7] fallen 0/1
             /balance/<name> (std_msgs/Float64), the same values with readable names
               pitch_deg, pitch_ref_deg, speed, speed_ref, torque_L, torque_R, yaw_rate, fallen
Gains can be changed while running:  ros2 param set /balance_controller kp_pitch 40.0
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

from twlr_control.balance_core import BalanceCore, DEFAULTS


class BalanceNode(Node):
    def __init__(self):
        super().__init__('balance_controller')
        params = {}
        for k, v in DEFAULTS.items():
            params[k] = self.declare_parameter(k, v).value
        self.left_joint = self.declare_parameter('left_wheel_joint', 'wheel_L_joint').value
        self.right_joint = self.declare_parameter('right_wheel_joint', 'wheel_R_joint').value
        # for the real robot: flip a sign if a motor/encoder turns the other way
        self.left_sign = self.declare_parameter('left_wheel_sign', 1.0).value
        self.right_sign = self.declare_parameter('right_wheel_sign', 1.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        self.enabled = self.declare_parameter('enabled', True).value
        self.core = BalanceCore(params)
        self.add_on_set_parameters_callback(self.on_params)

        self.wl = self.wr = 0.0
        self.have_joints = False
        self.v_cmd = self.yaw_cmd = 0.0
        self.last_cmd = None
        self.last_t = None
        self.was_fallen = False

        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.pub_l = self.create_publisher(Float64, '/wheel_L/cmd_torque', 10)
        self.pub_r = self.create_publisher(Float64, '/wheel_R/cmd_torque', 10)
        self.pub_dbg = self.create_publisher(Float64MultiArray, '/balance/debug', 10)
        self.dbg_names = ['pitch_deg', 'pitch_ref_deg', 'speed', 'speed_ref',
                          'torque_L', 'torque_R', 'yaw_rate', 'fallen']
        self.pub_named = [self.create_publisher(Float64, '/balance/' + n, 10)
                          for n in self.dbg_names]
        self.get_logger().info('balance controller ready, pitch offset %.2f deg'
                               % params['pitch_offset_deg'])

    def on_params(self, plist):
        for prm in plist:
            if prm.name in self.core.p:
                self.core.p[prm.name] = float(prm.value)
            elif prm.name == 'enabled':
                self.enabled = bool(prm.value)
                if not self.enabled:
                    self.core.reset()
            elif prm.name == 'left_wheel_sign':
                self.left_sign = float(prm.value)
            elif prm.name == 'right_wheel_sign':
                self.right_sign = float(prm.value)
            elif prm.name == 'cmd_timeout':
                self.cmd_timeout = float(prm.value)
            self.get_logger().info('param %s = %s' % (prm.name, prm.value))
        return SetParametersResult(successful=True)

    def on_joints(self, msg):
        try:
            il = msg.name.index(self.left_joint)
            ir = msg.name.index(self.right_joint)
        except ValueError:
            return
        if len(msg.velocity) > max(il, ir):
            self.wl = self.left_sign * msg.velocity[il]
            self.wr = self.right_sign * msg.velocity[ir]
            self.have_joints = True

    def on_cmd(self, msg):
        self.v_cmd = msg.linear.x
        self.yaw_cmd = msg.angular.z
        self.last_cmd = self.get_clock().now()

    def on_imu(self, msg):
        now = self.get_clock().now()
        # dt from the IMU message's own time stamp, so it always matches the data.
        # (falls back to the node clock if the driver leaves the stamp empty)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = now.nanoseconds * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        if dt <= 0.0:                      # duplicate / out-of-order sample: skip it
            return
        self.last_t = t
        if dt > 0.1:                       # long gap (sim restart, sensor dropout)
            self.get_logger().warn('IMU gap %.3f s - controller reset' % dt)
            self.core.reset()
            return

        q = msg.orientation
        s = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, s)))     # + = nose down
        pitch_rate = msg.angular_velocity.y
        yaw_rate = msg.angular_velocity.z

        if self.last_cmd is not None and \
                (now - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            self.v_cmd = self.yaw_cmd = 0.0

        if not self.enabled or not self.have_joints:
            self.publish(0.0, 0.0)
            return

        o = self.core.step(dt, pitch, pitch_rate, yaw_rate, self.wl, self.wr,
                           self.v_cmd, self.yaw_cmd)
        self.publish(self.left_sign * o['tau_l'], self.right_sign * o['tau_r'])

        dbg = Float64MultiArray()
        dbg.data = [math.degrees(o['pitch']), math.degrees(o['pitch_ref']), o['v'],
                    o['v_ref'], o['tau_l'], o['tau_r'], yaw_rate, float(o['fallen'])]
        self.pub_dbg.publish(dbg)
        for pub, val in zip(self.pub_named, dbg.data):
            pub.publish(Float64(data=float(val)))

        if o['fallen'] and not self.was_fallen:
            self.get_logger().warn('FALLEN (pitch %.1f deg) - motors off' % dbg.data[0])
        if not o['fallen'] and self.was_fallen:
            self.get_logger().info('re-armed')
        self.was_fallen = o['fallen']
        self.get_logger().info(
            'pitch %6.2f ref %6.2f deg | v %5.2f ref %5.2f m/s | tau L %5.2f R %5.2f'
            % (dbg.data[0], dbg.data[1], o['v'], o['v_ref'], o['tau_l'], o['tau_r']),
            throttle_duration_sec=1.0)

    def publish(self, tl, tr):
        self.pub_l.publish(Float64(data=float(tl)))
        self.pub_r.publish(Float64(data=float(tr)))


def main():
    rclpy.init()
    node = BalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
