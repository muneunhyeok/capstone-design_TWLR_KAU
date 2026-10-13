#!/usr/bin/env python3
"""Sign check: apply the same torque to both wheels for a few seconds.
Positive torque should push the robot FORWARD (+x). Watch it in Gazebo.
ros2 run twlr_description wheel_torque_test.py --ros-args -p torque:=0.5 -p duration:=2.0
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class WheelTorqueTest(Node):
    def __init__(self):
        super().__init__('wheel_torque_test')
        self.torque = self.declare_parameter('torque', 0.5).value
        self.duration = self.declare_parameter('duration', 2.0).value
        self.pub_l = self.create_publisher(Float64, '/wheel_L/cmd_torque', 10)
        self.pub_r = self.create_publisher(Float64, '/wheel_R/cmd_torque', 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.005, self.tick)

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        u = self.torque if t < self.duration else 0.0
        self.pub_l.publish(Float64(data=u))
        self.pub_r.publish(Float64(data=u))
        if t > self.duration + 0.5:
            self.get_logger().info('done')
            raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(WheelTorqueTest())
    except SystemExit:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
