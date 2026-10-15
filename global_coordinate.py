#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GlobalCoordinate(Node):

    def __init__(self):
        super().__init__('global_coordinate')

        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_separation', 0.17)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('rate_hz', 10.0)

        self.r = self.get_parameter('wheel_radius').value
        self.s = self.get_parameter('wheel_separation').value
        self.theta = self.get_parameter('initial_theta').value
        self.dt = 1.0 / self.get_parameter('rate_hz').value

        self.wl = 0.0
        self.wr = 0.0
        self.x = 0.0
        self.y = 0.0

        self.create_subscription(
            Float64, '/left_wheel/command', self.left_callback, 10
        )
        self.create_subscription(
            Float64, '/right_wheel/command', self.right_callback, 10
        )

        self.create_timer(self.dt, self.calculate)

        self.get_logger().info(
            'Global Coordinate aktif, menunggu perintah kecepatan roda.'
        )

    def left_callback(self, msg):
        self.wl = msg.data

    def right_callback(self, msg):
        self.wr = msg.data

    def calculate(self):
        v_left = self.r * self.wl
        v_right = self.r * self.wr

        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.s

        vx = v * math.cos(self.theta)
        vy = v * math.sin(self.theta)

        radius = float('inf')
        if abs(omega) > 1e-9:
            radius = v / omega

        self.x += vx * self.dt
        self.y += vy * self.dt
        self.theta += omega * self.dt
        self.theta = math.atan2(
            math.sin(self.theta), math.cos(self.theta)
        )

        self.get_logger().info(
            f'\nV_L = {v_left:.3f} m/s | V_R = {v_right:.3f} m/s'
            f'\nV_B = {v:.3f} m/s | Omega = {omega:.3f} rad/s'
            f'\nVx global = {vx:.3f} m/s | Vy global = {vy:.3f} m/s'
            f'\nX = {self.x:.3f} m | Y = {self.y:.3f} m'
            f'\nTheta = {math.degrees(self.theta):.1f} deg'
            f' | Radius = {radius:.3f} m',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = GlobalCoordinate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()