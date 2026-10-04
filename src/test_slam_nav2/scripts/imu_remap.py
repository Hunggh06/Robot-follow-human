#!/usr/bin/env python3
"""Lay yaw-rate tu IMU dien thoai: Gx -> /imu/yaw (frame base_link).

Dien thoai lap landscape, camera nhin truoc: truc Gx trung truc yaw
robot (test xoay trai 90 do: Gx duong). EKF chi can van toc yaw nen
node nay trich Gx sang angular_velocity.z, bo orientation (magnet
trong nha nhieu, khong tin duoc).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class ImuRemap(Node):
    def __init__(self):
        super().__init__('imu_remap')
        self.declare_parameter('gyro_bias_x', 0.0)
        self.bias = float(self.get_parameter('gyro_bias_x').value or 0.0)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Imu, '/phone/imu', self.cb, qos)
        self.pub = self.create_publisher(Imu, '/imu/yaw', 10)

    def cb(self, msg):
        o = Imu()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = 'base_link'
        o.angular_velocity.z = msg.angular_velocity.x - self.bias
        o.angular_velocity_covariance[8] = 0.05
        o.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(o)


def main():
    rclpy.init()
    node = ImuRemap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
