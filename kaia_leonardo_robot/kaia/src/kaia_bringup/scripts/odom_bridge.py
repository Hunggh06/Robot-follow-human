#!/usr/bin/env python3
"""odom_bridge — /wheel/odom (ESP32 Kaia) -> TF odom->base_footprint + republish /odom.

Lý do cần: firmware Kaia để trống header.frame_id/child_frame_id (tránh
String__assign khác phiên bản micro_ros). odom_to_tf_cpp của Leonardo copy
nguyên msg->header.frame_id nên sẽ broadcast TF rỗng. Bridge này tự điền
cố định odom/base_footprint, chỉ copy pose+twist — đúng như comment trong
firmware: odom_tf_bridge.py chỉ đọc pose/twist.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(Odometry, '/wheel/odom', self.cb, 10)

    def cb(self, m: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = m.pose.pose.position.x
        t.transform.translation.y = m.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = m.pose.pose.orientation
        self.br.sendTransform(t)
        m.header.stamp = t.header.stamp
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_footprint'
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(OdomBridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
