#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class TestMessagePublisher(Node):
    def __init__(self):
        super().__init__("test_imu")
        self.publisher = self.create_publisher(Imu, "test_imu", 10)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.angle = 0.0
        self.radius = 5.0
        self.timer = self.create_timer(0.1, self.publish_messages)

    def publish_messages(self):
        now = self.get_clock().now().to_msg()

        imu = Imu()
        imu.header.frame_id = "base_link"
        imu.header.stamp = now
        imu.linear_acceleration.x = math.sin(10 * self.angle)
        imu.linear_acceleration.y = math.sin(20 * self.angle)
        imu.linear_acceleration.z = math.sin(40 * self.angle)
        self.publisher.publish(imu)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.radius * math.cos(self.angle)
        transform.transform.translation.y = self.radius * math.sin(self.angle)
        transform.transform.rotation.z = math.sin(self.angle / 2)
        transform.transform.rotation.w = math.cos(self.angle / 2)
        self.transform_broadcaster.sendTransform(transform)

        self.angle += 0.01


def main(args=None):
    rclpy.init(args=args)
    node = TestMessagePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
