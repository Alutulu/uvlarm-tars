#!/usr/bin/python3
import rclpy
from rclpy.node import Node

class MSContext(Node):

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def process(self, args):
        rclpy.init(args=args)
        self._node = Node()
        self._node.create_subscription(
            String, 'topic',
            self.listener_callback, 10)
        # Infinite loop:
        rclpy.spin(minimal_subscriber)
        # Clean stop:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

def main():
    minimal_subscriber= MSContext()
    minimal_subscriber.process()