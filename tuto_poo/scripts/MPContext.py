#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MPContext:

    def __init__(self):
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self._node.publisher_.publish(msg)
        self._node.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def process(self, args=None):
        rclpy.init(args=args)
        self._node= Node()
        # Create a publisher
        self._publisher= self._node.create_publisher(String, 'topic', 10)
        # Create a timer at 0.5 hertz, with a callback
        self._timer = self._node.create_timer(0.5, self.timer_callback)
        # Go
        rclpy.spin(self._node)
        # Clean stop
        self._node.minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    rosContext= MPContext()
    rosContext.process()