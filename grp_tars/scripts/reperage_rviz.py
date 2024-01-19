from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
import time


class MoveBasic(Node):

    def __init__(self, node_name, topic_move_name='/abracadabra'):
        super().__init__(node_name)
        self.i = 0


        self.position=PointStamped()
        self.position.point.x=-3.5620377
        self.position.point.y=-6.8689
        self.position.point.z=-0.00032126060055376326
        self.position.header.frame_id = 'map'
        self.position.header.stamp = self.get_clock().now().to_msg()
        self.pub = self.create_publisher(PointStamped, topic_move_name, 10)

    def publishh(self):
        while True:
            print(self.position.point.x, self.position.point.y)
            self.pub.publish(self.position)
            time.sleep(1)


def main(args=None):
    print("Node DETECTION started")
    isOk = True  # Capture ctrl-c event
    rclpy.init(args=args)
    publishhh = MoveBasic("vbhjv")

    while isOk:
        publishhh.publishh()


if __name__ == '__main__':
    main()
