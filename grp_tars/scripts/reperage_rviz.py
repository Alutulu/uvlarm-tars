from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
import time


class MoveBasic(Node):

    def __init__(self, node_name, topic_move_name='/multi/cmd_nav'):
        super().__init__(node_name)
        self.position = PoseStamped()
        self.position.pose.position.x = - 1.65
        self.position.pose.position.y = 1.45
        self.position.pose.position.z = -0.0
        self.position.header.frame_id = '/map'
        self.position.header.stamp = self.get_clock().now().to_msg()
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def publishh(self):
            print(self.position.pose.position.x, self.position.pose.position.y)
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
