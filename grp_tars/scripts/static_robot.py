#!/usr/bin/python3
import rclpy
from move_basic import MoveBasic


def main(args=None):
    print("Node STATIC_ROBOT started")
    rclpy.init(args=args)
    moveStaticNode = MoveBasic(
        'static_robot_node', 0.6, 0.25, 0.3, 0.5, '/multi/cmd_nav', False)
    rclpy.spin(moveStaticNode)
    moveStaticNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
