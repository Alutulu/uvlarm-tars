#!/usr/bin/python3
import rclpy
from move_basic import MoveBasic


def main(args=None):
    print("Node MOVE_ROBOT started")
    rclpy.init(args=args)
    moveRobotNode = MoveBasic(
        'move_robot_node', 0.6, 0.25, 0.3, 0.5, '/multi/cmd_nav')
    rclpy.spin(moveRobotNode)
    moveRobotNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
