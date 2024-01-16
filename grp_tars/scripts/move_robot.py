#!/usr/bin/python3
import rclpy
from move_basic import MoveBasic


def main(args=None):
    print("Node MOVE_SIMULATION started")
    rclpy.init(args=args)
    moveSimulationNode = MoveBasic(
        'move_robot_node', 0.6, 0.25, 0.3, 0.5, '/multi/cmd_nav')
    rclpy.spin(moveSimulationNode)
    moveSimulationNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
