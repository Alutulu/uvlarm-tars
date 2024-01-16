#!/usr/bin/python3
import rclpy
from move_basic import MoveBasic

def main(args=None):
    print("Node MOVE_SIMULATION started")
    rclpy.init(args=args)
    moveSimulationNode = MoveBasic('move_simulation_node', 0.6, 0.7, 0.3, 0.5, '/cmd_vel')
    rclpy.spin(moveSimulationNode)
    moveSimulationNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
