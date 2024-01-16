#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import sensor_msgs.msg._point_cloud2 as pc2
import sensor_msgs_py.point_cloud2
from geometry_msgs.msg import Twist
import random

class MoveSimulation(Node):

    def __init__(self):
        super().__init__('move_simulation_node')
        self._initTopics()

        # Detection rectangle sides
        self.LEFT = 2
        self.RIGHT = 1
        self.ALL = 0

        # Basic moves
        self.move_left = self.createMove(0.0, 0.6)
        self.move_right = self.createMove(0.0, -0.6)
        self.move_forward = self.createMove(0.7, 0.6)
        
        # Flags
        self.isGoingForward = False
        self.un_sur_trois = 0

    def _initTopics(self):
        self.subscription = self.create_subscription(
            LaserScan, 'scan',
            self.scan_callback, 50)
        self.cloud_publisher = self.create_publisher(
            pc2.PointCloud2, 'laser_link', 10)
        self.move1_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
    
    def createMove(self, forward, rotation):
        move = Twist()
        move.linear.x = forward
        move.angular.z = rotation
        return move
    
    def _getSampleCloud(self, scanMsg):
        obstacles = []
        angle = scanMsg.angle_min
        for aDistance in scanMsg.ranges:
            if 0.1 < aDistance and aDistance < 5.0:
                aPoint = [
                    math.cos(angle) * aDistance,
                    math.sin(angle) * aDistance
                ]
                obstacles.append(aPoint)
            angle += scanMsg.angle_increment
        sample = [[round(p[0], 2), round(p[1], 2), 0.0] for p in obstacles]
        sampleCloud = sensor_msgs_py.point_cloud2.create_cloud_xyz32(
            Header(frame_id='laser_link'), sample)
        return sample, sampleCloud
    
    def getObstacleNumbers(self, sample):
        obstacles_right = self.detectInRectangle(0.3, 0.5, self.RIGHT, sample)
        obstacles_left = self.detectInRectangle(0.3, 0.5, self.LEFT, sample)
        return len(obstacles_right), len(obstacles_left)

    def rotateLeft(self):
        self.move1_publisher.publish(self.move_left)
        self.isGoingForward = False

    def rotateRight(self):
        self.move1_publisher.publish(self.move_right)
        self.isGoingForward = False

    def moveForward(self):
        self.move1_publisher.publish(self.move_forward)
        self.isGoingForward = True

    def setForwardStraight(self):
        self.move_forward.angular.z = 0.0
        self.un_sur_trois += 1
        self.isGoingForward = True

    def setForwardCurved(self):
        self.move_forward.angular.z = (random.uniform(
                0, 0.1)+0.1)*random.choice([-1, 1])
        self.un_sur_trois = (self.un_sur_trois+1) % 3
        self.isGoingForward = True

    def scan_callback(self, scanMsg):
        sample, sampleCloud = self._getSampleCloud(scanMsg)
        self.cloud_publisher.publish(sampleCloud)
        number_obstacles_right, number_obstacles_left = self.getObstacleNumbers(sample)
        self.decideMove(number_obstacles_right, number_obstacles_left)
    
    def decideMove(self, number_obstacles_right, number_obstacles_left):
        if number_obstacles_right > 0:
            self.rotateLeft()
        elif number_obstacles_left > 0:
            self.rotateRight()
        elif (not self.isGoingForward) and self.un_sur_trois != 0:
            print("là")
            self.setForwardCurved()
        elif not self.isGoingForward:
            self.setForwardStraight()
        else:
            self.moveForward()

    def detectInRectangle(self, dim_x=3, dim_y=2, scan=0, pointcloud=any):
        cloud_obstacle = []
        if scan == self.ALL:
            print("scan all")
            for point in pointcloud:
                if abs(point[1]) <= dim_y/2 and point[0] < dim_x and point[0] >= 0:
                    cloud_obstacle.append(point)
                    print("obstacle trouvé")
            return cloud_obstacle
        elif scan == self.LEFT:
            for point in pointcloud:
                if point[1] <= dim_y/2 and point[1] >= 0 and point[0] < dim_x and point[0] >= 0:
                    cloud_obstacle.append(point)
            return cloud_obstacle
        else:
            for point in pointcloud:
                if point[1] >= -dim_y/2 and point[1] <= 0 and point[0] < dim_x and point[0] >= 0:
                    cloud_obstacle.append(point)
            return cloud_obstacle


def main(args=None):
    print("Node MOVE_SIMULATION started")
    rclpy.init(args=args)
    moveSimulationNode = MoveSimulation()
    rclpy.spin(moveSimulationNode)
    moveSimulationNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
