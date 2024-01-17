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
from kobuki_ros_interfaces.msg import WheelDropEvent, ButtonEvent
import time


class MoveBasic(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        # Load parameters
        self.declare_parameter('topic_move_name')
        topic_move_name = self.get_parameter('topic_move_name').get_parameter_value().string_value
        self.declare_parameter('rotation_speed')
        rotation_speed = self.get_parameter('rotation_speed').get_parameter_value().double_value
        self.declare_parameter('forward_speed')
        forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.declare_parameter('detection_depth')
        detection_depth = self.get_parameter('detection_depth').get_parameter_value().double_value
        self.declare_parameter('detection_width')
        detection_width = self.get_parameter('detection_width').get_parameter_value().double_value

        self._initTopics(topic_move_name)

        # Detection rectangle sides
        self.LEFT = 2
        self.RIGHT = 1
        self.ALL = 0

        # Basic moves
        self.move_left = self.createMove(0.0, rotation_speed)
        self.move_right = self.createMove(0.0, -rotation_speed)
        self.move_forward = self.createMove(forward_speed, 0.0)
        self.move_stop = self.createMove(0.0, 0.0)

        # Detection parameters
        self.detection_depth = detection_depth
        self.detection_width = detection_width

        # Status
        self.isGoingForward = False
        self.un_sur_trois = 0
        self.aTourneADroite = False
        self.aTourneAGauche = False
        self.leftWheelDroped = False
        self.rightWheelDroped = False
        self.stopped = False

    def _initTopics(self, topic_move_name):
        self.subscription = self.create_subscription(
            LaserScan, 'scan',
            self.scan_callback, 50)
        self.cloud_publisher = self.create_publisher(
            pc2.PointCloud2, 'laser_link', 10)
        self.move1_publisher = self.create_publisher(
            Twist, topic_move_name, 10)
        self.wheel_drop_publisher = self.create_subscription(
            WheelDropEvent, '/events/wheel_drop', self.wheel_drop_callback, 50)
        self.wheel_drop_publisher = self.create_subscription(
            ButtonEvent, '/events/button', self.button_callback, 50)

    def wheel_drop_callback(self, wheel_msg):
        rightWheel = wheel_msg.wheel == 1
        droped = wheel_msg.state == 1
        if rightWheel and droped:
            self.rightWheelDroped = True
        elif rightWheel and not droped:
            self.rightWheelDroped = False
        elif not rightWheel and droped:
            self.leftWheelDroped = True
        elif not rightWheel and not droped:
            self.leftWheelDroped = False
        if self.robotUp():
            self.stopped = True
            self.stopMove()
        elif self.robotDown() and self.stopped:
            time.sleep(5)
            self.stopped = False

    def button_callback(self, button_msg):
        if button_msg.state == 1 and not self.stopped:
            self.stopped = True
            self.stopMove()
        elif button_msg.state == 1 and self.stopped:
            self.stopped = False

    def robotUp(self):
        return self.leftWheelDroped and self.rightWheelDroped

    def robotDown(self):
        return not self.leftWheelDroped and not self.rightWheelDroped

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
        obstacles_right = self.detectInRectangle(
            self.detection_depth, self.detection_width, self.RIGHT, sample)
        obstacles_left = self.detectInRectangle(0.3, 0.5, self.LEFT, sample)
        return len(obstacles_right), len(obstacles_left)

    def rotateLeft(self):
        self.move1_publisher.publish(self.move_left)
        self.isGoingForward = False
        self.aTourneAGauche = True

    def rotateRight(self):
        self.move1_publisher.publish(self.move_right)
        self.isGoingForward = False
        self.aTourneADroite = True

    def isBloque(self, number_obstacles_right, number_obstacles_left):
        return self.aTourneADroite and self.aTourneAGauche and (number_obstacles_left > 0 or number_obstacles_right > 0)

    def moveForward(self):
        self.move1_publisher.publish(self.move_forward)
        self.isGoingForward = True
        self.aTourneADroite = False
        self.aTourneAGauche = False

    def setForwardStraight(self):
        self.move_forward.angular.z = 0.0
        self.un_sur_trois += 1
        self.isGoingForward = True

    def setForwardCurved(self):
        self.move_forward.angular.z = (random.uniform(
            0, 0.1)+0.1)*random.choice([-1, 1])
        self.un_sur_trois = (self.un_sur_trois+1) % 3
        self.isGoingForward = True

    def stopMove(self):
        self.move1_publisher.publish(self.move_stop)

    def scan_callback(self, scanMsg):
        sample, sampleCloud = self._getSampleCloud(scanMsg)
        self.cloud_publisher.publish(sampleCloud)
        number_obstacles_right, number_obstacles_left = self.getObstacleNumbers(
            sample)
        self.decideMove(number_obstacles_right, number_obstacles_left)

    def decideMove(self, number_obstacles_right, number_obstacles_left):
        if not self.stopped:
            # si bloqué dans un coin
            if self.isBloque(number_obstacles_right, number_obstacles_left):
                self.rotateLeft()
            elif number_obstacles_right > 0:
                self.rotateLeft()
            elif number_obstacles_left > 0:
                self.rotateRight()
            # 2 fois sur 3 avant de commencer à avancer
            elif (not self.isGoingForward) and self.un_sur_trois != 0:
                self.setForwardCurved()
            # 1 fois sur 3 avant de commencer à avancer
            elif not self.isGoingForward:
                self.setForwardStraight()
            # si peut avancer
            else:
                self.moveForward()

    def detectInRectangle(self, dim_x, dim_y, scan=0, pointcloud=any):
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
