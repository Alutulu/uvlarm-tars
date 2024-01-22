#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import sensor_msgs.msg._point_cloud2 as pc2
import sensor_msgs_py.point_cloud2
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import random
from kobuki_ros_interfaces.msg import WheelDropEvent, ButtonEvent
import time
import numpy as np
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import String
import math
from geometry_msgs.msg import PointStamped
from bouteille import Bouteille


# tenir compte du nombre de bouteilles dans le champ de vision pour savoir si il y en a qui sont apparues ou non


class MoveBasic(Node):

    def __init__(self, node_name, default_rotation_speed=0.6, default_forward_speed=0.25, default_detection_depth=0.3, default_detection_width=0.5, default_topic_move_name='/multi/cmd_nav', default_can_move=True):
        super().__init__(node_name)

        # Load parameters
        self.declare_parameter('rotation_speed', default_rotation_speed)
        rotation_speed = self.get_parameter(
            'rotation_speed').get_parameter_value().double_value
        self.declare_parameter('forward_speed', default_forward_speed)
        forward_speed = self.get_parameter(
            'forward_speed').get_parameter_value().double_value
        self.declare_parameter('detection_depth', default_detection_depth)
        detection_depth = self.get_parameter(
            'detection_depth').get_parameter_value().double_value
        self.declare_parameter('detection_width', default_detection_width)
        detection_width = self.get_parameter(
            'detection_width').get_parameter_value().double_value
        self.declare_parameter('topic_move_name', default_topic_move_name)
        topic_move_name = self.get_parameter(
            'topic_move_name').get_parameter_value().string_value
        self.declare_parameter('can_move', default_can_move)
        can_move = self.get_parameter(
            'can_move').get_parameter_value().bool_value

        self._initTopics(topic_move_name)

        # Detection rectangle sides
        self.LEFT = 2
        self.RIGHT = 1
        self.ALL = 0

        # Basic moves
        self.move_left = self.createMove(0.0, rotation_speed)
        self.move_turn = self.createMove(0.0, math.pi/1.5)

        self.move_right = self.createMove(0.0, -rotation_speed)
        self.move_forward = self.createMove(forward_speed, 0.0)
        self.move_stop = self.createMove(0.0, 0.0)
        self.can_move = can_move

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

        # Position
        self.x = 0
        self.y = 0
        self.angle = 0

        # origine du SLAM par rapport à la MAP
        self.x_origine = 0
        self.y_origine = 0
        self.width = 0
        self.height = 0
        self.resolution = 0

        # Goal parameter:
        self.post_goal = PoseStamped()
        self.post_goal.header.stamp = self.get_clock().now().to_msg()
        self.post_goal.header.frame_id = '/map'
        self.post_goal.pose.position.x = 0
        self.post_goal.pose.position.y = 0

        # Bouteilles
        self.bouteilles = []
        self.margin = 0.15

        self.contour_fait = False

    def euler_from_quaternion(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def coordBouteilleRelative(self, distance, dy):
        # x = distance * math.cos(self.angle)
        # y = distance * math.sin(self.angle) + dy
        x = distance * math.cos(self.angle) + dy * math.sin(self.angle)
        y = distance * math.sin(self.angle) + dy * math.cos(self.angle)
        return x, y

    def coordBouteilleAbsolute(self, distance, dy):
        x, y = self.coordBouteilleRelative(distance, dy)
        return x + self.x, y + self.y

    def _initTopics(self, topic_move_name):
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 50)
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map',
            self.map_callback, 50)
        self.detection_subscription = self.create_subscription(
            String, '/detection',
            self.detection_callback, 50)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, 50)
        self.cloud_publisher = self.create_publisher(
            pc2.PointCloud2, 'laser_link', 10)
        self.move1_publisher = self.create_publisher(
            Twist, topic_move_name, 10)
        self.goal_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        self.wheel_drop_subscription = self.create_subscription(
            WheelDropEvent, '/events/wheel_drop', self.wheel_drop_callback, 50)
        self.button_subscriber = self.create_subscription(
            ButtonEvent, '/events/button', self.button_callback, 50)
        self.pointPublisher = self.create_publisher(
            PointStamped, '/points_bouteilles', 10)

    def map_callback(self, grid_msg):
        self.x_origine = grid_msg.info.origin.position.x
        self.y_origine = grid_msg.info.origin.position.y
        self.width = grid_msg.info.width
        self.height = grid_msg.info.height
        self.resolution = grid_msg.info.resolution
        self.data = grid_msg.data

    def coordonee_pixel(self, rang):
        x = rang % self.width
        y = rang//self.height
        x_pixel = x*self.resolution+self.x_origine
        y_pixel = y*self.resolution+self.y_origine
        return x_pixel, y_pixel

    def orienter_vers_angle_deg(self, angle_deg, orientation=0):
        if orientation == 0:
            self.move1_publisher.publish(self.move_left)
        else:
            self.move1_publisher.publish(self.move_right)
        angle_rad = angle_deg*2*math.pi/360
        if angle_deg > 179.98 or angle_deg < -179.98:
            while (self.angle > 179.98):
                continue
        else:
            while (self.angle >= angle_rad-0.01 and self.angle <= angle_rad+0.01):
                continue
        self.move1_publisher.publish(self.move_stop)

    def verification_bord_arene(self, aire_obstacle):
        k = 0
        interieur = 0
        exterieur = 0
        for i in self.data:
            if i[0] == 0:
                x_verif, y_verif = self.coordonee_pixel(k)
                if x_verif >= aire_obstacle[0] and x_verif <= aire_obstacle[1] and y_verif >= aire_obstacle[2] and y_verif >= aire_obstacle[3]:
                    interieur += 1
                else:
                    exterieur += 1
        if interieur > exterieur:
            return True
        else:
            return False

    def t360(self):

        self.move1_publisher.publish(self.move_turn)
        time.sleep(3)
        self.move1_publisher.publish(self.move_stop)

    def goal_pose(self, x, y):
        self.post_goal.pose.position.x = x
        self.post_goal.pose.position.y = y
        self.goal_publisher.publish(self.post_goal)
        self.pointPublisher = self.create_publisher(
            PointStamped, '/points_bouteilles', 10)

    def detection_callback(self, detect_msg):
        if len(detect_msg.data.split()) == 3:
            first_word, second_word, third_word = detect_msg.data.split()
            if first_word == 'bouteille':
                distance = float(second_word)
                dy = float(third_word)
                x, y = self.coordBouteilleAbsolute(distance, dy)
                bouteille = Bouteille(x, y)
                if not bouteille.alreadyIn(self.bouteilles):
                    self.stopMove()
                    self.bouteilles.append(bouteille)
                else:
                    for bottle in self.bouteilles:
                        if bouteille.sameAs(bottle):
                            if not bottle.placed:
                                xbouteille, ybouteille = bouteille.getPosition()
                                bottle.addtoSample(xbouteille, ybouteille)
                            break
            else:
                print(detect_msg.data)

        self.placeBottles()
        self.markBottles()

    def markBottles(self):
        for bouteille in self.bouteilles:
            if bouteille.needToBeMarked:
                self.publishMarker(bouteille.x, bouteille.y)
                bouteille.needToBePlaced = False

    def placeBottles(self):
        for bouteille in self.bouteilles:
            if bouteille.numberSeen() >= 10:
                bouteille.placeBottle()

    def odom_callback(self, odom_msg):
        msg = odom_msg.pose.pose
        self.x = msg.position.x
        self.y = msg.position.y
        self.angle = self.euler_from_quaternion(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

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

    def publishMarker(self, x, y):
        bouteille = Bouteille('b' + str(len(self.bouteilles)), x, y)
        if not bouteille.alreadyIn(self.bouteilles, self.margin):
            self.bouteilles.append(bouteille)
            self.position = PointStamped()
            self.position.point.x = x
            self.position.point.y = y
            self.position.point.z = 0.0
            self.position.header.frame_id = 'map'
            self.position.header.stamp = self.get_clock().now().to_msg()
            self.pointPublisher.publish(self.position)   # if self.can_move:
        #     self.decideMove(number_obstacles_right, number_obstacles_left)

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
        # while self.contour_fait == False:
        #     k = self.data.find(1)
        #     if k == -1:
        #         self.decideMove(number_obstacles_right, number_obstacles_left)
        if self.can_move:
            self.decideMove(number_obstacles_right, number_obstacles_left)

    def decideMove(self, number_obstacles_right, number_obstacles_left):
        k = random.randint(10000)
        if k == 1:
            self.t360()
        else:
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
