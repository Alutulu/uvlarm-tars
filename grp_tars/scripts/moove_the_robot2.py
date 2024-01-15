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
# from std_msgs.msg import String
# test


class reactive_move(Node):

    def __init__(self):
        super().__init__('reactive_node')
        self.subscription = self.create_subscription(
            LaserScan, 'scan',
            self.scan_callback, 50)
        self.cloud_publisher = self.create_publisher(
            pc2.PointCloud2, 'laser_link', 10)
        self.left = 2
        self.right = 1
        self.all = 0
        self.move1_publisher = self.create_publisher(
            Twist, '/multi/cmd_nav', 10)
        self.move_left = Twist()
        self.move_left.linear.x = 0.0  # meter per second
        self.move_left.angular.z = 0.6  # radian per second
        self.move_right = Twist()
        self.move_right.linear.x = 0.0  # meter per second
        self.move_right.angular.z = -0.6  # radian per second
        self.move_null = Twist()
        self.move_null.linear.x = 0.4  # meter per second
        self.move_null.angular.z = 0.0  # radian per second
        self.parametre = 0
        self.un_sur_trois = 0
        # self.detect_publisher = self.create_publisher(
        #     String, '/detection', 10)
        self.bloque = 0
        self.changement_de_tour_droite = False
        self.changement_de_tour_gauche = False

    def scan_callback(self, scanMsg):
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

        # for point in sensor_msgs_py.point_cloud2.read_points(sampleCloud):
        #     print(point)

        self.cloud_publisher.publish(sampleCloud)
       # try:
        obstacles_right = self.detectInRectangle(0.3, 0.5, self.right, sample)
        obstacles_left = self.detectInRectangle(0.3, 0.5, self.left, sample)
        print("right :", len(obstacles_right),
              " | left :", len(obstacles_left))
        if self.changement_de_tour_gauche and self.changement_de_tour_droite and (len(obstacles_left) > 0 or len(obstacles_right)) > 0:
            self.move1_publisher.publish(self.move_left)
        elif len(obstacles_right) > 0:
            self.move1_publisher.publish(self.move_left)
            self.parametre = 0
            self.changement_de_tour_gauche = True

        elif len(obstacles_left) > 0:
            self.move1_publisher.publish(self.move_right)
            self.parametre = 0
            self.changement_de_tour_droite = True

        elif self.parametre == 0 and self.un_sur_trois != 0:
            self.move_null.angular.z = (random.uniform(
                0, 0.3)+0.3)*random.choice([-1, 1])
            self.parametre = 1
            self.un_sur_trois = (self.un_sur_trois+1) % 3
            self.changement_de_tour_droite = False
            self.changement_de_tour_gauche = False
        elif self.parametre == 0:
            self.move_null.angular.z = 0.0

            self.move1_publisher.publish(self.move_null)
            self.parametre = 1
            self.un_sur_trois += 1
            self.changement_de_tour_droite = False
            self.changement_de_tour_gauche = False
        else:

            self.move1_publisher.publish(self.move_null)

    def detectInRectangle(self, dim_x=3, dim_y=2, scan=0, pointcloud=any):

        cloud_obstacle = []
        if scan == self.all:
            print("scan all")
            for point in pointcloud:
                # print("elt de pointCloud", point)
                if abs(point[1]) <= dim_y/2 and point[0] < dim_x and point[0] >= 0:
                    cloud_obstacle.append(point)
                    print("obstacle trouvé")
            return cloud_obstacle
        elif scan == self.left:
            # print("droite")
            for point in pointcloud:
                # print("elt de pointCloud", point)
                if point[1] <= dim_y/2 and point[1] >= 0 and point[0] < dim_x and point[0] >= 0:
                    cloud_obstacle.append(point)
            return cloud_obstacle
        else:
            # print("droite")
            for point in pointcloud:
                # print(point[0], point[1])
                # print("elt de pointCloud", point)
                if point[1] >= -dim_y/2 and point[1] <= 0 and point[0] < dim_x and point[0] >= 0:
                    cloud_obstacle.append(point)
            return cloud_obstacle


def main(args=None):
    print("Node REACTIVE_MOVE started")
    rclpy.init(args=args)
    reactive_node = reactive_move()
    rclpy.spin(reactive_node)
    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
