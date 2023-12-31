#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import sensor_msgs.msg._point_cloud2 as pc2
import sensor_msgs_py.point_cloud2
from geometry_msgs.msg import Twist
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
        self.move_left.angular.z = 0.3  # radian per second
        self.move_right = Twist()
        self.move_right.linear.x = 0.0  # meter per second
        self.move_right.angular.z = -0.3  # radian per second
        self.move_null = Twist()
        self.move_null.linear.x = 0.5  # meter per second
        self.move_null.angular.z = 0.0  # radian per second

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
        obstacles_right = self.detectInRectangle(0.3, 0.7, self.right, sample)
        obstacles_left = self.detectInRectangle(0.3, 0.7, self.left, sample)
        print("right :", len(obstacles_right),
              " | left :", len(obstacles_left))
        if len(obstacles_right) > 0:
            self.move1_publisher.publish(self.move_left)
        elif len(obstacles_left) > 0:
            self.move1_publisher.publish(self.move_right)
        else:
            self.move1_publisher.publish(self.move_null)
            # except:
            #     print("erreur ", type(sample))

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
