#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import sensor_msgs.msg._point_cloud2 as pc2
import sensor_msgs_py.point_cloud2

class reactive_node(Node):

    def __init__(self):
        super().__init__('reactive_node')
        self.subscription= self.create_subscription(
            LaserScan, 'scan',
            self.scan_callback, 10)
        self.cloud_publisher = self.create_publisher(pc2.PointCloud2, 'laser_link', 10)

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
        sampleCloud = sensor_msgs_py.point_cloud2.create_cloud_xyz32(Header(frame_id='laser_link'), sample)

        for point in sensor_msgs_py.point_cloud2.read_points(sampleCloud):
            print(point)

        self.cloud_publisher.publish(sampleCloud)

def main(args=None):
    print("Node REACTIVE_MOVE started")
    rclpy.init(args=args)
    reactive_node = reactive_node()
    rclpy.spin(reactive_node)
    reactive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()