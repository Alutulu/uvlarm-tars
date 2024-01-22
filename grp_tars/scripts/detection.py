#!/usr/bin/env python3
import pyrealsense2 as rs
import signal
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from bouteille import Bouteille


class Detection(Node):
    def __init__(self, fps=60):
        super().__init__('detection_node')
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.detect_publisher = self.create_publisher(
            String, '/detection', 10)
        self.colorGreen = 60
        self.deltaColor = 40
        self.sizeDistanceSample = 10

        self.lo = np.array([self.colorGreen-int(self.deltaColor/2), 100, 50])
        self.hi = np.array([self.colorGreen+int(self.deltaColor/2), 255, 255])

        self.color_info = (0, 0, 255)

        self.hsv_px = [0, 0, 0]
        self.number_bouteilles = 0
        self.ratioBouteilleDepth = 115 / 0.3  # mm / meters
        self.deltaSize = 0.3

        # Creating morphological kernel
        self.kernel = np.ones((3, 3), np.uint8)

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(
            self.device.get_info(rs.camera_info.product_line))

        self.checkDevices()

        # init stream caméra avec le format
        self.config.enable_stream(
            rs.stream.color, 848, 480, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

        signal.signal(signal.SIGINT, self.signalInteruption)

        self.count = 1
        self.refTime = time.process_time()
        self.freq = 60

        self.bridge = CvBridge()

        self.image_publisher = self.create_publisher(Image, 'image_color', 10)
        self.depth_publisher = self.create_publisher(Image, 'image_depth', 10)

        # Start streaming
        self.pipeline.start(self.config)
        align_to = rs.stream.depth
        self.align = rs.align(align_to)

        self.isOk = True
        self.testi = 0

        self.distanceSample = []
        self.currentDistance = -1

        self.currentY = 0
        self.ysample = []

    def checkDevices(self):
        print(f"Connect: {self.device_product_line}")
        found_rgb = True
        for s in self.device.sensors:
            print("Name:" + s.get_info(rs.camera_info.name))
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True

        if not (found_rgb):
            print("Depth camera equired !!!")
            exit(0)

    def getDistance(self, lig=240, col=424):
        # résultat en mm
        depth = self.depth_image[int(lig)][int(col)]
        dx, dy, dz = rs.rs2_deproject_pixel_to_point(
            self.color_intrin, [lig, col], depth)
        print(dz, dy)
        return dz, dy - dz/2

    def checkSizeBouteille(self, depth, rayon):
        ratio = rayon / depth
        return (1-self.deltaSize) * self.ratioBouteilleDepth <= ratio and ratio <= (1+self.deltaSize) * self.ratioBouteilleDepth and rayon >= 30

    def getMeanDistance(self, lig=240, col=424, delta=5):
        # résultat en m
        sum_distance = 0
        sum_y = 0
        deltaRange = [-delta, 0, delta]
        for dx in deltaRange:
            for dy in deltaRange:
                distance, y = self.getDistance(lig+dx, col+dy)
                sum_distance += distance
                sum_y += y
        return round(sum_distance / 9000, 2), round(sum_y / 9000, 2)

    def read_imgs(self):

        res = self.captureFrame()
        if res == -1:
            return
        self.color_image, self.depth_image, self.displayed_color_frame, self.color_intrin = res
        # Dim usually is equal to 480, 848, 3
        # color_colormap_dim = self.color_image.shape
        # depth_colormap_dim = self.depth_image.shape

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
        #     self.depth_image, alpha=0.15), cv2.COLORMAP_JET)

        self.image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(self.image, self.lo, self.hi)
        self.image = cv2.blur(self.image, (7, 7))
        mask = cv2.erode(mask, self.kernel, iterations=4)
        mask = cv2.dilate(mask, self.kernel, iterations=4)
        image2 = cv2.bitwise_and(self.color_image, self.color_image, mask=mask)
        cv2.putText(self.displayed_color_frame, "Couleur: {:d}".format(
            self.colorGreen), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
        # Affichage des composantes HSV sous la souris sur l'image
        pixel_hsv = " ".join(str(values) for values in self.hsv_px)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.displayed_color_frame, "px HSV: "+pixel_hsv, (10, 260),
                    font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        self.findObjects(image2, mask, self.displayed_color_frame)

        # Show images
        images = np.hstack((self.displayed_color_frame, image2))
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        # Frequency:
        self.updateFrequency()

    def captureFrame(self):
        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        displayed_color_frame = frames.first(rs.stream.color)
        if not (depth_frame and color_frame):
            return -1

        return np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data()), np.asanyarray(displayed_color_frame.get_data()), color_frame.profile.as_video_stream_profile().intrinsics

    def drawCircle(self, image, x, y, rayon):
        cv2.circle(image, (int(x), int(y)),
                   int(rayon), self.color_info, 2)

    def labelObject(self, image, x, y):
        cv2.circle(image, (int(x), int(y)),
                   5, self.color_info, 10)
        cv2.line(image, (int(x), int(y)),
                 (int(x)+150, int(y)), self.color_info, 2)
        cv2.putText(image, "Bouteille", (int(x) + int(self.deltaColor/2), int(y) - int(self.deltaColor/2)),
                    cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)

    def updateDistance(self, lig=240, col=424):
        # résultat de la distance en mètres
        distance, dy = self.getMeanDistance(lig, col)
        self.distanceSample.append(distance)
        self.ysample.append(dy)
        if len(self.distanceSample) >= self.sizeDistanceSample:
            self.currentDistance = round(
                sum(self.distanceSample) / (len(self.distanceSample)), 2)
            self.currentY = round(
                sum(self.ysample) / (len(self.ysample)), 2)
            self.publishMessage(
                "bouteille " + str(self.currentDistance) + " " + str(self.currentY))
            self.distanceSample = []
            self.ysample = []

    def findObjects(self, image, mask, displayed_image):
        elements = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            # if len(elements) > 4:
            #     elements = elements[:4]
            for circle in elements:
                ((x, y), rayon) = cv2.minclosingCircle(circle)
                if rayon >= 30:
                    self.publishMessage(
                        "bouteille " + str(self.currentDistance) + " " + str(self.currentY))
                # if self.checkSizeBouteille(self.currentDistance, rayon):
                self.drawCircle(image, x, y, rayon)
                self.labelObject(displayed_image, x, y)
            else:
                self.publishMessage("pas de bouteille")
                self.bouteilleDansChampsVision = False

    def updateFrequency(self):
        if self.count == 10:
            newTime = time.process_time()
            self.freq = 10/((newTime-self.refTime))
            self.refTime = newTime
            self.count = 0
        self.count += 1

    def publishMessage(self, message):
        str_message = String()
        str_message.data = message
        self.detect_publisher.publish(str_message)

    def publish_imgs(self):

        timestamp = self.get_clock().now().to_msg()
        self.publish_img(self.displayed_color_frame, "image_color", timestamp)

        # Utilisation de colormap sur l'image depth de la Realsense (image convertie en 8-bit par pixel)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
        #     self.depth_image, alpha=0.2), cv2.COLORMAP_JET)

        # self.publish_img(depth_colormap, "depth", timestamp)

    def publish_img(self, image, frame_id, timestamp):
        msg_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        msg_image.header.stamp = timestamp
        msg_image.header.frame_id = frame_id
        self.image_publisher.publish(msg_image)

    def signalInteruption(self, signum, frame):
        print("\nCtrl-c pressed")
        self.isOk = False


def main(args=None):
    print("Node DETECTION started")
    rclpy.init(args=args)
    detectNode = Detection()
    while detectNode.isOk:
        detectNode.read_imgs()
        detectNode.publish_imgs()
        rclpy.spin_once(detectNode, timeout_sec=0.001)
    print("\nDestroying DETECTION NODE...")
    detectNode.pipeline.stop()
    detectNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
