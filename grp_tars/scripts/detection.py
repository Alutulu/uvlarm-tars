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

class Detection(Node):
    def __init__(self, fps=60):
        super().__init__('detection_node')
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.detect_publisher = self.create_publisher(
            String, '/detection', 10)
        self.colorGreen = 50
        self.bouteilleDansChampsVision = False

        self.lo = np.array([self.colorGreen-5, 100, 50])
        self.hi = np.array([self.colorGreen+5, 255, 255])

        self.color_info = (0, 0, 255)

        self.hsv_px = [0, 0, 0]

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
        self.isOk = True
        self.testi = 0

        self.distanceSample = []
        self.currentDistance = -1


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
        return self.depth_image[int(lig)][int(col)]
    
    def getMeanDistance(self, lig=240, col=424, delta=5):
        # résultat en m
        sum = 0
        deltaRange = [-delta, 0, delta]
        for dx in deltaRange:
            for dy in deltaRange:
                sum += self.getDistance(lig+dx, col+dy)
        return round(sum / 9000, 2)

    def read_imgs(self):

        res = self.captureFrame()
        if res == -1:
            return
        self.color_image, self.depth_image = res
        # Dim usually is equal to 480, 848, 3
        color_colormap_dim = self.color_image.shape
        depth_colormap_dim = self.depth_image.shape

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
        #     self.depth_image, alpha=0.15), cv2.COLORMAP_JET)

        self.image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(self.image, self.lo, self.hi)
        self.image = cv2.blur(self.image, (7, 7))
        mask = cv2.erode(mask, self.kernel, iterations=4)
        mask = cv2.dilate(mask, self.kernel, iterations=4)
        image2 = cv2.bitwise_and(self.color_image, self.color_image, mask=mask)
        cv2.putText(self.color_image, "Couleur: {:d}".format(
            self.colorGreen), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
        # Affichage des composantes HSV sous la souris sur l'image
        pixel_hsv = " ".join(str(values) for values in self.hsv_px)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.color_image, "px HSV: "+pixel_hsv, (10, 260),
                    font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        self.findObjects(image2, mask)

        # Show images
        images = np.hstack((self.color_image, image2))
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        # Frequency:
        self.updateFrequency()

    def captureFrame(self):
        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.first(rs.stream.color)
        depth_frame = frames.first(rs.stream.depth)
        if not (depth_frame and color_frame):
            return -1
        return np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data())

    def drawCircle(self, image, x, y, rayon):
        cv2.circle(image, (int(x), int(y)),
            int(rayon), self.color_info, 2)
        
    def labelObject(self, image, x, y, distance):
        cv2.circle(image, (int(x), int(y)),
                    5, self.color_info, 10)
        cv2.line(image, (int(x), int(y)),
                    (int(x)+150, int(y)), self.color_info, 2)
        cv2.putText(image, "Bouteille", (int(x)+10, int(y) - 10),
                    cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
        if self.currentDistance > 0:
            cv2.putText(image, str(self.currentDistance) + " m", (int(x)+20, int(y) + 30),
                    cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
        
    def updateDistance(self, lig=240, col=424):
        # résultat de la distance en mètres
        distance = self.getMeanDistance(lig, col)
        self.distanceSample.append(distance)
        if len(self.distanceSample) >= 50:
            self.currentDistance = round(sum(self.distanceSample) / (len(self.distanceSample)), 2)
            self.distanceSample = []
    
    def findObjects(self, image, mask):
        elements = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c = max(elements, key=cv2.contourArea)
            ((x, y), rayon) = cv2.minEnclosingCircle(c)
            if rayon > 30:
                if not self.bouteilleDansChampsVision:
                    self.publishMessage("Bouteille détectée")
                    self.bouteilleDansChampsVision = True
                # self.updateDistance(y, x)
                self.currentDistance = self.getMeanDistance(y, x)
                self.drawCircle(image, x, y, rayon)
                self.labelObject(self.color_image, x, y, self.currentDistance)
            elif self.bouteilleDansChampsVision:
                self.publishMessage("Bouteille disparue")
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
        self.publish_img(self.color_image, "image", timestamp)

        # Utilisation de colormap sur l'image depth de la Realsense (image convertie en 8-bit par pixel)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            self.depth_image, alpha=0.2), cv2.COLORMAP_JET)

        self.publish_img(depth_colormap, "depth", timestamp)

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
