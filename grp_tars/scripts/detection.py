#!/usr/bin/env python3
import pyrealsense2 as rs
import signal
import time
import numpy as np
import sys
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class Realsense(Node):
    def __init__(self, fps=60):
        super().__init__('detection_node')
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.detect_publisher = self.create_publisher(
            String, '/detection', 10)
        self.color = 50
        self.k = 0

        self.lo = np.array([self.color-5, 100, 50])
        self.hi = np.array([self.color+5, 255, 255])

        self.color_info = (0, 0, 255)

        self.cap = cv2.VideoCapture(0)
        cv2.namedWindow('Camera')
        cv2.setMouseCallback('Camera', self.souris)
        self.hsv_px = [0, 0, 0]

        # Creating morphological kernel
        self.kernel = np.ones((3, 3), np.uint8)

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(
            self.device.get_info(rs.camera_info.product_line))

        print(f"Connect: {self.device_product_line}")
        found_rgb = True  # ligne 27 à 35 vérification que tous les supports fonctionnent
        for s in self.device.sensors:
            print("Name:" + s.get_info(rs.camera_info.name))
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True

        if not (found_rgb):
            print("Depth camera equired !!!")
            exit(0)

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

        self.config.enable_stream(
            rs.stream.infrared, 1, 848, 480, rs.format.y8, 60)
        self.config.enable_stream(
            rs.stream.infrared, 2, 848, 480, rs.format.y8, 60)
        # Start streaming
        self.pipeline.start(self.config)

        self.infra_publisher_1 = self.create_publisher(Image, 'infrared_1', 10)
        self.infra_publisher_2 = self.create_publisher(Image, 'infrared_2', 10)

        sys.stdout.write("-")

    def souris(self, event, x, y, flags, param):

        if event == cv2.EVENT_MOUSEMOVE:
            # Conversion des trois couleurs RGB sous la souris en HSV
            px = self.color_image[y, x]
            px_array = np.uint8([[px]])
            self.hsv_px = cv2.cvtColor(px_array, cv2.COLOR_BGR2HSV)

        if event == cv2.EVENT_MBUTTONDBLCLK:
            self.color = self.image[y, x][0]

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.color > 5:
                self.color -= 1

        if event == cv2.EVENT_RBUTTONDOWN:
            if self.color < 250:
                self.color += 1

        self.lo[0] = self.color-10
        self.hi[0] = self.color+10

    def read_imgs(self):
        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.first(rs.stream.color)
        depth_frame = frames.first(rs.stream.depth)

        if not (depth_frame and color_frame):
            return

        # Convert images to numpy arrays
        self.depth_image = np.asanyarray(depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
        #     self.depth_image, alpha=0.15), cv2.COLORMAP_JET)

        # depth_colormap_dim = depth_colormap.shape
        # color_colormap_dim = self.color_image.shape

        # infra_frame_1 = frames.get_infrared_frame(1)
        # infra_frame_2 = frames.get_infrared_frame(2)
        # infra_image_1 = np.asanyarray(infra_frame_1.get_data())
        # infra_image_2 = np.asanyarray(infra_frame_2.get_data())

        # Utilisation de colormap sur l'image infrared de la Realsense (image convertie en 8-bit par pixel)
        # self.infra_colormap_1 = cv2.applyColorMap(
        #     cv2.convertScaleAbs(infra_image_1, alpha=1), cv2.COLORMAP_JET)

        # # Utilisation de colormap sur l'image infrared de la Realsense (image convertie en 8-bit par pixel)
        # self.infra_colormap_2 = cv2.applyColorMap(
        #     cv2.convertScaleAbs(infra_image_2, alpha=1), cv2.COLORMAP_JET)

        # sys.stdout.write(
        #     f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(self.freq)} fps)")

        self.image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(self.image, self.lo, self.hi)
        self.image = cv2.blur(self.image, (7, 7))
        mask = cv2.erode(mask, self.kernel, iterations=4)
        mask = cv2.dilate(mask, self.kernel, iterations=4)
        image2 = cv2.bitwise_and(self.color_image, self.color_image, mask=mask)
        cv2.putText(self.color_image, "Couleur: {:d}".format(
            self.color), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)

        # Affichage des composantes HSV sous la souris sur l'image
        pixel_hsv = " ".join(str(values) for values in self.hsv_px)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.color_image, "px HSV: "+pixel_hsv, (10, 260),
                    font, 1, (255, 255, 255), 1, cv2.LINE_AA)

        elements = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c = max(elements, key=cv2.contourArea)
            ((x, y), rayon) = cv2.minEnclosingCircle(c)
            if rayon > 30:
                if self.k == 0:

                    message = String()
                    message.data = "bouteille detecté"
                    self.detect_publisher.publish(message)
                    self.k = 1
                cv2.circle(image2, (int(x), int(y)),
                           int(rayon), self.color_info, 2)
                cv2.circle(self.color_image, (int(x), int(y)),
                           5, self.color_info, 10)
                cv2.line(self.color_image, (int(x), int(y)),
                         (int(x)+150, int(y)), self.color_info, 2)
                cv2.putText(self.color_image, "Objet !!!", (int(x)+10, int(y) - 10),
                            cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
            elif self.k == 1:
                message2 = String()
                message2.data = "Bouteille disparue"
                self.detect_publisher.publish(message2)
                self.k = 0

        # Show images
        images = np.hstack((self.color_image, image2))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        # Frequency:
        if self.count == 10:
            newTime = time.process_time()
            self.freq = 10/((newTime-self.refTime))
            self.refTime = newTime
            self.count = 0
        self.count += 1

    def publish_imgs(self):

        msg_image = self.bridge.cv2_to_imgmsg(self.color_image, "bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)

        # Utilisation de colormap sur l'image depth de la Realsense (image convertie en 8-bit par pixel)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            self.depth_image, alpha=0.2), cv2.COLORMAP_JET)

        msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")
        msg_depth.header.stamp = msg_image.header.stamp
        msg_depth.header.frame_id = "depth"
        self.depth_publisher.publish(msg_depth)

    def signalInteruption(self, signum, frame):
        global isOk
        print("\nCtrl-c pressed")
        isOk = False


# Capture ctrl-c event
isOk = True

# Node processes:


def process_img(args=None):
    rclpy.init(args=args)
    rsNode = Realsense()
    while isOk:
        rsNode.read_imgs()
        rsNode.publish_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.001)
    # Stop streaming
    print("\nEnding...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()


process_img()
