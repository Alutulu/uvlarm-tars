#!/usr/bin/env python3
# Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

# Note importante pour détecter les bouteilles : utiliser les seuils pour détecter le vert des bouteilles en RGB
# --> Comme ça peut varier en fonction de la luminosité, il faut d'abord convertir les canaux RGB en canaux HSV, qui a l'intensité lumineuse en paramètre, avant de faire le seuillage
# --> on obtient alors un masque blanc correspondant à la bouteille, puis s'intéresser à la depth de la bouteille
# --> enlever le "bruit" aussi : il y aura des petits pixels blancs isolés, qu'il faudra "gommer" (surement par érosion)

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

# Realsense Node:
class Realsense(Node):
    def __init__(self, fps=60):
        super().__init__('realsense')
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

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

        # Start streaming
        self.pipeline.start(self.config)

        self.count = 1
        self.refTime = time.process_time()
        self.freq = 60

        self.bridge=CvBridge()

        self.image_publisher = self.create_publisher(Image, 'image_color', 10)
        self.depth_publisher = self.create_publisher(Image, 'image_depth', 10)

        sys.stdout.write("-")

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
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            self.depth_image, alpha=0.15), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = self.color_image.shape

        sys.stdout.write(
            f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(self.freq)} fps)")

        # Show images
        images = np.hstack((self.color_image, depth_colormap))

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

        msg_image = self.bridge.cv2_to_imgmsg(self.color_image,"bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)

        # Utilisation de colormap sur l'image depth de la Realsense (image convertie en 8-bit par pixel)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

        msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap,"bgr8")
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
