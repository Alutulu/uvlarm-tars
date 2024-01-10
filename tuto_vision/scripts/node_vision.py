#!/usr/bin/env python3
# Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import signal
import time
import numpy as np
import sys
import cv2
import rclpy
from rclpy.node import Node

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

        sys.stdout.write("-")

    def read_imgs(self):
        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.first(rs.stream.color)
        depth_frame = frames.first(rs.stream.depth)

        if not (depth_frame and color_frame):
            return

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            depth_image, alpha=0.15), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        sys.stdout.write(
            f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(self.freq)} fps)")

        # Show images
        images = np.hstack((color_image, depth_colormap))

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
        pass

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
