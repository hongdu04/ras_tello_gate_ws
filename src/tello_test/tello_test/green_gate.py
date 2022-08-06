#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
import cv2
import sys

class RAS_Tello_GREEN_GATE(Node):
    def __init__(self):
        super().__init__('green_gate')

        self.subscription = self.create_subscription(
            Image,
            '/drone1/image_raw',
            self.image_callback,
            10
        )

        self.publisher_ = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        
    def image_callback(self, msg):
        x_center = 0
        y_center = 0
        rect     = ()

        self.frame = self.imgmsg_to_cv2(msg)

        # convert RGB to HSV
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        # strucruring element
        # line = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15), (-1, -1))
        # HSV
        mask_green  = cv2.inRange(hsv, (35,43,46), (77,255,255))
        mask        = mask_green
        i=1
        # Contour extraction, find the largest contour
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        index = -1
        max   = 0
        for c in range(len(contours)):
            area = cv2.contourArea(contours[c])
            if area > max:
                max   = area
                index = c
        # pic
        if index >= 0:
            rect = cv2.minAreaRect(contours[index])
            # Ellipse Fitting
            cv2.ellipse(self.frame, rect, (0, 255, 0), 2, 8)
            # center point positioning of green gate
            x_center = np.int32(rect[0][0])  # the enter of x
            y_center = np.int32(rect[0][1])  # the center of y
            cv2.circle(self.frame, (np.int32(rect[0][0]), np.int32(rect[0][1])), 2, (0, 255, 0), 2, 8, 0)
            cv2.putText(self.frame, "gate center: ({}, {})".format(x_center, y_center), (x_center + 15, y_center + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # center coordinates of the frame (H, W) = (720, 960)
        cX_frame = 480
        cY_frame = 225
        cv2.circle(self.frame, (cX_frame, cY_frame), 4, (0, 0, 255), -1)
        cv2.putText(self.frame, "camera: ({}, {})".format(cX_frame, cY_frame), (cX_frame - 15, cY_frame - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # align the center of the frame with the center of the ArUco marker
        cX_diff = cX_frame - x_center
        cY_diff = cY_frame - y_center
        cv2.putText(self.frame, "(X error: {}, Y error: {})".format(cX_diff, cY_diff), (cX_frame + 30, cY_frame + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # align X and Y coordinates of the ArUco marker with the center of the frame
        if cX_diff < -5:
            self.tello_rotate_right()
        elif cX_diff > 5:
            self.tello_rotate_left()

        if cY_diff < -5:
            self.tello_move_down()
        elif cY_diff > 5:
            self.tello_move_up()
        
        if x_center in range(cX_frame - 5, cX_frame + 5) and y_center in range(cY_frame - 5, cY_frame + 5):
            while 1:
                for i in range(20):
                    self.tello_move_down()
                for i in range(2000):
                    self.tello_move_forward()


        cv2.imshow("Frame", self.frame)
        cv2.waitKey(1)

    def imgmsg_to_cv2(self, img_msg):
        n_channels = len(img_msg.data) // (img_msg.height * img_msg.width)
        dtype = np.uint8

        img_buf = np.asarray(img_msg.data, dtype=dtype) if isinstance(img_msg.data, list) else img_msg.data

        if n_channels == 1:
            cv2_img = np.ndarray(shape=(img_msg.height, img_msg.width), dtype=dtype, buffer=img_buf)
        else:
            cv2_img = np.ndarray(shape=(img_msg.height, img_msg.width, n_channels), dtype=dtype, buffer=img_buf)

        # If the byte order is different between the message and the system
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            cv2_img = cv2_img.byteswap().newbyteorder()

        return cv2_img
    
    def tello_move_forward(self):
        msg          = Twist()
        msg.linear.x = 0.2
        self.publisher_.publish(msg)
        print("[INFO] Tello move forward")
    
    def tello_move_left(self):
        msg          = Twist()
        msg.linear.y = 0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello move left")
    
    def tello_move_right(self):
        msg          = Twist()
        msg.linear.y = -0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello move right")
    
    def tello_move_down(self):
        msg          = Twist()
        msg.linear.z = -0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello move down")
    
    def tello_move_up(self):
        msg          = Twist()
        msg.linear.z = 0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello move up")
    
    def tello_rotate_left(self):
        msg           = Twist()
        msg.angular.z = 0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello rotate left")
    
    def tello_rotate_right(self):
        msg           = Twist()
        msg.angular.z = -0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello rotate right")

def main(args=None):
    rclpy.init(args=args)
    green_gate = RAS_Tello_GREEN_GATE()
    rclpy.spin(green_gate)
    green_gate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
