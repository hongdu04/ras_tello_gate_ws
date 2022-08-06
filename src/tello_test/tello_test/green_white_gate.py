#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
import cv2
import sys
import imutils
import matplotlib.pyplot as plt

class RAS_Tello_GREEN_GATE(Node):
    def __init__(self):
        super().__init__('green_gate')

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def image_callback(self, msg):

        self.frame = self.imgmsg_to_cv2(msg)

        hsv = cv2.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        # green
        lower_boundary1 = [40, 80, 60]
        upper_boundary1 = [60, 200, 255]
        lower1 = np.array(lower_boundary1, dtype="uint8")
        upper1 = np.array(upper_boundary1, dtype="uint8")
        mask = cv2.inRange(hsv, lower1, upper1)

        color_detected = cv2.bitwise_and(self.frame, self.frame, mask=mask)

        # detected contours and all color points
        gray = cv2.cvtColor(color_detected, cv2.COLOR_BGR2GRAY)

        cnt = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = imutils.grab_contours(cnt)
        # Splicing all detected contours together into a point set
        cnt = np.concatenate(cnt, axis=0)
        # 3d to 2d
        cnts = cnt[:,0,:]

        if cnts.any():
            # Contour stitching, minimum rectangle estimation, means to draw a minimum rectangle that can include all points
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(self.frame, (x, y), (x+w, y+h), (0, 255 ,0),8)
            # drow contours
            cv2.drawContours(self.frame, cnt, -1, (155, 255, 0), 8)
            gate_center=(int(x+(w/2)),int(y+(h/2)))
            # drow the center
            cv2.circle(self.frame ,gate_center, 7, (0, 255, 0), -1)
        
        x_center = gate_center[0]
        y_center = gate_center[1]
        
        # center coordinates of the frame (H, W) = (720, 960)
        cX_frame = 240
        cY_frame = self.frame.shape[0] // 2
        cv2.circle(self.frame, (cX_frame, cY_frame), 4, (0, 0, 255), -1)
        cv2.putText(self.frame, "camera: ({}, {})".format(cX_frame, cY_frame), (cX_frame - 15, cY_frame - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # align X and Y coordinates of the ArUco marker with the center of the frame 
        if y_center < (cY_frame - 10):
            self.tello_move_up()
        elif y_center > (cY_frame + 10):
            self.tello_move_down()
        # else:
        #     self.tello_vertical_stop()

        if x_center > (cX_frame - 10):
            self.tello_rotate_right()
        elif x_center < (cX_frame + 10):
            self.tello_rotate_left()
        # else:
        #     self.tello_horizontal_stop()
        
        if x_center in range(cX_frame - 10, cX_frame + 10) and y_center in range(cY_frame - 10, cY_frame + 10):
            while 1:
                self.tello_move_forward()

        cv2.imshow("Frame", self.frame[:, :, ::-1])
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
        msg = Twist()
        msg.linear.x = 0.2
        self.publisher_.publish(msg)
        print("[INFO] move forward")
    
    def tello_move_left(self):
        msg = Twist()
        msg.linear.y = 0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello move left")
    
    def tello_move_right(self):
        msg = Twist()
        msg.linear.y = -0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello move right")
    
    def tello_move_down(self):
        msg = Twist()
        msg.linear.z = -0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello move down")
    
    def tello_move_up(self):
        msg = Twist()
        msg.linear.z = 0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello move up")
    
    def tello_rotate_left(self):
        msg = Twist()
        msg.angular.z = 0.1
        self.publisher_.publish(msg)
        print("[INFO] Tello rotate left")
    
    def tello_rotate_right(self):
        msg = Twist()
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
