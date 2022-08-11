#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
import argparse
import cv2
import sys

class RAS_Tello_ARUCO_GATE(Node):
    def __init__(self):
        super().__init__('aruco_gate')

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cX1 = 0
        self.cY1 = 0
        self.cX2 = 0
        self.cY2 = 0
        self.cX3 = 0
        self.cY3 = 0
        self.cX4 = 0
        self.cY4 = 0
        
    def image_callback(self, msg):
        self.ap = argparse.ArgumentParser()
        self.ap.add_argument("-t", "--type", type=str, default="DICT_4X4_100", help="type of ArUCo tag to detect")
        self.args = vars(self.ap.parse_args())

        self.ARUCO_DICT = {
            "DICT_4X4_50"   : cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100"  : cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250"  : cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000" : cv2.aruco.DICT_4X4_1000
        }

        self.desired_aruco_dictionary = "DICT_4X4_100"
        print("[INFO] loading ...")

        if self.ARUCO_DICT.get(self.args["type"], None) is None:
            print("[INFO] ArUCo tag of '{}' is not supported".format(self.args["type"]))
            sys.exit(0)

        print("[INFO] detecting '{}' tags...".format(self.args["type"]))
        self.arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.args["type"]])
        self.arucoParams = cv2.aruco.DetectorParameters_create()

        print("[INFO] starting video stream...")

        self.frame = self.imgmsg_to_cv2(msg)

        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.frame, self.arucoDict, parameters=self.arucoParams)

        markerIDList = []

        if len(corners) > 0:
            
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight    = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft  = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft     = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box of the ArUCo detection
                cv2.line(self.frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(self.frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(self.frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(self.frame, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(self.frame, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                markerIDList.append(markerID)
                print("[INFO] ArUco marker ID: {}".format(markerID))
                cv2.putText(self.frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # show the coordinates of the center of the ArUco marker
                cv2.putText(self.frame, "({}, {})".format(cX, cY), (topRight[0], topRight[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # calculate the center coordinates of 4 ArUco markers
                if markerID == 1:
                    self.cX1 = cX
                    self.cY1 = cY
                    print("[INFO] ArUco marker 1: ({}, {})".format(cX, cY))
                if markerID == 2:
                    self.cX2 = cX
                    self.cY2 = cY
                    print("[INFO] ArUco marker 2: ({}, {})".format(cX, cY))
                if markerID == 3:
                    self.cX3 = cX
                    self.cY3 = cY
                    print("[INFO] ArUco marker 3: ({}, {})".format(cX, cY))
                if markerID == 4:
                    self.cX4 = cX
                    self.cY4 = cY
                    print("[INFO] ArUco marker 4: ({}, {})".format(cX, cY))
                    
        # calculate the center coordinates of the 4 ArUco markers
        cX_center = (self.cX1 + self.cX2 + self.cX3 + self.cX4)/4
        cY_center = (self.cY1 + self.cY2 + self.cY3 + self.cY4)/4
        cX_center = int(cX_center)
        cY_center = int(cY_center)
        cv2.circle(self.frame, (cX_center, cY_center), 4, (0, 0, 255), -1)
        cv2.putText(self.frame, "Center: ({}, {})".format(cX_center, cY_center), (cX_center - 15, cY_center - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # center coordinates of the frame (H, W) = (720, 960)
        cX_frame = 480
        cY_frame = 360
        cv2.circle(self.frame, (cX_frame, cY_frame), 4, (255, 0, 0), -1)
        cv2.putText(self.frame, "camera: ({}, {})".format(cX_frame, cY_frame), (cX_frame - 15, cY_frame - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        print("[INFO] markerIDList: {}".format(markerIDList))

        # align the center of the frame with the center of the ArUco marker
        cX_diff = cX_frame - cX_center
        cY_diff = cY_frame - cY_center
        cv2.putText(self.frame, "(X error: {}, Y error: {})".format(cX_diff, cY_diff), (cX_frame + 30, cY_frame + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # align X and Y coordinates of the ArUco marker with the center of the frame
        if cX_diff < -5:
            self.tello_move_right() 
        elif cX_diff > 5:
            self.tello_move_left()

        if cY_diff < -5:
            self.tello_move_down()
        elif cY_diff > 5:
            self.tello_move_up()
        
        if cX_center in range(cX_frame - 5, cX_frame + 5) and cY_center in range(cY_frame - 5, cY_frame + 5):
            while 1:
                self.tello_move_forward()

        # show the frame
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
    aruco_gate = RAS_Tello_ARUCO_GATE()
    rclpy.spin(aruco_gate)
    aruco_gate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
