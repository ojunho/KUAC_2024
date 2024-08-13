#!/usr/bin/env python3

from __future__ import print_function
from math import radians, pi
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import numpy as np
from utils import *

from std_msgs.msg import Int64MultiArray, Bool

class RubberconeOrangeDetection:
    def __init__(self):
        rospy.init_node('rubbercone_orange_detection', anonymous=True)

        try:
            self.bridge = CvBridge()

            rospy.Subscriber("/usb_cam/image_raw", Image, self.cameraCB)
            self.is_orange_pub = rospy.Publisher("/is_orange", Bool, queue_size=1)


            self.cv_image = None

            self.is_orange_msg = Bool()

            # 트랙바 윈도우 생성
            cv2.namedWindow('ORANGE Trackbars')
            cv2.createTrackbar('H_min_orange', 'ORANGE Trackbars', 0, 179, self.nothing)
            cv2.createTrackbar('H_max_orange', 'ORANGE Trackbars', 17, 179, self.nothing)
            cv2.createTrackbar('S_min_orange', 'ORANGE Trackbars', 133, 255, self.nothing)
            cv2.createTrackbar('S_max_orange', 'ORANGE Trackbars', 205, 255, self.nothing)
            cv2.createTrackbar('V_min_orange', 'ORANGE Trackbars', 113, 255, self.nothing)
            cv2.createTrackbar('V_max_orange', 'ORANGE Trackbars', 161, 255, self.nothing)
            
            # cv2.namedWindow('Green Trackbars')
            # cv2.createTrackbar('H_min_green1', 'Green Trackbars', 68, 179, self.nothing)
            # cv2.createTrackbar('H_max_green1', 'Green Trackbars', 91, 179, self.nothing)
            # cv2.createTrackbar('S_min_green1', 'Green Trackbars', 0, 255, self.nothing)
            # cv2.createTrackbar('S_max_green1', 'Green Trackbars', 180, 255, self.nothing)
            # cv2.createTrackbar('V_min_green1', 'Green Trackbars', 218, 255, self.nothing)
            # cv2.createTrackbar('V_max_green1', 'Green Trackbars', 255, 255, self.nothing)

            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                if self.cv_image is not None:
                    self.detect_orange(self.cv_image)
                    cv2.waitKey(1)
                rate.sleep()

        finally:
            cv2.destroyAllWindows()

    def nothing(self, x):
        pass

    def cameraCB(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)




    # def get_contour_centers(self, contours):
    #     centers = []
    #     for contour in contours:
    #         M = cv2.moments(contour)
    #         if M["m00"] != 0:
    #             cX = int(M["m10"] / M["m00"])
    #             cY = int(M["m01"] / M["m00"])
    #             centers.append((cX, cY))
    #     return centers

    def detect_orange(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        h, s, v = cv2.split(hsv_image)

        # cv2.imshow('h', h)
        # cv2.imshow('s', s)
        # cv2.imshow('v', v)

        # Red trackbar values
        h_min_orange = cv2.getTrackbarPos('H_min_orange', 'ORANGE Trackbars')
        h_max_orange = cv2.getTrackbarPos('H_max_orange', 'ORANGE Trackbars')
        s_min_orange = cv2.getTrackbarPos('S_min_orange', 'ORANGE Trackbars')
        s_max_orange = cv2.getTrackbarPos('S_max_orange', 'ORANGE Trackbars')
        v_min_orange = cv2.getTrackbarPos('V_min_orange', 'ORANGE Trackbars')
        v_max_orange = cv2.getTrackbarPos('V_max_orange', 'ORANGE Trackbars')

        lower_orange = np.array([h_min_orange, s_min_orange, v_min_orange])
        upper_orange = np.array([h_max_orange, s_max_orange, v_max_orange])

        orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

        # cv2.imshow('orange_mask', orange_mask)
        # 윤곽선 찾기

        orange_pixel_counts = np.count_nonzero(orange_mask)

        #
        if orange_pixel_counts > 10000:
            self.is_orange_msg.data = True
        else:
            self.is_orange_msg.data = False
        
        self.is_orange_pub.publish(self.is_orange_msg)




if __name__ == '__main__':
    try:
        rubbercone_orange_detection_node = RubberconeOrangeDetection()
    except rospy.ROSInterruptException:
        pass
