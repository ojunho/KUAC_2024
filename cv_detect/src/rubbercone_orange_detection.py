#!/usr/bin/env python3

from __future__ import print_function
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import numpy as np
from utils import *

from std_msgs.msg import Bool

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
            # cv2.namedWindow('ORANGE Trackbars')
            # cv2.createTrackbar('H_min_orange', 'ORANGE Trackbars', 0, 179, self.nothing)
            # cv2.createTrackbar('H_max_orange', 'ORANGE Trackbars', 17, 179, self.nothing)
            # cv2.createTrackbar('S_min_orange', 'ORANGE Trackbars', 133, 255, self.nothing)
            # cv2.createTrackbar('S_max_orange', 'ORANGE Trackbars', 205, 255, self.nothing)
            # cv2.createTrackbar('V_min_orange', 'ORANGE Trackbars', 113, 255, self.nothing)
            # cv2.createTrackbar('V_max_orange', 'ORANGE Trackbars', 161, 255, self.nothing)
            
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

    def detect_orange(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Red trackbar values
        h_min_orange = 0
        h_max_orange = 17
        s_min_orange = 133
        s_max_orange = 205
        v_min_orange = 113
        v_max_orange = 161

        lower_orange = np.array([h_min_orange, s_min_orange, v_min_orange])
        upper_orange = np.array([h_max_orange, s_max_orange, v_max_orange])

        orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

        # 윤곽선 찾기
        orange_pixel_counts = np.count_nonzero(orange_mask)
        # rospy.loginfo(f'orange_pixel_counts: {orange_pixel_counts}')
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
