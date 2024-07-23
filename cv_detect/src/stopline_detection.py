#!/usr/bin/env python3

# 기본 Python3 인터프리터 설정

from __future__ import print_function

from math import radians, pi  # 각도를 라디안으로 변환하는 함수 임포트

import cv2  # OpenCV 라이브러리 임포트

from cv_bridge import CvBridge, CvBridgeError  # CV-Bridge 라이브러리 임포트

import rospy  # ROS 파이썬 라이브러리 임포트
from sensor_msgs.msg import Image  # 이미지 데이터 메시지 모듈 임포트
from utils import *

class StopLineDetection:
    def __init__(self):
        rospy.init_node('stopline_detection', anonymous=True)  # ROS 노드 초기화
        
        try:
            # 카메라와 IMU 데이터 구독
            rospy.Subscriber("/usb_cam/image_raw", Image, self.cameraCB)



            self.bridge = CvBridge()  # CV-Bridge 초기화

            self.cv_image = None  # 카메라 이미지 초기화

            rate = rospy.Rate(30)  # 루프 주기 설정
            while not rospy.is_shutdown():  # ROS 노드가 종료될 때까지 반복

                if self.cv_image is not None:  # 카메라 이미지가 있는 경우

                    cropped_image = roi_for_lane(self.cv_image)
                    gray_img, blurred_image, adaptive_gaussian, edged, closed_image = process_image(cropped_image)
                    

                    check_stop = self.check_stopline(adaptive_gaussian)
                    print(check_stop)

                    cv2.waitKey(1)  # 키 입력 대기
                rate.sleep()  # 주기마다 대기
                
        finally:
            cv2.destroyAllWindows()  # 창 닫기

        
    def cameraCB(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # ROS 이미지 메시지를 OpenCV 이미지로 변환
        except CvBridgeError as e:
            print(e)

    def check_stopline(self, image):
        stopline_count = cv2.countNonZero(image)
        print(f"Number of non-zero pixels: {stopline_count}")
        if stopline_count > 13500:
            return 1
        else:
            return 0


if __name__ == '__main__':
    try:
        stopline_detection_node = StopLineDetection()  # AutopilotControl 객체 생성
    except rospy.ROSInterruptException:
        pass  # 예외 발생 시 무시하고 종료
