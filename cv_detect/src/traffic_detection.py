#!/usr/bin/env python3

# 기본 Python3 인터프리터 설정

from __future__ import print_function

from math import radians, pi  # 각도를 라디안으로 변환하는 함수 임포트

import cv2  # OpenCV 라이브러리 임포트

from cv_bridge import CvBridge, CvBridgeError  # CV-Bridge 라이브러리 임포트

import rospy  # ROS 파이썬 라이브러리 임포트
from sensor_msgs.msg import Image  # 이미지 데이터 메시지 모듈 임포트
from utils import *

class TrafficDetection:
    def __init__(self):
        rospy.init_node('traffic_detection', anonymous=True)  # ROS 노드 초기화
        
        try:
            # 카메라와 IMU 데이터 구독
            rospy.Subscriber("/usb_cam/image_raw", Image, self.cameraCB)



            self.bridge = CvBridge()  # CV-Bridge 초기화

            self.cv_image = None  # 카메라 이미지 초기화

            rate = rospy.Rate(30)  # 루프 주기 설정
            while not rospy.is_shutdown():  # ROS 노드가 종료될 때까지 반복

                if self.cv_image is not None:  # 카메라 이미지가 있는 경우



                    result_image = self.detect_traffic_light(self.cv_image)
                    cv2.imshow('result_image', result_image)

                    cv2.waitKey(1)  # 키 입력 대기
                rate.sleep()  # 주기마다 대기
                
        finally:
            cv2.destroyAllWindows()  # 창 닫기

        
    def cameraCB(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # ROS 이미지 메시지를 OpenCV 이미지로 변환
        except CvBridgeError as e:
            print(e)

    def detect_traffic_light(self, image):
        # 이미지 색상 공간 변환 (BGR -> HSV)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 신호등의 색상 범위 정의 (HSV 색상 범위)
        # 빨간색
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # 초록색 (세분화된 범위)
        lower_green1 = np.array([30, 100, 100])
        upper_green1 = np.array([50, 255, 255])
        lower_green2 = np.array([50, 100, 100])
        upper_green2 = np.array([85, 255, 255])

        # 색상 범위에 따라 마스크 생성
        red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        green_mask1 = cv2.inRange(hsv_image, lower_green1, upper_green1)
        green_mask2 = cv2.inRange(hsv_image, lower_green2, upper_green2)
        green_mask = cv2.bitwise_or(green_mask1, green_mask2)

        # 마스크를 정제하기 위해 형태학적 연산 적용
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        # 마스크를 원본 이미지에 적용
        red_result = cv2.bitwise_and(image, image, mask=red_mask)
        green_result = cv2.bitwise_and(image, image, mask=green_mask)

        # 마스크 이미지에서 신호등 위치 검출
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 신호등을 원본 이미지에 표시
        for contour in red_contours:
            if cv2.contourArea(contour) > 500:  # 면적 필터링
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)  # 빨간색 박스
                print("RED")

        for contour in green_contours:
            if cv2.contourArea(contour) > 10:  # 면적 필터링
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)  # 초록색 박스
                print("GREEN")

        return image


if __name__ == '__main__':
    try:
        traffic_detection_node = TrafficDetection()  # AutopilotControl 객체 생성
    except rospy.ROSInterruptException:
        pass  # 예외 발생 시 무시하고 종료
