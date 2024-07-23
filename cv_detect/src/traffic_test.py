#!/usr/bin/env python3

# 기본 Python3 인터프리터 설정

from __future__ import print_function

from xycar_msgs.msg import xycar_motor  # xycar 모터 메시지 모듈 임포트
from sensor_msgs.msg import Imu  # IMU 데이터 메시지 모듈 임포트
from std_msgs.msg import Float32  # Float32 메시지 모듈 임포트

from math import radians, pi  # 각도를 라디안으로 변환하는 함수 임포트

from slidewindow_testing import SlideWindow  # 슬라이드 윈도우 알고리즘 모듈 임포트
import cv2  # OpenCV 라이브러리 임포트
import numpy as np  # NumPy 라이브러리 임포트
import math

from cv_bridge import CvBridge, CvBridgeError  # CV-Bridge 라이브러리 임포트

import rospy  # ROS 파이썬 라이브러리 임포트
from sensor_msgs.msg import Image, CompressedImage  # 이미지 데이터 메시지 모듈 임포트

# from obstacle_detector.msg import Obstacles

import tf

from KMUfoscar import KMUFoscar

import tkinter as tk

def nothing(x):
    pass

def detect_traffic_light(image):
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


# PID 클래스 정의
class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp  # 비례 이득 설정
        self.ki = ki  # 적분 이득 설정
        self.kd = kd  # 미분 이득 설정
        self.p_error = 0.0  # 이전 비례 오차 초기화
        self.i_error = 0.0  # 적분 오차 초기화
        self.d_error = 0.0  # 미분 오차 초기화

    def pid_control(self, cte):
        self.d_error = cte - self.p_error  # 미분 오차 계산
        self.p_error = cte  # 비례 오차 갱신
        self.i_error += cte  # 적분 오차 갱신

        # PID 제어 계산
        return self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error



class LaneDetection(object):
    def __init__(self):
        rospy.init_node('lane_detection', anonymous=True)  # ROS 노드 초기화
        
        try:
            # 카메라와 IMU 데이터 구독
            rospy.Subscriber("/usb_cam/image_raw", Image, self.cameraCB)
            # rospy.Subscriber("/imu", Imu, self.imuCB)
            # rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacleCB)

            self.foscar = KMUFoscar()

            # 모터 제어 명령과 현재 속도 퍼블리셔 설정
            self.ctrl_cmd_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)
            # self.current_speed_pub = rospy.Publisher('/current_speed', Float32, queue_size=1)
            
            self.bridge = CvBridge()  # CV-Bridge 초기화
            self.ctrl_cmd_msg = xycar_motor()  # 모터 제어 메시지 초기화
            self.current_speed_msg = Float32()  # 현재 속도 메시지 초기화
            
            self.slidewindow = SlideWindow()  # 슬라이드 윈도우 알고리즘 초기화
            
            self.steer = 0.0  # 조향각 초기화
            self.motor = 25.0  # 모터 속도 초기화
            
            # self.pid = PID(0.15, 0.0003, 0.325)  # PID 제어기 초기화
   
            self.pid = PID(0.7, 0.0008, 0.15)
            self.cv_image = None  # 카메라 이미지 초기화
            self.prev_center_index = 320  # 이전 중심 인덱스 초기화
            
            self.start_flag = False  # 신호등 감지 시작 플래그 초기화
            
            # IMU 기반 속도 계산을 위한 변수 초기화
            self.current_speed = 0.0
            self.last_time = None
            self.linear_acceleration_x = 0.0

            self.v_over_200_coords = []

            self.obstacles = None
            self.v = None

            self.heading = 0.0

            self.lower_threshold = 100
            self.upper_threshold = 200

            root = tk.Tk()
            screen_width = root.winfo_screenwidth()
            screen_height = root.winfo_screenheight()
            root.destroy()

            print(f"Screen resolution: {screen_width}x{screen_height}")

            # Create windo
            # Directly set the values instead of using trackbars
            self.gaussian_kernel_size = 13  # Example value for Gaussian Kernel Size
            self.adaptive_thresh_C = 2.0  # Example value for Adaptive Threshold C
            self.adaptive_thresh_block_size = 11  # Example value for Adaptive Threshold Block Size
            self.canny_lower = 50  # Example value for Canny Lower
            self.canny_upper = 150  # Example value for Canny Upper
            self.morph_kernel_size = 5


            rate = rospy.Rate(30)  # 루프 주기 설정
            while not rospy.is_shutdown():  # ROS 노드가 종료될 때까지 반복

                if self.cv_image is not None:  # 카메라 이미지가 있는 경우
                    y, x = self.cv_image.shape[0:2]
                    cropped_image = self.cv_image[246:396, :]
                    y, x = cropped_image.shape[0:2]
                    
                    cropped_image_copy = cropped_image.copy()
                    
                    gray_img = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
                    blurred_image = cv2.GaussianBlur(gray_img, (self.gaussian_kernel_size, self.gaussian_kernel_size), 0)
                    adaptive_gaussian = cv2.adaptiveThreshold(blurred_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                                cv2.THRESH_BINARY, self.adaptive_thresh_block_size, self.adaptive_thresh_C)
                    edged = cv2.Canny(adaptive_gaussian, self.canny_lower, self.canny_upper)
                    kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
                    closed_image = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)
                    
                    # Define perspective transform points and apply warp
                    src_point1 = [0, 130]
                    src_point2 = [180, 54]
                    src_point3 = [x - 180, 54]
                    src_point4 = [x - 0, 130]
                    
                    src_points = np.float32([src_point1, src_point2, src_point3, src_point4])
                    dst_point1 = [x // 4, y]
                    dst_point2 = [x // 4, 0]
                    dst_point3 = [x // 4 * 3, 0]
                    dst_point4 = [x // 4 * 3, y]
                    
                    dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])
                    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                    warped_img = cv2.warpPerspective(closed_image, matrix, (x, y))
                    
                    out_img, x_location, _ = self.slidewindow.slidewindow(warped_img)
                    
                    if x_location is None:
                        x_location = last_x_location
                    else:
                        last_x_location = x_location
                    
                    center_index = x_location
                    angle = round(self.pid.pid_control(center_index - 320))
                    self.steer = angle
                    self.motor = 30
                    
                    self.publishCtrlCmd(self.motor, self.steer)
                    
                    cv2.imshow("Camera", self.cv_image)
                    # cv2.imshow('cropped_image', cropped_image)
                    # cv2.imshow('gray_img', gray_img)
                    # cv2.imshow('blurred_image', blurred_image)
                    # cv2.imshow('adaptive_gaussian', adaptive_gaussian)
                    # cv2.imshow('edged', edged)
                    # cv2.imshow('closed_image', closed_image)
                    # cv2.imshow('warped_img', warped_img)
                    # cv2.imshow('out_img', out_img)
                    # 이미지 로드
                    image = self.cv_image

                    # 신호등 검출
                    result_image = detect_traffic_light(image)

                    # 결과 이미지 표시
                    cv2.imshow('Detected Traffic Lights', result_image)    
                    cv2.waitKey(1)


                    # print("Heading: ", self.heading)

                rate.sleep()  # 주기마다 대기
                
        finally:
            cv2.destroyAllWindows()  # 창 닫기

    def update_lower_threshold(self, value):
        self.lower_threshold = value

    def update_upper_threshold(self, value):
        self.upper_threshold = value
        
    def publishCtrlCmd(self, motor_msg, servo_msg):
        self.ctrl_cmd_msg.speed = motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시
        
    def cameraCB(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # ROS 이미지 메시지를 OpenCV 이미지로 변환
        except CvBridgeError as e:
            print(e)
    
    # 마우스 콜백 함수 정의
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 현재 프레임에서 픽셀 값을 읽음
            pixel_value = self.v[y, x]
            # 픽셀 값과 좌표를 출력
            print(f'X: {x}, Y: {y}, Pixel Value: {pixel_value}')

    def imuCB(self, msg):
        # 쿼터니언을 사용하여 roll, pitch, yaw 계산
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.heading = yaw * 180.0 / pi

    # LiDAR에서 장애물 좌표 받아오기 
    def obstacleCB(self, msg):
        try:
            obstacles = []
            for circle in msg.circles:
                x = round(circle.center.x, 3)
                y = round(circle.center.y, 3)
                obstacles.append([x, y])
            self.obstacles = sorted(obstacles, key=lambda c: math.sqrt(c[0]**2 + c[1]**2))

            # 장애물이 어느 방향에 있는지 (왼쪽, 중앙, 오른쪽)
            if self.obstacles[0][1] > 0:
                self.foscar.static_obstacle_lcr = -1
            elif self.obstacles[0][1] < 0:
                self.foscar.static_obstacle_lcr = 1
            else:
                self.foscar.static_obstacle_lcr = 0

            self.foscar.static_obstacle_distance = self.obstacles[0][0]**2 + self.obstacles[0][1]**2
            # print("obstacle distance:", self.foscar.static_obstacle_distance)
            #-----------------------------------------------------------------#
            # 3m 이내에 장애물이 있는 경우 회피 동작(수양이가 수정해보는 것)
            if self.foscar.static_obstacle_distance <= 3.0:
                if self.foscar.static_obstacle_lcr == -1:
                    # 장애물이 왼쪽에 있는 경우
                    print("Obstacle on the left. Moving right.")
                    self.steer = 30  # 오른쪽으로 회전
                elif self.foscar.static_obstacle_lcr == 1:
                    # 장애물이 오른쪽에 있는 경우
                    print("Obstacle on the right. Moving left.")
                    self.steer = -30  # 왼쪽으로 회전
                else:
                    # 장애물이 중앙에 있는 경우
                    print("Obstacle in the center. Moving right.")
                    self.steer = 30  # 오른쪽으로 회전

                self.motor = 20  # 속도 감소
                self.publishCtrlCmd(self.motor, self.steer)

            #-----------------------------------------------------------------#
        except:
            print("Obstacle Detection Fail")
            self.obstacles = None



if __name__ == '__main__':
    try:
        autopilot_control = LaneDetection()  # AutopilotControl 객체 생성
    except rospy.ROSInterruptException:
        pass  # 예외 발생 시 무시하고 종료
