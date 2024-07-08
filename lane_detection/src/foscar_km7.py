#!/usr/bin/env python3

# 기본 Python3 인터프리터 설정

from __future__ import print_function

from xycar_msgs.msg import xycar_motor  # xycar 모터 메시지 모듈 임포트
from sensor_msgs.msg import Imu  # IMU 데이터 메시지 모듈 임포트
from std_msgs.msg import Float32  # Float32 메시지 모듈 임포트

from math import radians  # 각도를 라디안으로 변환하는 함수 임포트

from slidewindow import SlideWindow  # 슬라이드 윈도우 알고리즘 모듈 임포트
import cv2  # OpenCV 라이브러리 임포트
import numpy as np  # NumPy 라이브러리 임포트

from cv_bridge import CvBridge, CvBridgeError  # CV-Bridge 라이브러리 임포트

import rospy  # ROS 파이썬 라이브러리 임포트
from sensor_msgs.msg import Image  # 이미지 데이터 메시지 모듈 임포트


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


# AutopilotControl 클래스 정의
class AutopilotControl(object):
    def __init__(self):
        rospy.init_node('lane_detection', anonymous=True)  # ROS 노드 초기화
        
        try:
            # 카메라와 IMU 데이터 구독
            rospy.Subscriber("/usb_cam/image_raw/compressed", Image, self.cameraCB)
            # rospy.Subscriber("/imu", Imu, self.imuCB)
            
            # 모터 제어 명령과 현재 속도 퍼블리셔 설정
            self.ctrl_cmd_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)
            self.current_speed_pub = rospy.Publisher('/current_speed', Float32, queue_size=1)
            
            self.bridge = CvBridge()  # CV-Bridge 초기화
            self.ctrl_cmd_msg = xycar_motor()  # 모터 제어 메시지 초기화
            self.current_speed_msg = Float32()  # 현재 속도 메시지 초기화
            
            self.slidewindow = SlideWindow()  # 슬라이드 윈도우 알고리즘 초기화
            
            self.steer = 0.0  # 조향각 초기화
            self.motor = 25.0  # 모터 속도 초기화
            
            self.pid = PID(0.15, 0.0003, 0.325)  # PID 제어기 초기화
            self.cv_image = None  # 카메라 이미지 초기화
            self.prev_center_index = 400  # 이전 중심 인덱스 초기화
            
            self.start_flag = False  # 신호등 감지 시작 플래그 초기화
            
            # IMU 기반 속도 계산을 위한 변수 초기화
            self.current_speed = 0.0
            self.last_time = None
            self.linear_acceleration_x = 0.0
            
            rate = rospy.Rate(30)  # 루프 주기 설정
            while not rospy.is_shutdown():  # ROS 노드가 종료될 때까지 반복

                if self.cv_image is not None:  # 카메라 이미지가 있는 경우
                    y, x = self.cv_image.shape[0:2]  # 이미지의 높이와 너비 가져오기
                    # print('current speed: ', self.current_speed)  # 현재 속도 출력
                    
                    ############################### 신호등 감지 ############################
                    # if self.start_flag is False:  # 신호등 감지를 시작하지 않은 경우
                    #     y_roi_start = 98  # ROI 시작 y 좌표
                    #     y_roi_end = 261  # ROI 끝 y 좌표
                    #     x_roi_start = 572  # ROI 시작 x 좌표
                    #     x_roi_end = 673  # ROI 끝 x 좌표

                    #     cv_image_cropped = self.cv_image.copy()  # 원본 이미지 복사
                    #     cv_image_cropped = cv_image_cropped[y_roi_start:y_roi_end, x_roi_start:x_roi_end, :]  # ROI 설정
                        
                    #     cv2.imshow('cv_image_cropped', cv_image_cropped)  # ROI 이미지 출력
                        
                    #     hsv_red = cv2.cvtColor(cv_image_cropped, cv2.COLOR_BGR2HSV)  # BGR 이미지를 HSV로 변환
                    #     hsv_green = cv2.cvtColor(cv_image_cropped, cv2.COLOR_BGR2HSV)  # BGR 이미지를 HSV로 변환

                    #     lower_red_1 = np.array([0, 160, 125])  # 빨간색 하한 값 설정
                    #     upper_red_1 = np.array([3, 255, 255])  # 빨간색 상한 값 설정
                    #     red_mask_1 = cv2.inRange(hsv_red, lower_red_1, upper_red_1)  # 빨간색 마스크 생성

                    #     lower_red_2 = np.array([165, 160, 125])  # 빨간색 하한 값 설정
                    #     upper_red_2 = np.array([180, 255, 255])  # 빨간색 상한 값 설정
                    #     red_mask_2 = cv2.inRange(hsv_red, lower_red_2, upper_red_2)  # 빨간색 마스크 생성

                    #     red_mask = red_mask_1 + red_mask_2  # 두 개의 빨간색 마스크 합치기

                    #     lower_green = np.array([60, 200, 120])  # 녹색 하한 값 설정
                    #     upper_green = np.array([70, 255, 255])  # 녹색 상한 값 설정
                    #     green_mask = cv2.inRange(hsv_green, lower_green, upper_green)  # 녹색 마스크 생성

                    #     red_pixel_counts = np.count_nonzero(red_mask)  # 빨간색 픽셀 수 계산
                    #     green_pixel_counts = np.count_nonzero(green_mask)  # 녹색 픽셀 수 계산
                        
                    #     if green_pixel_counts > 50:  # 녹색 픽셀 수가 50보다 많은 경우
                    #         self.start_flag = True  # 신호등 감지 시작
                            
                    #     continue  # 다음 루프로 넘어감
                    
                    #####################################################################
                    
                    cropped_image = self.cv_image[300:, :]  # 이미지의 하단 부분만 사용
                    y = y // 2  # 이미지 높이 절반으로 줄이기
                    gray_img = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)  # 그레이스케일 변환
                    
                    blurred_image = cv2.GaussianBlur(gray_img, (15, 15), 0)  # 가우시안 블러

                    # 적응형 이진화
                    adaptive_gaussian = cv2.adaptiveThreshold(blurred_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                              cv2.THRESH_BINARY, 11, -2.5)
                    # 캐니 에지 검출
                    edges = cv2.Canny(adaptive_gaussian, 50, 150)

                    # 형태학적 닫기 연산
                    kernel = np.ones((4, 4), np.uint8)
                    closed_image = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
                    
                    # 카메라 위치 (y=1.5)
                    left_margin_1 = 49
                    top_margin_1 = 298
                    
                    left_margin_2 = 305
                    top_margin_2 = 78
                    
                    src_point1 = [left_margin_1, top_margin_1]  # 왼쪽 아래 점
                    src_point2 = [left_margin_2, top_margin_2]  # 왼쪽 위 점
                    src_point3 = [x - left_margin_2, top_margin_2]  # 오른쪽 위 점
                    src_point4 = [x - left_margin_1, top_margin_1]  # 오른쪽 아래 점

                    src_points = np.float32([src_point1, src_point2, src_point3, src_point4])  # 원본 이미지에서의 점들
                    
                    dst_point1 = [x // 4, y]  # 변환 이미지에서의 왼쪽 아래 점
                    dst_point2 = [x // 4, 0]  # 변환 이미지에서의 왼쪽 위 점
                    dst_point3 = [x // 4 * 3, 0]  # 변환 이미지에서의 오른쪽 위 점
                    dst_point4 = [x // 4 * 3, y]  # 변환 이미지에서의 오른쪽 아래 점

                    dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])  # 변환 이미지에서의 점들
                    
                    matrix = cv2.getPerspectiveTransform(src_points, dst_points)  # 원근 변환 행렬 계산
                    warped_img = cv2.warpPerspective(closed_image, matrix, [x, y])  # 원근 변환 적용
                    
                    out_img, x_location, _ = self.slidewindow.slidewindow(warped_img)  # 슬라이드 윈도우 알고리즘 적용

                    if x_location == None:  # x 위치가 없는 경우
                        x_location = last_x_location  # 이전 x 위치 사용
                    else:
                        last_x_location = x_location  # x 위치 갱신
                    
                    center_index = x_location  # 중심 인덱스 설정
                    
                    # 튀는 값 잡기
                    if abs(center_index - self.prev_center_index) > 60:  # 차이가 60보다 큰 경우
                        center_index = self.prev_center_index  # 이전 중심 인덱스 사용
                        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    else:
                        self.prev_center_index = center_index  # 중심 인덱스 갱신
                    
                    angle = self.pid.pid_control(center_index - 400)  # PID 제어를 통한 각도 계산
                    
                    self.steer = radians(angle)  # 각도를 라디안으로 변환
                    self.motor = 0.43  # 모터 속도 설정
                    
                    self.publishCtrlCmd(self.motor, self.steer)  # 제어 명령 퍼블리시
                    
                    self.current_speed_msg.data = self.current_speed  # 현재 속도 설정
                    self.current_speed_pub.publish(self.current_speed_msg)  # 현재 속도 퍼블리시
                    
                    cv2.waitKey(1)  # 키 입력 대기

                rate.sleep()  # 주기마다 대기
                
        finally:
            cv2.destroyAllWindows()  # 창 닫기
        
    def publishCtrlCmd(self, motor_msg, servo_msg):
        self.ctrl_cmd_msg.speed = motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시
        
    def cameraCB(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # ROS 이미지 메시지를 OpenCV 이미지로 변환
        except CvBridgeError as e:
            print(e)
    
    def imuCB(self, msg):
        linear_acceleration = msg.linear_acceleration.x  # IMU에서 선형 가속도 가져오기
        current_time = msg.header.stamp.to_sec()  # 현재 시간 가져오기

        if self.last_time is None:  # 이전 시간이 없는 경우
            self.last_time = current_time  # 현재 시간을 이전 시간으로 설정
            return

        dt = current_time - self.last_time  # 시간 차이 계산
        self.last_time = current_time  # 이전 시간 갱신

        # 가속도를 시간에 대해 적분하여 속도 계산
        self.current_speed += (linear_acceleration * dt)
        

if __name__ == '__main__':
    try:
        autopilot_control = AutopilotControl()  # AutopilotControl 객체 생성
    except rospy.ROSInterruptException:
        pass  # 예외 발생 시 무시하고 종료
