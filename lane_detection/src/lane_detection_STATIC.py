#!/usr/bin/env python3

# 기본 Python3 인터프리터 설정

from __future__ import print_function

from xycar_msgs.msg import xycar_motor  # xycar 모터 메시지 모듈 임포트
from sensor_msgs.msg import Imu  # IMU 데이터 메시지 모듈 임포트
from std_msgs.msg import Float32  # Float32 메시지 모듈 임포트

from math import radians, pi  # 각도를 라디안으로 변환하는 함수 임포트

from slidewindow import SlideWindow  # 슬라이드 윈도우 알고리즘 모듈 임포트
import cv2  # OpenCV 라이브러리 임포트
import numpy as np  # NumPy 라이브러리 임포트
import math

from cv_bridge import CvBridge, CvBridgeError  # CV-Bridge 라이브러리 임포트

import os
import rospy  # ROS 파이썬 라이브러리 임포트
from sensor_msgs.msg import Image, CompressedImage  # 이미지 데이터 메시지 모듈 임포트

from obstacle_detector.msg import Obstacles

import tf

from KMUfoscar import KMUFoscar



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
            rospy.Subscriber("/imu", Imu, self.imuCB)
            rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacleCB)

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
            self.static_flag = False
            self.static_left = False # left 1, right -1, center 0
            self.rubbercone_flag = False
            
            # IMU 기반 속도 계산을 위한 변수 초기화
            self.current_speed = 0.0
            self.last_time = None
            self.linear_acceleration_x = 0.0

            self.v_over_200_coords = []

            self.obstacles = None
            self.v = None

            self.heading = 0.0

            rate = rospy.Rate(30)  # 루프 주기 설정
            while not rospy.is_shutdown():  # ROS 노드가 종료될 때까지 반복
                if self.static_flag:
                    print("=================================")
                    tangent = self.obstacles[0][1] / self.obstacles[0][0]
                    degree = math.atan(tangent) * 180 / math.pi
                    if self.static_left: # 왼쪽
                        # self.steer = round(163 - 5.56 * degree + 0.04 * degree ** 2)
                        self.steer = round(38 - 1.74 * degree + 0.0136 * degree ** 2) # 38.1 + -1.74x + 0.0136x^2
                    else:
                        self.steer = round(-163 - 5.56 * degree - 0.04 * degree ** 2)
                    os.system("clear")
                    print(f"STATIC: {'LEFT' if self.static_left else 'RIGHT'}, Degree: {degree:.2f}, Steer: {self.steer:.2f}")
                    self.publishCtrlCmd(self.motor, self.steer)

                elif self.cv_image is not None:  # 카메라 이미지가 있는 경우
                    y, x = self.cv_image.shape[0:2]  # 이미지의 높이와 너비 가져오기
                    
                    # X: 640
                    # Y: 150


                    cropped_image = self.cv_image[246:396, :]  # 이미지의 하단 부분만 사용
                    y, x = cropped_image.shape[0:2]

                    # cv2.imshow('cropped_image', cropped_image)
                    cropped_image_copy = cropped_image.copy()
                    # -------------------------------------------------------------------- # 


                    # -------------------- 예선과제 이미지처리 ----------------------------- # 
                    gray_img = cv2.cvtColor(cropped_image_copy, cv2.COLOR_BGR2GRAY)  # 그레이스케일 변환
                    
                    blurred_image = cv2.GaussianBlur(gray_img, (15, 3), 0)  # 가우시안 블러(5, 5)

                    # 적응형 이진화
                    adaptive_gaussian = cv2.adaptiveThreshold(blurred_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                              cv2.THRESH_BINARY, 9, -2.5) # 11, -2.5
                    sigma = 0.33
                    median_value = np.median(adaptive_gaussian)
                    lower = int(max(0, (1.0-sigma) * median_value))
                    upper = int(min(255, (1.0 + sigma) * median_value))
                    edged = cv2.Canny(adaptive_gaussian, lower, upper)

                    # 캐니 에지 검출
                    # edges = cv2.Canny(adaptive_gaussian, 70, 210) # 50, 150

                    # 형태학적 닫기 연산
                    kernel = np.ones((1, 1), np.uint8) # 4, 4
                    closed_image = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)
                    # ---------------------------------------------------------------------- #


                    # X: 0, Y: 130, Pixel Value: 76
                    # X: 166, Y: 54, Pixel Value: 65
                    # X: 498, Y: 57, Pixel Value: 66
                    # X: 637, Y: 106, Pixel Value: 79


                    # 카메라 위치 (y=1.5)
                    left_margin_1 = 0
                    top_margin_1 = 130


                    
                    left_margin_2 = 180
                    top_margin_2 = 54


                    
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
                    warped_img = cv2.warpPerspective(edged, matrix, (x, y))  # 원근 변환 적용
                    
                    out_img, x_location, _ = self.slidewindow.slidewindow(warped_img)  # 슬라이드 윈도우 알고리즘 적용

                    if x_location == None:  # x 위치가 없는 경우
                        x_location = last_x_location  # 이전 x 위치 사용
                    else:
                        last_x_location = x_location  # x 위치 갱신
                    
                    center_index = x_location  # 중심 인덱스 설정
                    
                    # 튀는 값 잡기
                    diff_center_index = abs(center_index - self.prev_center_index)
                    # print("Diff Center Index: ", diff_center_index)
                    if diff_center_index > 50:  # 차이가 60보다 큰 경우
                        center_index = self.prev_center_index  # 이전 중심 인덱스 사용
                        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    else:
                        self.prev_center_index = center_index  # 중심 인덱스 갱신
                    
                    angle = round(self.pid.pid_control(center_index - 320))  # PID 제어를 통한 각도 계산
                    # angle = round(center_index - 320)
                    self.steer = angle
                    # self.steer = radians(angle)  # 각도를 라디안으로 변환

                    # print("Steer: ", self.steer)
                    self.motor = 5  # 모터 속도 설정
                    
                    self.publishCtrlCmd(self.motor, self.steer)  # 제어 명령 퍼블리시
                    
                    # self.current_speed_msg.data = self.current_speed  # 현재 속도 설정
                    # self.current_speed_pub.publish(self.current_speed_msg)  # 현재 속도 퍼블리시

                    # cv2.imshow("Camera", self.cv_image)
                    # cv2.imshow('cropped_image', cropped_image)

                    # cv2.imshow('gray_img', gray_img)
                    # cv2.imshow('blurred_image', blurred_image)
                    # cv2.imshow('adaptive_gaussian', adaptive_gaussian)
                    cv2.imshow('edged', edged)
                    # cv2.imshow('closed_image', closed_image)

                    # cv2.imshow('warped_img', warped_img)
                    cv2.imshow('out_img', out_img)

                    # cv2.imshow('thres_img', thres_img)
                    cv2.waitKey(1)  # 키 입력 대기


                    # print("Heading: ", self.heading)

                rate.sleep()  # 주기마다 대기
                
        finally:
            cv2.destroyAllWindows()  # 창 닫기
        
    def publishCtrlCmd(self, motor_msg, servo_msg):
        self.ctrl_cmd_msg.speed = 5 #motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = servo_msg  # 조향각 설정
        print(servo_msg, end="  ")
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

            if (len(obstacles) == 0) :
                self.static_flag = False
                self.rubbercone_flag = False
                self.obstacles = []
            else:
                self.obstacles = sorted(obstacles, key=lambda c: math.sqrt(c[0]**2 + c[1]**2))
                closest = math.sqrt(self.obstacles[0][0] ** 2 + self.obstacles[0][1] ** 2)
                print('Closest:', closest)

                if closest  < 0.7: # 0.7:
                    if 0 < len(self.obstacles) <= 2:
                        print("STATIC")
                        self.static_flag = True
                        self.rubbercone_flag = False
                        if self.obstacles[0][1] > 0:
                            self.static_left = True
                        else:
                            self.static_left = False

                    elif len(self.obstacles) >= 3:
                        print("RUBBERCONE")
                        self.static_flag = False
                        self.rubbercone_flag = True

            # print("obstacle distance:", self.foscar.static_obstacle_distance)
            #-----------------------------------------------------------------#
            # 3m 이내에 장애물이 있는 경우 회피 동작(수양이가 수정해보는 것)
            # if self.foscar.static_obstacle_distance <= 3.0:
            #     if self.foscar.static_obstacle_lcr == -1:
            #         # 장애물이 왼쪽에 있는 경우
            #         print("Obstacle on the left. Moving right.")
            #         self.steer = 30  # 오른쪽으로 회전
            #     elif self.foscar.static_obstacle_lcr == 1:
            #         # 장애물이 오른쪽에 있는 경우
            #         print("Obstacle on the right. Moving left.")
            #         self.steer = -30  # 왼쪽으로 회전
            #     else:
            #         # 장애물이 중앙에 있는 경우
            #         print("Obstacle in the center. Moving right.")
            #         self.steer = 30  # 오른쪽으로 회전

            #     self.motor = 20  # 속도 감소
            #     self.publishCtrlCmd(self.motor, self.steer)

            #-----------------------------------------------------------------#
        except Exception as e:
            print(e)
            print("Obstacle Detection Fail")
            self.obstacles = None



if __name__ == '__main__':
    try:
        import time
        time.sleep(10)
        autopilot_control = LaneDetection()  # AutopilotControl 객체 생성
    except rospy.ROSInterruptException:
        pass  # 예외 발생 시 무시하고 종료
