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

import rospy  # ROS 파이썬 라이브러리 임포트
from sensor_msgs.msg import Image, CompressedImage  # 이미지 데이터 메시지 모듈 임포트

from obstacle_detector.msg import Obstacles

from utils import *

import tf




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
                    if self.obstacles[0][1] > 0: # 왼쪽
                        self.steer = 30
                    elif self.obstacles[0][1] < 0:
                        self.steer = -30
                    else:
                        self.steer = -30
                    self.publishCtrlCmd(self.motor, self.steer)

                elif self.cv_image is not None:  # 카메라 이미지가 있는 경우
                    cropped_image = roi_for_lane(self.cv_image)
                    gray_img, blurred_image, adaptive_gaussian, edged, closed_image = process_image(cropped_image)
                    warped_img = warper(closed_image)
                


                    out_img, x_location, _ = self.slidewindow.slidewindow(warped_img)  # 슬라이드 윈도우 알고리즘 적용

                    if x_location == None:  # x 위치가 없는 경우
                        x_location = last_x_location  # 이전 x 위치 사용
                    else:
                        last_x_location = x_location  # x 위치 갱신
                    
                    
                    self.steer = round(self.pid.pid_control(x_location - 320))  # PID 제어를 통한 각도 계산

                    self.motor = 30 # 모터 속도 설정 30
                    
                    self.publishCtrlCmd(self.motor, self.steer)  # 제어 명령 퍼블리시
                    

                    cv2.waitKey(1)  # 키 입력 대기



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

                if closest  < 0.7:
                    if 0 < len(self.obstacles) <= 2:
                        print("STATIC")
                        self.static_flag = True
                        self.rubbercone_flag = False

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
