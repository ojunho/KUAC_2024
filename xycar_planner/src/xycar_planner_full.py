#!/usr/bin/env python3

# 기본 Python3 인터프리터 설정

from __future__ import print_function

from xycar_msgs.msg import xycar_motor  # xycar 모터 메시지 모듈 임포트
from std_msgs.msg import String  # Float32 메시지 모듈 임포트

import cv2  # OpenCV 라이브러리 임포트

from cv_bridge import CvBridge  # CV-Bridge 라이브러리 임포트

import rospy  # ROS 파이썬 라이브러리 임포트

from obstacle_detector.msg import Obstacles

import os

import time

import numpy as np

class Obstacle:
    def __init__(self, x=None, y=None, distance=None):
        self.x = x
        self.y = y
        self.distance = distance

def nothing(x):
    pass

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



class XycarPlanner:
    def __init__(self):
        self.ctrl_cmd_msg = xycar_motor()

        self.ctrl_lane = xycar_motor()  # 모터 제어 메시지 초기화
        self.ctrl_static = xycar_motor()
        self.ctrl_rubbercone = xycar_motor()
        self.ctrl_ar = xycar_motor()

        rospy.init_node('xycar_planner', anonymous=True)  # ROS 노드 초기화
        
        # 카메라와 IMU 데이터 구독
        rospy.Subscriber("/xycar_motor_lane", xycar_motor, self.ctrlLaneCB)
        rospy.Subscriber("/xycar_motor_static", xycar_motor, self.ctrlStaticCB)
        rospy.Subscriber("/xycar_motor_ar", xycar_motor, self.ctrlARCB)
        rospy.Subscriber("/xycar_motor_rubbercone", xycar_motor, self.ctrlRubberconeCB)



        rospy.Subscriber("/raw_obstacles_static", Obstacles, self.staticObstacleCB)
        rospy.Subscriber("/raw_obstacles_rubbercone", Obstacles, self.rubberconeObstacleCB)


        # 모터 제어 명령과 현재 속도 퍼블리셔 설정
        self.ctrl_cmd_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)
        self.mode_pub = rospy.Publisher('/mode', String, queue_size=1)


        self.bridge = CvBridge()  # CV-Bridge 초기화

        self.version = rospy.get_param('~version', 'safe')

        rospy.loginfo(f"PLANNER_FULL: {self.version}")

        self.steer = 0.0  # 조향각 초기화
        self.motor = 0.0  # 모터 속도 초기화


        self.ar_mode_flag = True
        self.rubbercone_mode_flag = False
        self.static_mode_flag = False
        self.lane_mode_flag = False


        # mode
        self.mode = ''
        self.prev_mode = ''

        self.start_flag = False
        self.kill_flag = False

        self.steer_queue_size = 30
        self.steer_queue = np.zeros(self.steer_queue_size, dtype=int)



        self.pid = PID(0.7, 0.0008, 0.15)

        self.static_obstacles = []
        self.rubbercone_obstacles = []

        rate = rospy.Rate(30)  # 루프 주기 설정
        while not rospy.is_shutdown():  # ROS 노드가 종료될 때까지 반복



            # MODE 판별
            if self.ar_mode_flag == True:
                self.mode = 'AR'
            elif self.rubbercone_mode_flag == True:
                self.mode = 'RUBBERCONE'
            elif self.static_mode_flag == True:
                self.mode = 'STATIC'
            else:
                self.mode = 'LANE'
                if (self.prev_mode == 'AR') and (self.start_flag == False):
                    self.start_flag = True

                    stop_time = time.time()
                    while time.time() - stop_time < 2.2:
                        self.publishCtrlCmd(0, 0)
                        
                        if self.kill_flag == False:
                            os.system('rosnode kill /ar_tag_driver')
                            os.system('rosnode kill /ar_track_alvar')
                            os.system('rosnode kill /traffic_detection_node')
                            self.kill_flag = True


                    go_time = time.time()
                    while time.time() - go_time < 1.0:
                        self.publishCtrlCmd(7, 0)
                    continue
            
            self.prev_mode = self.mode

            self.mode_pub.publish(self.mode)

            # MODE에 따른 motor, steer 설정
            if self.mode != '':
                if self.mode == 'AR':
                    self.motor = self.ctrl_ar.speed
                    self.steer = self.ctrl_ar.angle
                elif self.mode == 'RUBBERCONE':
                    self.motor = self.ctrl_rubbercone.speed
                    self.steer = self.ctrl_rubbercone.angle
                elif self.mode == 'STATIC':
                    self.motor = self.ctrl_static.speed
                    self.steer = self.ctrl_static.angle
                else:               # LANE
                    self.motor = self.ctrl_lane.speed
                    self.steer = self.ctrl_lane.angle
            else:
                rospy.logwarn("SOMETHING WRONG")
                continue


            # --------------------------- 장애물 인지시 감속 --------------------------- # 
            if (len(self.static_obstacles) > 0) and (self.mode != "RUBBERCONE") and (self.mode != "AR"):

                if (-10 <= np.mean(self.steer_queue) <= 10) and (np.var(self.steer_queue) < 150):
                    # 특정 roi에 인지가 들어오면 일단 감속
                    for obstacle in self.static_obstacles:
                        if (0 < obstacle.x < 1.5) and (-0.25 <= obstacle.y <= 0.25):

                            if self.version == 'fast':
                                self.motor = 7
                            else:
                                self.motor = 7
            # --------------------------- 장애물 인지시 감속 --------------------------- # 

            # --------------------------- 라바콘 인지시 감속 --------------------------- # 
            # if len(self.rubbercone_obstacles) > 0:
            #     # 특정 roi에 인지가 들어오면 일단 감속
            #     for obstacle in self.rubbercone_obstacles:
            #         if (0 < obstacle.x < 2.0) and (-0.45 <= obstacle.y <= 0.45):
            #             self.motor = 30
            # --------------------------- 라바콘 인지시 감속 --------------------------- # 

            rospy.loginfo(f"MODE: {self.mode}")
            rospy.loginfo(f"SPEED: {self.motor}")


            self.publishCtrlCmd(self.motor, self.steer)

            cv2.waitKey(1)  # 키 입력 대기
            rate.sleep()  # 주기마다 대기
                
        cv2.destroyAllWindows()  # 창 닫기

        
    def publishCtrlCmd(self, motor_msg, servo_msg):

        self.steer_queue = np.roll(self.steer_queue, -1)
        self.steer_queue[-1] = servo_msg

        self.ctrl_cmd_msg.speed = motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시

    def ctrlLaneCB(self, msg):
        self.ctrl_lane.speed = msg.speed
        self.ctrl_lane.angle = msg.angle
        self.lane_mode_flag = msg.flag

    def ctrlStaticCB(self, msg):
        self.ctrl_static.speed = msg.speed
        self.ctrl_static.angle = msg.angle
        self.static_mode_flag = msg.flag

    def ctrlRubberconeCB(self, msg):
        self.ctrl_rubbercone.speed = msg.speed
        self.ctrl_rubbercone.angle = msg.angle
        self.rubbercone_mode_flag = msg.flag

    def ctrlARCB(self, msg):
        self.ctrl_ar.speed = msg.speed
        self.ctrl_ar.angle = msg.angle
        self.ar_mode_flag = msg.flag


    def staticObstacleCB(self, msg):
        self.static_obstacles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = (x**2 + y**2) ** 0.5  # 유클리드 거리 계산
            obstacle = Obstacle(x, y, distance)
            self.static_obstacles.append(obstacle)
        
        self.static_obstacles.sort(key=lambda obs: obs.distance)

        if len(self.static_obstacles) > 0:
            self.closest_static_obstacle = self.static_obstacles[0]
        else:
            self.closest_static_obstacle = Obstacle()
    
    def rubberconeObstacleCB(self, msg):
        self.rubbercone_obstacles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = (x**2 + y**2) ** 0.5  # 유클리드 거리 계산
            obstacle = Obstacle(x, y, distance)
            self.rubbercone_obstacles.append(obstacle)
        
        self.rubbercone_obstacles.sort(key=lambda obs: obs.distance)

        if len(self.rubbercone_obstacles) > 0:
            self.closest_rubbercone_obstacle = self.rubbercone_obstacles[0]
        else:
            self.closest_rubbercone_obstacle = Obstacle()




if __name__ == '__main__':
    try:
        autopilot_control = XycarPlanner()  # AutopilotControl 객체 생성
    except rospy.ROSInterruptException:
        pass  # 예외 발생 시 무시하고 종료
