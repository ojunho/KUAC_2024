#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
import math
import time

class Obstacle:
    def __init__(self, x=None, y=None, distance=None):
        self.x = x
        self.y = y
        self.distance = distance


class StaticAvoidance():
    def __init__(self):
        rospy.Subscriber("/raw_obstacles_static", Obstacles, self.obstacleCB)
        rospy.Subscriber("/xycar_ultrasonic", Int32MultiArray, self.ultrasonicCB)

        # self.state -> 차량이 피해야 하는 방향
        # C: Center 
        # R: Right 
        # L: Left 
        self.state = 'C'
        self.obstacles = []
        self.ultrasonic = [] # 왼쪽, 오른쪽, 오른쪽뒤, 뒤, 왼쪽뒤
        self.is_static = False
        self.steer = 0
        self.speed = 10
        self.is_left = False

        self.closest_obstacle = Obstacle()

        # self.distance_threshold = 0.45
        self.distance_threshold = 1.2       # 정적 회피 시작거리 
        self.distance_threshold_max = 1.25   # 정적 회피중 다른 장애물과의 구분을 위한 임계점
        self.margin_y = 0.4

        self.angle = 0
        self.prev_angle = 0

        self.is_static = False # 

        self.steer_right_time = 0
        self.steer_left_time = 0
        self.final_adjust_time = 0

        self.ctrl_cmd_pub = rospy.Publisher('/xycar_motor_static', xycar_motor, queue_size=1)
        self.ctrl_cmd_msg = xycar_motor()

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():


            # 몇개가 나올지도 모르고, 어떻게 나올지도 모름. 
            # 1. 장애물의 위치 고려 
            if len(self.obstacles) > 0:
                self.closest_obstacle = self.obstacles[0]

                if (self.closest_obstacle.distance <= self.distance_threshold) and (self.is_static == False):
                    self.is_static = True
                    if self.closest_obstacle.y > 0:
                        self.state = 'R'
                    else:
                        self.state = 'L'
                    


            if self.is_static == True:
                
                if len(self.obstacles) > 0:

                    self.closest_obstacle = self.obstacles[0]



                    if self.state == 'R':
                        if (self.closest_obstacle.y > 0) and (self.closest_obstacle.distance < self.distance_threshold_max):
                            # self.angle = -100 * (self.closest_obstacle.distance - self.distance_threshold)
                            if (self.closest_obstacle.y - self.margin_y) > 0:
                                self.angle = -1000 * (self.closest_obstacle.y - self.margin_y)
                            else: 
                                self.angle = -100 * (self.closest_obstacle.y - self.margin_y)
                                
                        else:
                            self.angle = self.prev_angle
                    elif self.state == 'L':
                        if (self.closest_obstacle.y < 0) and (self.closest_obstacle.distance < self.distance_threshold_max):
                            self.angle = -100 * (self.closest_obstacle.y + self.margin_y)
                        else:
                            self.angle = self.prev_angle   

                else: 
                    self.angle = self.prev_angle

            self.prev_angle = self.angle

            print('self.state', self.state)
            print('self.angle', self.angle)

            # print('self.closest_obstacle', self.closest_obstacle.distance)
            print('self.closest_obstacle.y:', self.closest_obstacle.y)
            print(" ")
            


                


            # 2. 라이다를 통해서 우선 피한채로 유지 [1. 거리 기반  2. 각도 기반]

            # 3. 초음파 센서를 통해 상태 유지

            # 4. 다시 복귀 




            

            self.publishCtrlCmd(4, self.angle, self.is_static)


            # self.static_pub.publish(self.steer)
            rate.sleep()
                        

    def obstacleCB(self, msg):
        self.obstacles = []
        for circle in msg.circles:
            x = circle.center.x
            y = circle.center.y
            distance = (x**2 + y**2) ** 0.5  # 유클리드 거리 계산
            obstacle = Obstacle(x, y, distance)
            self.obstacles.append(obstacle)
        
        self.obstacles.sort(key=lambda obs: obs.distance)

        if len(self.obstacles) > 0:
            self.closest_obstacle = self.obstacles[0]
        else:
            self.closest_obstacle = Obstacle()

    def ultrasonicCB(self, msg):
        ultrasonic = list(msg.data)
        ultrasonic.pop(1)
        ultrasonic.pop(1)
        ultrasonic.pop(1)
        self.ultrasonic = ultrasonic

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg.speed = round(motor_msg)  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = round(servo_msg)  # 조향각 설정
        self.ctrl_cmd_msg.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시


if __name__ == '__main__':
    rospy.init_node('static_obstacle_avoidance', anonymous=True)
    try:
        static_obstacle_avoidance = StaticAvoidance()
    except rospy.ROSInterruptException:
        pass
