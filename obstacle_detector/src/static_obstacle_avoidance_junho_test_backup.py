#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32, Float64, Int32MultiArray
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
        rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacleCB)
        rospy.Subscriber("/heading", Float64, self.headingCB)


        self.ctrl_cmd_pub = rospy.Publisher('/xycar_motor_static', xycar_motor, queue_size=1)
        self.ctrl_cmd_msg = xycar_motor()

        # self.state
        # L: Lane
        # A: Avoid
        # R: Return
        self.state = 'L'

        self.obstacles = []
        self.is_static = False
        self.steer = 0
        self.speed = 0
        self.is_left = False

        self.closest_obstacle = Obstacle()

        # self.distance_threshold = 0.45
        self.distance_threshold = 1.2       # 정적 회피 시작거리 
        self.distance_threshold_max = 1.25   # 정적 회피중 다른 장애물과의 구분을 위한 임계점
        self.margin_y = 0.4

        self.angle = 0
        self.prev_angle = 0


        self.steer_right_time = 0
        self.steer_left_time = 0
        self.final_adjust_time = 0


        self.static_obstacle_cnt = 0

        self.gt_heading = None
        self.heading = None

        self.avoid_heading = None
        self.return_heading = None

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():


            if len(self.obstacles) > 0:
                # 특정 roi에 인지가 들어오면 일단 감속
                for obstacle in self.obstacles:
                    if (0 < obstacle.x < 2.0) and (-0.45 <= obstacle.y <= 0.45):
                        self.speed = 4
                    
                    if (0 < obstacle.x < 2.0) and (-0.25 <= obstacle.y <= 0.25):
                        self.static_obstacle_cnt += 1
                    else:
                        self.static_obstacle_cnt -= 1
            else:
                self.static_obstacle_cnt -= 1

            # 완전히 차선 폭에 들어오는 장애물이 몇번 연속으로 찍힌다? cnt++ -> 특정 숫자 이상이면 그러면 아예 정적 모드
            # 하지만 계속 안들어오면 비례하게 cnt--

            if self.static_obstacle_cnt < 0:
                self.static_obstacle_cnt = 0
            elif self.static_obstacle_cnt > 30:
                self.static_obstacle_cnt = 30


            if self.state == 'L':
                if self.static_obstacle_cnt == 30:
                    # gt heading
                    self.gt_heading = self.heading

                    # gt heading 음수면 -> heading이 증가하는 방향 -> 왼쪽으로 회피
                    if self.gt_heading < 0:
                        self.avoid_heading = self.gt_heading + 30
                        self.return_heading = self.gt_heading - 15

                    # gt heading 양수면 -> heading이 감소하는 방향 -> 오른으로 회피
                    elif self.gt_heading >= 0:
                        self.avoid_heading = self.gt_heading - 30
                        self.return_heading = self.gt_heading + 15

                    # flag
                    self.state = 'A'
                    self.is_static = True

            elif self.state == 'A':
                # gt heading 음수면 -> heading이 증가하는 방향 -> 왼쪽으로 회피
                if self.gt_heading < 0:
                    if (self.avoid_heading > self.heading): # 목표 heading에 도달하지 못했으면 좌조향
                        self.angle = -50 * abs(self.heading - self.avoid_heading)
                        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                    else:
                        self.state = 'R'

                # gt heading 양수면 -> heading이 감소하는 방향 -> 오른으로 회피
                elif self.gt_heading >= 0:
                    if (self.avoid_heading < self.heading): # 목표 heading에 도달하지 못했으면 우조향
                        self.angle = 50 * abs(self.heading - self.avoid_heading)
                    else:
                        self.state = 'R'

            elif self.state == 'R':
                # gt heading 음수면 -> heading이 증가하는 방향 -> 왼쪽으로 회피 -> 오른쪽으로 return
                if self.gt_heading < 0:
                    if (self.return_heading < self.heading): # 목표 heading에 도달하지 못했으면 우조향
                        self.angle = 25 * abs(self.heading - self.return_heading)
                    else:
                        self.state = 'L'
                        self.gt_heading = None
                        self.avoid_heading = None
                        self.return_heading = None
                        self.static_obstacle_cnt = 0
                        self.is_static = False

                # gt heading 양수면 -> heading이 감소하는 방향 -> 오른으로 회피 -> 왼쪽으로 return
                elif self.gt_heading >= 0:
                    if (self.return_heading > self.heading): # 목표 heading에 도달하지 못했으면 좌조향
                        self.angle = -25 * abs(self.heading - self.return_heading)
                    else:
                        self.state = 'L'
                        self.gt_heading = None
                        self.avoid_heading = None
                        self.return_heading = None
                        self.static_obstacle_cnt = 0
                        self.is_static = False
            

            # 정적 모드를 들어감과 동시에 현재의 heading을 gt heading으로 정하기. 
            
            # 일정 기준 heading을 달성할 때 까지 오른쪽으로 쭉 가기 

            # 다시 일정 기준 heading을 만족하도록 돌아오게 하기. 


            print('STATE: ', self.state)
            print('CNT  : ', self.static_obstacle_cnt)
            print('FLAG : ', self.is_static)
            print('GT   : ', self.gt_heading)
            print('AVOID: ', self.avoid_heading)
            print('RETUR: ', self.return_heading)
            print('HEADI: ', self.heading)
            print()


            self.publishCtrlCmd(self.speed, self.angle, self.is_static)


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

    def headingCB(self, msg):
        self.heading = msg.data

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
