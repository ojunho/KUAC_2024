#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
import math
import time

class StaticAvoidance():
    def __init__(self):
        rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacleCB)
        rospy.Subscriber("/xycar_ultrasonic", Int32MultiArray, self.ultrasonicCB)
        self.state = 'LANE_DETECTION'
        self.obstacles = []
        self.ultrasonic = [] # 왼쪽, 오른쪽, 오른쪽뒤, 뒤, 왼쪽뒤
        self.is_static = False
        self.steer = 0
        self.is_left = False

        self.steer_right_time = 0
        self.steer_left_time = 0
        self.final_adjust_time = 0

        self.static_pub = rospy.Publisher("static_steer", Int32, queue_size=5)
        self.ctrl_cmd_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == 'LANE_DETECTION':
                print("=======================")
                print("lane_detection")
                if len(self.obstacles) > 0 and math.sqrt(self.obstacles[0][0] ** 2 + self.obstacles[0][1] ** 2) < 1.0:
                    self.state = 'OBSTACLE_DETECTED'
                    self.is_static = True
            
            elif self.state == 'OBSTACLE_DETECTED':
                print("static_obstacle")
                start_time = time.time()
                self.state = 'STEER_RIGHT'
                self.steer_right_time = start_time
            
            elif self.state == 'STEER_RIGHT':
                if time.time() - self.steer_right_time < 1:
                    self.steer = 30
                    self.publishCtrlCmd(0, self.steer)
                else:
                    start_time = time.time()
                    self.state = 'STEER_LEFT'
                    self.steer_left_time = start_time
            
            elif self.state == 'STEER_LEFT':
                if time.time() - self.steer_left_time < 1:
                    self.steer = -20
                    self.publishCtrlCmd(0, self.steer)
                else:
                    self.state = 'STRAIGHT'
            
            elif self.state == 'STRAIGHT':
                if self.ultrasonic[0] < 25:
                    self.steer = 0
                    self.publishCtrlCmd(0, self.steer)
                else:
                    start_time = time.time()
                    self.state = 'FINAL_ADJUST'
                    self.final_adjust_time = start_time
            
            elif self.state == 'FINAL_ADJUST':
                if time.time() - self.final_adjust_time < 0.5:
                    self.steer = -10
                    self.publishCtrlCmd(0, self.steer)
                else:
                    self.state = 'LANE_DETECTION'
                    self.is_static = False


            # self.static_pub.publish(self.steer)
            print("현재 미션", self.state)
            print("조향:", self.steer)
            rate.sleep()
                        

    def obstacleCB(self, msg):
        obstacles = []
        for circle in msg.circles:
            x = round(circle.center.x, 3)
            y = round(circle.center.y, 3)
            obstacles.append([x, y])

        if len(obstacles) > 0:
            self.obstacles = sorted(obstacles, key=lambda c: math.sqrt(c[0]**2 + c[1]**2))
        else:
            self.obstacles = []

    def ultrasonicCB(self, msg):
        ultrasonic = list(msg.data)
        ultrasonic.pop(1)
        ultrasonic.pop(1)
        ultrasonic.pop(1)
        self.ultrasonic = ultrasonic

    def publishCtrlCmd(self, motor_msg, servo_msg):
        self.ctrl_cmd_msg.speed = 0  # 모터 속도 설정
        self.ctrl_cmd_msg.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)  # 명령 퍼블리시


if __name__ == '__main__':
    rospy.init_node('static_obstacle_avoidance', anonymous=True)
    try:
        static_obstacle_avoidance = StaticAvoidance()
    except rospy.ROSInterruptException:
        pass
