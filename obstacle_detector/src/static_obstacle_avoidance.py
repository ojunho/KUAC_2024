#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import math

class StaticAvoidance():
    def __init__(self):
        rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacleCB)
        rospy.Subscriber("/xycar_ultrasonic", Int32MultiArray, self.ultrasonicCB)
        self.obstacles = []
        self.ultrasonic = [] # 왼쪽, 오른쪽, 오른쪽뒤, 뒤, 왼쪽뒤
        self.is_static = False
        self.steer = 0
        self.is_left = False

        self.static_pub = rospy.Publisher("static_steer", Int32, queue_size=5)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print("=======================")
            print(f"{'static_obstacle' if self.is_static else 'lane_detection'}")
            if self.is_static: # 장애물 주행 하다가
                if len(self.obstacles) == 0: # 장애물이 안 잡히면
                    print("초음파 활성화", end=" ")
                    if self.is_left and self.ultrasonic[0] < 25:
                        print("왼쪽에 아직 장애물 있음")
                    elif not self.is_left and self.ultrasonic[1] < 20:
                        print("오른쪽에 아직 장애물 있음")
                        self.steer = 0
                    else: # 장애물이 주변에 없음 - 정적 장애물 미션 끝
                        self.is_static = False
            else: # 일반 주행 하다가
                if len(self.obstacles) > 0:
                    self.is_static = True
                    tangent = self.obstacles[0][1] / self.obstacles[0][0]
                    degree = math.atan(tangent) * 180 / math.pi
                    self.is_left = True if degree > 0 else False
                    print(f"장애물 각도:{degree}")
                    if self.is_left: # 왼쪽 장애물
                        self.steer = 30
                    else:
                        self.steer = -30
            print(f"조향각:{self.steer}")

            self.static_pub.publish(self.steer)

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


if __name__ == '__main__':
    rospy.init_node('static_obstacle_avoidance', anonymous=True)
    try:
        static_obstacle_avoidance = StaticAvoidance()
    except rospy.ROSInterruptException:
        pass
