#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
from time import sleep
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32, Float32
from xycar_msgs.msg import xycar_motor

motor = None

class Rabacon_drive:
    
    def __init__(self):
        rospy.Subscriber("raw_obstacles", Obstacles, self.Rabacon)
        self.angle = 0.0
        self.previous_midpoint = None 

    def Rabacon(self, data):
        left_rabacon = []
        right_rabacon = []
        print("go")
        for i in data.circles:
            # roi 설정
            # if i.center.x > -1.5 and i.center.x < -0.05:
            if i.center.x > 0 and i.center.x < 10:
                if i.center.y > 0 and i.center.y < 0.7:
                    left_rabacon.append(i)
                    
                elif i.center.y < 0 and i.center.y > -0.7:
                    right_rabacon.append(i)

        # if  2 > len(left_rabacon) > 0 and 2 > len(right_rabacon) > 0:
        #     left_close_rabacon = sorted(left_rabacon, key = lambda x : -x.center.x)[0]
        #     right_close_rabacon = sorted(right_rabacon, key = lambda x : -x.center.x)[0]
        #     avg_x_rabacon = (left_close_rabacon.center.x + right_close_rabacon.center.x) / 2
        #     avg_y_rabacon = (left_close_rabacon.center.y + right_close_rabacon.center.y) 
        #     self.angle = avg_y_rabacon * 150 


# -----------------------------------------------------------#
        # elif len(left_rabacon) >= 2 and len(right_rabacon) >= 2:
        if  len(left_rabacon) > 0 and len(right_rabacon) > 0:

            if len(left_rabacon) < 2 or len(right_rabacon) < 2:
                left_close_rabacon = sorted(left_rabacon, key = lambda x : -x.center.x)[0]
                right_close_rabacon = sorted(right_rabacon, key = lambda x : -x.center.x)[0]
                avg_x_rabacon = (left_close_rabacon.center.x + right_close_rabacon.center.x) / 2
                avg_y_rabacon = (left_close_rabacon.center.y + right_close_rabacon.center.y) 
                self.angle = avg_y_rabacon * 150 
                print('angle:',self.angle)

            else:
                # 가장 가까운 2개의 좌측 라바콘과 2개의 우측 라바콘 선택
                left_close_rabacon = sorted(left_rabacon, key=lambda x: x.center.x)[:2]
                right_close_rabacon = sorted(right_rabacon, key=lambda x: x.center.x)[:2]

                # 좌측 A, B 라바콘과 우측 A, B 라바콘의 중점 계산
                left_mid_a_b = self.calculate_midpoint(left_close_rabacon[0], left_close_rabacon[1])
                right_mid_a_b = self.calculate_midpoint(right_close_rabacon[0], right_close_rabacon[1])

                # 좌측 A 라바콘과 우측 A 라바콘의 중점, 좌측 B 라바콘과 우측 B 라바콘의 중점 계산
                mid_a = self.calculate_midpoint(left_close_rabacon[0], right_close_rabacon[0])
                mid_b = self.calculate_midpoint(left_close_rabacon[1], right_close_rabacon[1])

                midpoints = [left_mid_a_b, right_mid_a_b, mid_a, mid_b]
                closest_midpoint = self.find_closest_midpoint(midpoints)

                # 가장 가까운 점이 다른 중점들과 크게 벗어나는지 확인
                if self.is_outlier(closest_midpoint, midpoints):
                    closest_midpoint = self.adjust_midpoint(closest_midpoint, midpoints)
                
                # y 좌표를 기준으로 각도 계산            
                self.angle = closest_midpoint[1] * 150 
                # 이전 중점 업데이트
                self.previous_midpoint = closest_midpoint     

                print("rabacon_drive")
                print('angle:',self.angle)

        else:
            print("no_rabacon")


        print('left_rabacon: ',len(left_rabacon))
        print('right_rabacon: ',len(right_rabacon))
        



    def calculate_midpoint(self, point1, point2):
        mid_x = (point1.center.x + point2.center.x) / 2
        mid_y = (point1.center.y + point2.center.y) / 2
        return (mid_x, mid_y)

    def find_closest_midpoint(self, midpoints):
        if self.previous_midpoint is None:
            # 이전 중점이 없는 경우, 첫 번째 중점을 선택
            return midpoints[0]
        # 현재 중점 중에서 이전 중점과 가장 가까운 중점을 선택
        chosen_midpoint = min(midpoints, key=lambda p: (p[0] - self.previous_midpoint[0]) ** 2 + (p[1] - self.previous_midpoint[1]) ** 2)
        return chosen_midpoint

    def adjust_midpoint(self, closest, midpoints):
        # 가장 가까운 점이 벗어날 경우, 이전 중점과 나머지 두 중점의 평균 x 좌표를 사용하여 새로운 중점 설정
        second_closest, third_closest = self.get_second_and_third_closest(midpoints, closest)
        if self.previous_midpoint is None:
            # If previous_midpoint is None, use the average of the two closest midpoints
            avg_x = (second_closest[0] + third_closest[0]) / 2
            avg_y = (second_closest[1] + third_closest[1]) / 2
        else:
            avg_x = (self.previous_midpoint[0] + second_closest[0] + third_closest[0]) / 3
            avg_y = (self.previous_midpoint[1] + second_closest[1] + third_closest[1]) / 3
        return (avg_x, avg_y)

    def get_second_and_third_closest(self, midpoints, closest):
        sorted_midpoints = sorted(midpoints, key=lambda p: (p[0] - closest[0]) ** 2 + (p[1] - closest[1]) ** 2)
        return sorted_midpoints[1], sorted_midpoints[2]

    def is_outlier(self, point, midpoints, threshold=0.1):
        distances = [math.sqrt((point[0] - p[0]) ** 2 + (point[1] - p[1]) ** 2) for p in midpoints]
        mean_distance = sum(distances) / len(distances)
        return any(dist > mean_distance + threshold for dist in distances)


# -----------------------------------------------------------#


    def drive(self,angle, speed):
        global motor
        motor_msg = xycar_motor()
        motor_msg.angle = angle 
        motor_msg.speed = speed
        motor.publish(motor_msg)
        
    def start(self):
        rospy.init_node('my_driver')
        rabacon = Rabacon_drive()  # Create an instance of Rabacon_drive class

        global motor
        motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

        while not rospy.is_shutdown():
            # Use the angle value from the Rabacon_drive instance
            angle_to_drive = int(rabacon.angle)
            speed = max(8, 15 - abs(angle_to_drive)/3)
            self.drive(angle_to_drive, speed)  # Example: Drive with a speed of 0.5
            sleep(0.1)  # Example: Sleep for 0.1 seconds to control the update rate


if __name__ == '__main__':
    rabacon = Rabacon_drive()
    rabacon.start()