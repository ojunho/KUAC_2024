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
        self.previous_right_rabacon = None
        self.previous_left_rabacon = None

    def Rabacon(self, data):
        left_rabacon = []
        right_rabacon = []
        print("go")
        for i in data.circles:
            # roi 설정
            # if i.center.x > -1.5 and i.center.x < -0.05:
            if i.center.x > 0 and i.center.x < 1:
                if i.center.y > 0 and i.center.y < 0.7:
                    left_rabacon.append(i)
                
                elif i.center.y < 0 and i.center.y > -0.7:
                    right_rabacon.append(i)

        # -----------------------------------------------------------#
        # 잘못된 라바콘이 들어가 있는지 확인
        left_rabacon = self.check_cone_consistency(left_rabacon)
        right_rabacon = self.check_cone_consistency(right_rabacon)
        # -----------------------------------------------------------#

        if  len(left_rabacon) > 0 and len(right_rabacon) > 0:

            if len(left_rabacon) < 2 or len(right_rabacon) < 2:
                left_close_rabacon = sorted(left_rabacon, key = lambda x : -x.center.x)[0]
                right_close_rabacon = sorted(right_rabacon, key = lambda x : -x.center.x)[0]
                avg_x_rabacon = (left_close_rabacon.center.x + right_close_rabacon.center.x) / 2
                avg_y_rabacon = (left_close_rabacon.center.y + right_close_rabacon.center.y) 
                self.angle = -(avg_y_rabacon * 200) # 150
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
                print('angle:',self.angle)


                # Update previous cones
                self.previous_left_rabacon = left_close_rabacon
                self.previous_right_rabacon = right_close_rabacon


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
                self.angle = -(closest_midpoint[1] * 150 )
                self.previous_midpoint = closest_midpoint     

                # print("rabacon_drive")
                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                print('angle:',self.angle)

        elif len(left_rabacon) == 0  and  len(right_rabacon) == 0: 
                print("no_rabacon")
            

        else:
            # 라바콘이 잘 인식된 방향의 라바콘을 기준으로 이전의 라바콘과의 각도를 계산하여
            # (이전 라바콘과 현재 라바콘의 각도(조향각)) 안 보이는 쪽의 이전 라바콘에 같은 각도로 라바콘을 생성해서 중점을 잡기!!!

            if len(right_rabacon) == 0 and self.previous_right_rabacon:
                left_close_rabacon = sorted(left_rabacon, key=lambda x: -x.center.x)[0]  # 가장 가까운 라바콘 선택
                angle = self.calculate_angle(left_close_rabacon, self.previous_right_rabacon)
                distance = self.calculate_distance(left_close_rabacon, self.previous_right_rabacon)
                right_close_rabacon = self.virtual_cone(left_close_rabacon, angle, distance)
                self.previous_left_rabacon = left_close_rabacon 
                mid_point = self.calculate_midpoint(left_close_rabacon, right_close_rabacon)
                
            elif len(left_rabacon) == 0 and self.previous_left_rabacon:
                right_close_rabacon = sorted(right_rabacon, key=lambda x: -x.center.x)[0]
                angle = self.calculate_angle(right_close_rabacon, self.previous_left_rabacon)
                distance = self.calculate_distance(right_close_rabacon, self.previous_left_rabacon)
                left_close_rabacon = self.virtual_cone(right_close_rabacon, angle, distance)
                self.previous_right_rabacon = right_close_rabacon 
                mid_point = self.calculate_midpoint(left_close_rabacon, right_close_rabacon)

            self.angle = -(mid_point[1] * 150)
            self.previous_midpoint = mid_point
            print("angle:", self.angle)
   

        print('left_rabacon: ',len(left_rabacon))
        print('right_rabacon: ',len(right_rabacon))


    # -----------------------------------------------------------#
    def check_cone_consistency(self, rabacon_list):

        if len(rabacon_list) < 2:
            
            return rabacon_list

        valid_rabacons = [] 

        # 두 라바콘 쌍 검사
        for i in range(len(rabacon_list) - 1):
            for j in range(i + 1, len(rabacon_list)): 
                distance = self.calculate_distance(rabacon_list[i], rabacon_list[j])
                angle_diff = self.calculate_angle(rabacon_list[i], rabacon_list[j])

                if distance < 1.5 and angle_diff < 10:  # 파라미터 정해야 함
                    valid_rabacons.append(rabacon_list[i])
                    valid_rabacons.append(rabacon_list[j])

        # 유효한 라바콘 리스트에서 중복 제거
        unique_rabacons = []
        seen = set()
        for rabacon in valid_rabacons:
            identifier = (rabacon.center.x, rabacon.center.y)
            if identifier not in seen:
                unique_rabacons.append(rabacon)
                seen.add(identifier)

        return unique_rabacons

    # -----------------------------------------------------------#

    def calculate_midpoint(self, point1, point2):
        mid_x = (point1.center.x + point2.center.x) / 2
        mid_y = (point1.center.y + point2.center.y) / 2
        return (mid_x, mid_y)

    def find_closest_midpoint(self, midpoints):
        if self.previous_midpoint is None:
            return midpoints[0]
        # 현재 중점 중에서 이전 중점과 가장 가까운 중점을 선택
        chosen_midpoint = min(midpoints, key=lambda p: (p[0] - self.previous_midpoint[0]) ** 2 + (p[1] - self.previous_midpoint[1]) ** 2)
        return chosen_midpoint

    def adjust_midpoint(self, closest, midpoints):
        # 가장 가까운 점이 벗어날 경우, 이전 중점과 나머지 두 중점의 평균 x 좌표를 사용하여 새로운 중점 설정
        second_closest, third_closest = self.get_second_and_third_closest(midpoints, closest)
        if self.previous_midpoint is None:
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


    def calculate_angle(self, cone1, cone2):
        return math.atan2(cone1.center.y - cone2.center.y, cone1.center.x - cone2.center.x)

    def calculate_distance(self, cone1, cone2):
        return math.sqrt((cone1.center.x - cone2.center.x) ** 2 + (cone1.center.y - cone2.center.y) ** 2)


    # def virtual_cone(self, rabacon, angle, distance):
    #     virtual_x = rabacon.center.x + distance * math.cos(angle)
    #     virtual_y = rabacon.center.y + distance * math.sin(angle)
        
    #     virtual_circle = Obstacles()
    #     circle = Obstacles.Circle()
    #     circle.center.x = virtual_x
    #     circle.center.y = virtual_y
    #     circle.radius = rabacon.radius  
        
    #     virtual_circle.circles.append(circle)
        
    #     return circle

    # -----------------------------------------------------------#
    # -----------------------------------------------------------#

    def drive(self, angle, speed):
        global motor
        motor_msg = xycar_motor()
        motor_msg.angle = angle 
        motor_msg.speed = speed
        motor.publish(motor_msg)
        
    def start(self):
        rospy.init_node('my_driver')
        rabacon = Rabacon_drive()  

        global motor
        motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

        while not rospy.is_shutdown():

            angle_to_drive = round(rabacon.angle)
            # speed = round(max(8, 15 - abs(angle_to_drive)/3))

            speed = 4
            self.drive(angle_to_drive, speed)  # Example: Drive with a speed of 0.5
            sleep(0.1)  # Example: Sleep for 0.1 seconds to control the update rate


if __name__ == '__main__':
    rabacon = Rabacon_drive()
    rabacon.start()