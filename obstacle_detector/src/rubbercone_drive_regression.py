#!/usr/bin/env python
#-*- coding: utf-8 -*-


import rospy
import math
from time import sleep
from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32, Float32
from xycar_msgs.msg import xycar_motor


import numpy as np
from scipy.spatial import distance
from scipy.stats import linregress
import math

motor = None

class Rabacon_drive:
    def __init__(self):
        rospy.Subscriber("raw_obstacles_rubbercone", Obstacles, self.Rabacon)
        self.angle = 0.0
        self.previous_midpoint = None

    def Rabacon(self, data):
        tuned_data = []
        for i in data.circles:
            if math.sqrt((i.center.x) ** 2 +  (i.center.y) ** 2) >= 0.8:
                continue
            tuned_data.append([i.center.x, i.center.y])
        tuned_data = np.array(tuned_data)    
        eps = 0.6
        min_samples = 2
        labels = self.dbscan(tuned_data, eps, min_samples)

        group1 = tuned_data[labels == 0]
        group2 = tuned_data[labels == 1]
        print("Length:", len(group1), len(group2), end=" ")
        # 각 그룹의 각도 계산
        if len(group1) > 1:  # 최소 2개 이상의 점이 있어야 각도 계산 가능
            angle_group1 = self.calculate_angle(group1)

        #     print(f"Group 1 Angle: {angle_group1} degrees")
        else:
            angle_group1 = None
        #     print("Group 1 has insufficient points for angle calculation.")

        if len(group2) > 1:  # 최소 2개 이상의 점이 있어야 각도 계산 가능
            angle_group2 = self.calculate_angle(group2)
        #     print(f"Group 2 Angle: {angle_group2} degrees")
        else:
            angle_group2 = None
        #     print("Group 2 has insufficient points for angle calculation.")

        if angle_group1 is not None and angle_group2 is not None:
            angle = (angle_group1 + angle_group2) // 2
            self.angle = -0.8 * angle
        elif angle_group1 is not None:
            self.angle = -0.8 * angle_group1
        elif angle_group2 is not None:
            self.angle = -0.8 * angle_group2
        else:
            self.angle = None

    def dbscan(self, data, eps, min_samples):
        labels = -1 * np.ones(len(data), dtype=int)
        cluster_id = 0
        
        for i in range(len(data)):
            if labels[i] != -1:
                continue
            
            neighbors = self.find_neighbors(data, i, eps)
            
            if len(neighbors) < min_samples:
                labels[i] = -1
            else:
                self.expand_cluster(data, labels, i, neighbors, cluster_id, eps, min_samples)
                cluster_id += 1

        return labels

    def find_neighbors(self, data, idx, eps):
        neighbors = []
        for i in range(len(data)):
            if distance.euclidean(data[idx], data[i]) <= eps:
                neighbors.append(i)
        return neighbors

    def expand_cluster(self, data, labels, idx, neighbors, cluster_id, eps, min_samples):
        labels[idx] = cluster_id
        i = 0
        
        while i < len(neighbors):
            neighbor_idx = neighbors[i]
            
            if labels[neighbor_idx] == -1:
                labels[neighbor_idx] = cluster_id
            elif labels[neighbor_idx] == -1:
                labels[neighbor_idx] = cluster_id
            else:
                i += 1
                continue
            
            new_neighbors = self.find_neighbors(data, neighbor_idx, eps)
            if len(new_neighbors) >= min_samples:
                neighbors = neighbors + new_neighbors
            i += 1

    def calculate_angle(self, cones):
        # x와 y 값 분리
        x_vals = cones[:, 0]
        y_vals = cones[:, 1]

        # 선형 회귀를 통해 기울기 계산
        slope, intercept, r_value, p_value, std_err = linregress(x_vals, y_vals)
        
        # 기울기를 각도로 변환
        angle_rad = math.atan(slope)
        angle_deg = math.degrees(angle_rad)

        # 각도의 이상치 제거
        filtered_cones = self.filter_outliers(cones)
        
        if len(filtered_cones) < 2:
            return angle_deg

        # 이상치 제거된 점들을 기반으로 각도 다시 계산
        x_vals = filtered_cones[:, 0]
        y_vals = filtered_cones[:, 1]
        slope, intercept, r_value, p_value, std_err = linregress(x_vals, y_vals)
        angle_rad = math.atan(slope)
        angle_deg = math.degrees(angle_rad)
        
        return angle_deg

    def filter_outliers(self, cones):
        x_vals = cones[:, 0]
        y_vals = cones[:, 1]

        slope, intercept, r_value, p_value, std_err = linregress(x_vals, y_vals)
        angle_rad = math.atan(slope)
        angle_deg = math.degrees(angle_rad)

        # IQR을 이용한 이상치 제거
        q1 = np.percentile(angle_deg, 25)
        q3 = np.percentile(angle_deg, 75)
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr

        filtered_cones = cones[(angle_deg >= lower_bound) & (angle_deg <= upper_bound)]
        return filtered_cones


    def drive(self, angle, speed=4):
        global motor
        motor_msg = xycar_motor()
        motor_msg.angle = round(angle)
        motor_msg.speed = round(speed)
        motor.publish(motor_msg)


    def start(self):
        rospy.init_node('my_driver')
        rabacon = Rabacon_drive()  

        global motor
        motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

        prev_angle = 0
        while not rospy.is_shutdown():

            angle_to_drive = rabacon.angle
            if angle_to_drive is None:
                angle_to_drive = prev_angle
            else:
                angle_to_drive = round(angle_to_drive)
                prev_angle = angle_to_drive
            print("angle:", angle_to_drive)
            speed = 4
            self.drive(angle_to_drive, speed)  
            sleep(0.05)  

if __name__ == '__main__':
    rabacon = Rabacon_drive()
    rabacon.start()