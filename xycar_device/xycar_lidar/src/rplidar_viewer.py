#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : rplidar_viewer.py
# 버 전 : xavier.411.py3.1_0
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy
import numpy as np
import time
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan

class rplidar:

    S_ru = [0,0]
    S_ld = [0,0]

    def __init__(self, nodeName, topic, queue_size, ld, ru):
        self.topic = topic
        self.queue_size = queue_size
        self.S_ru = ru
        self.S_ld = ld

        #self.setting = [505, np.sin, np.cos, -1, 1]
        self.setting = [720, np.sin, np.cos, 1, -1]

        rospy.init_node(nodeName, anonymous=False)
        plt.ion()
            
    def sub_start(self):
        print(self.topic, self.queue_size)
        rospy.Subscriber(self.topic, LaserScan, self.callback, queue_size=self.queue_size)

    def detect_degree_func(self, x, y):
        OK = 0

        if (self.S_ld[0] <= x <= self.S_ru[0]) or (self.S_ru[0] <= x <= self.S_ld[0]):
            OK += 1
        if (self.S_ld[1] <= y <= self.S_ru[1]) or (self.S_ru[1] <= y <= self.S_ld[1]):
            OK += 1

        if OK == 2:
            return True

        return False

    def callback(self, data):
        ran = data.ranges
        lidar_increment = data.angle_increment
        mode = 0
        #if len(ran) == 720:
        #    mode = 1
        #elif len(ran) == 360:
        #    mode = 2
        #else:
        #    return

        if len(ran) != self.setting[0]:
            return

        X = []
        Y = []
        self.detect_degrees = []

        for i in range(0, self.setting[0]):
            radian = i * lidar_increment
            #x = (-1) * self.lidar_array[i] * np.cos(rad)
            #y = self.lidar_array[i] * np.sin(rad)
            #radian = np.radians(degree)

            x = ran[i] * self.setting[1](radian) * self.setting[3]
            y = ran[i] * self.setting[2](radian) * self.setting[4]

            if self.detect_degree_func(x, y):
                self.detect_degrees.append(np.degrees(radian))
 
            X.append(x)
            Y.append(y)
            
        X_graph = np.array(Y)
        Y_graph = np.array(X)
        
        xlim = [self.S_ld[0], self.S_ru[0]]
        ylim = [self.S_ld[1], self.S_ru[1]]

        plt.xlim([min(xlim), max(xlim)])
        plt.ylim([min(ylim), max(ylim)])

        if len(self.detect_degrees) != 0:
            print(self.detect_degrees)

        dot = plt.scatter(X_graph,Y_graph) 

        plt.show()
        plt.pause(0.001)
        dot.remove()

if __name__ == '__main__':
    lidar = rplidar("test", "/scan", 1, [-0.25, 0],[0.25, 0.25])
    lidar.sub_start()
    rospy.spin()

