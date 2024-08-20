#!/usr/bin/env python3

import rospy

from xycar_msgs.msg import xycar_motor  # xycar 모터 메시지 모듈 임포트

from ar_track_alvar_msgs.msg import AlvarMarkers

from std_msgs.msg import Int64MultiArray

import math

import time

import os

class ArTag:
    def __init__(self, marker):
        self.x = marker.pose.pose.position.x
        self.y = marker.pose.pose.position.y
        self.z = marker.pose.pose.position.z

        self.q_x = marker.pose.pose.orientation.x
        self.q_y = marker.pose.pose.orientation.y
        self.q_z = marker.pose.pose.orientation.z
        self.q_w = marker.pose.pose.orientation.w


class ArTagDriver:
    def __init__(self):
        rospy.init_node('ar_tag_driver_node', anonymous=True)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.arCB, queue_size= 1)
        rospy.Subscriber('/traffic_light', Int64MultiArray, self.trafficCB, queue_size= 1)

        rospy.Subscriber('/red_center', Int64MultiArray, self.red_centers_CB, queue_size= 1)
        rospy.Subscriber('/green_center', Int64MultiArray, self.green_centers_CB, queue_size= 1)

        self.ctrl_cmd_pub = rospy.Publisher('/xycar_motor_ar', xycar_motor, queue_size=1)

        self.rate = rospy.Rate(30)  # 30hz

        self.ctrl_cmd_msg_ar = xycar_motor()

        # for ar tag
        self.sorted_ar_list = []
        self.k = 12.5
        self.l = -30
        self.l2 = 10

        # 신호등 추종 기준
        self.k_traffic = 0.003

        self.angle_cal_traffic = 35

        # AR 태그 추종 기준
        self.angle_cal_x = 0.2 # 0.017

        self.speed = 0
        self.flag = True
        self.angle = 0

        self.heading = None


        self.is_traffic_passed = False
        # 신호등 안되어서 강제로 이후로 점프
        # self.is_traffic_passed = True
        

        self.closest_ar = None

        self.ar_heading_threshold = 90.0


        self.ar_z_threshold_min = 0.0
        self.ar_z_threshold = 6.0 # 일정 거리 이내의 ar 태그만 인식하게 하기 위함.


        self.stop_distance = 2.5

        self.is_stopped = False

        self.green_light_count = 0
        self.red_light_count = 0

        self.traffic_light = None

        self.red_centers = []
        self.green_centers = []

        self.ar_after_traffic_flag = False
        self.ar_tag_not_detected_count = 0



    def run(self):
        while not rospy.is_shutdown():

            # 신호등 분기점 지났는지만 판단
            if (self.is_stopped == True) and (self.traffic_light == 'Green'):
                self.is_traffic_passed = True

            # ------------------------------------------- 신호등 통과 전 ------------------------------------------- #
            if self.is_traffic_passed == False:

                # 신호등 아직 안지났는데 한번이라도 멈춤 플래그 세워졌으면 그냥 정지해있기.
                if self.is_stopped == True:
                    self.publishCtrlCmd(0, self.angle, self.flag)
                    continue

                # 태그 인식 되면 첫번째 태그이므로 정보를 통해 이동
                if len(self.sorted_ar_list) > 0:
                    
                    # 일정 거리 이내로 들어오고 + 빨간 불이면 멈추기
                    if (self.closest_ar.z < self.stop_distance) and (self.traffic_light == 'Red'):
                        self.publishCtrlCmd(0, self.angle, self.flag)
                        self.is_stopped = True
                        continue
                    
                    # 신호등 보고 따라가기
                    if len(self.red_centers) > 0:
                        self.angle = int(math.degrees(self.k_traffic * (self.red_centers[0] + self.angle_cal_traffic - 320)))
                    elif len(self.green_centers) > 0:
                        self.angle = int(math.degrees(self.k_traffic * (self.green_centers[0] - 33 - 320)))

                    # ar 태그 보며 따라가는 기준
                    else:
                        self.angle = int(math.degrees(5 * (self.closest_ar.x + 0.025) / (self.closest_ar.z)))


                # 태그가 인식되지 않으면 그냥 좌조향
                elif len(self.sorted_ar_list) == 0: 
                    self.angle = -20
            # ------------------------------------------- 신호등 통과 전 ------------------------------------------- #



            # self.flag 를 언제 False 로 바꿔 주느냐..? 
            # 신호등 통과 이후 한번이라도 ar을 본 이후로 몇번 연속이나 ar 을 못보면 lane으로 이동 
            # ------------------------------------------- 신호등 통과 이후 ------------------------------------------- #
            elif self.is_traffic_passed == True:
                if len(self.green_centers) > 0:
                    self.angle = int(math.degrees(self.k_traffic * (self.green_centers[0] - 33 - 320)))

                else:
                    # ar 태그를 추종하며 따라가기
                    if len(self.sorted_ar_list) > 0:

                        if self.closest_ar.z < 1.5:
                            self.angle = int(math.degrees(18.75 * (self.closest_ar.x - 0.19) / (self.closest_ar.z)))

                        else:
                            self.angle = int(math.degrees(18.75 * (self.closest_ar.x - 0.05) / (self.closest_ar.z)))


                        # 신호등 이후 AR 을 인지 했는지를 점검 -> 인지가 특정 프레임 이상 안되면 다음 미션으로 넘기기 위함.
                        if self.ar_after_traffic_flag == False:
                            self.ar_after_traffic_flag = True

                    elif len(self.sorted_ar_list) == 0: 

                        self.angle = -2
            # ------------------------------------------- 신호등 통과 이후 ------------------------------------------- #



            # ------------------------------------------- 미션 상태 관리 ------------------------------------------- #
            if self.ar_after_traffic_flag == True:
                if len(self.sorted_ar_list) == 0:
                    self.ar_tag_not_detected_count += 1
                else:
                    self.ar_tag_not_detected_count = 0
            
            ar_tag_not_detected_count_threshold = 20
            if self.ar_tag_not_detected_count > ar_tag_not_detected_count_threshold:
                self.ar_tag_not_detected_count = ar_tag_not_detected_count_threshold
            
                if self.ar_tag_not_detected_count >= ar_tag_not_detected_count_threshold:

                    # os.system('rosnode kill /ar_track_alvar')
                    # os.system('rosnode kill /traffic_detection_node')

                    # stop_time = time.time()
                    # while time.time() - stop_time < 2.2:
                    #     self.publishCtrlCmd(0, 0, self.flag)

                    # go_time = time.time()
                    # while time.time() - go_time < 1.0:
                    #     self.publishCtrlCmd(7, 0, self.flag)

                    self.flag = False

            # ------------------------------------------- 미션 상태 관리 ------------------------------------------- #


            self.speed = 5
            self.publishCtrlCmd(self.speed, self.angle, self.flag)

            self.rate.sleep()


    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg_ar.speed = int(motor_msg)  # 모터 속도 설정
        self.ctrl_cmd_msg_ar.angle = int(servo_msg)  # 조향각 설정
        self.ctrl_cmd_msg_ar.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg_ar)  # 명령 퍼블리시

    def arCB(self, msg):

        self.sorted_ar_list = []

        if not msg.markers:
            return
        # Calculate distances and sort markers by distance
        for marker in msg.markers:
            new_ar = ArTag(marker)
            if self.ar_z_threshold_min <= new_ar.z <= self.ar_z_threshold: # 일정 거리 이내에 있는 ar 태그만 append
                self.sorted_ar_list.append(new_ar)

        if len(self.sorted_ar_list) > 0:
            # 처음에는 distance를 position.x, y, z를 통해 구해야 한다고 생각했었는데, just position.z값 만으로 정렬
            self.sorted_ar_list.sort(key=lambda x: x.z)

            # 가장 가까운 ar tag 
            self.closest_ar = self.sorted_ar_list[0]

        else:
            self.closest_ar = None

    def trafficCB(self, msg):
        self.red_light_count = msg.data[0]
        self.green_light_count = msg.data[1]

        if (self.red_light_count >= 300) and (self.green_light_count <= 5):
            self.traffic_light = 'Red'
        elif self.green_light_count >= 300:
            self.traffic_light = 'Green'
        else:  
            self.traffic_light = ''

    def red_centers_CB(self, msg):
        self.red_centers = msg.data

    def green_centers_CB(self, msg):
        self.green_centers = msg.data
    
if __name__ == '__main__':
    try:
        node = ArTagDriver()
        node.run()
    except rospy.ROSInterruptException:
        pass
