#!/usr/bin/env python3

import rospy

from xycar_msgs.msg import xycar_motor  # xycar 모터 메시지 모듈 임포트

from ar_track_alvar_msgs.msg import AlvarMarkers

from std_msgs.msg import Float64, Int64MultiArray

import math

class ArTag:
    def __init__(self, marker):
        self.x = marker.pose.pose.position.x
        self.y = marker.pose.pose.position.y
        self.z = marker.pose.pose.position.z

class ArTagDriver:
    def __init__(self):
        rospy.init_node('ar_tag_driver_node', anonymous=True)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.arCB, queue_size= 1)
        rospy.Subscriber('/heading', Float64, self.headingCB, queue_size= 1)
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

        self.heading = 0.0

        self.is_traffic_passed = False

        self.closest_ar = None

        self.ar_heading_threshold = 90.0


        self.ar_z_threshold_min = 0.0
        self.ar_z_threshold = 6.0 # 일정 거리 이내의 ar 태그만 인식하게 하기 위함.


        self.stop_distance = 2.2

        self.is_stopped = False

        self.green_light_count = 0
        self.red_light_count = 0

        self.traffic_light = None

        self.red_centers = []
        self.green_centers = []



    def run(self):
        while not rospy.is_shutdown():

            # 신호등 분기점 지났는지만 판단
            if (self.is_stopped == True) and (self.traffic_light == 'Green'):
                self.is_traffic_passed = True

            # 신호등 통과 전
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
                        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    elif len(self.green_centers) > 0:
                        self.angle = int(math.degrees(self.k_traffic * (self.green_centers[0] - 33 - 320)))
                        print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

                    # ar 태그 보며 따라가는 기준
                    else:
                        self.angle = int(math.degrees(5 * (self.closest_ar.x + 0.025) / (self.closest_ar.z)))
                        print("******************************************************")


                # 태그가 인식되지 않으면 heading 값을 통해 이동
                elif len(self.sorted_ar_list) == 0: 
                    print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
                    self.angle = -(self.ar_heading_threshold - self.heading)



            ############ 신호등을 기점으로 차이를 두기 ############## 

            elif self.is_traffic_passed == True:

                # 신호등 보고 따라가기
                if len(self.red_centers) > 0:
                    self.angle = int(math.degrees(self.k_traffic * (self.red_centers[0] + self.angle_cal_traffic - 320)))
                    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                elif len(self.green_centers) > 0:
                    self.angle = int(math.degrees(self.k_traffic * (self.green_centers[0] - 33 - 320)))
                    print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

                else:
                    # ar 태그를 추종하며 따라가기
                    if len(self.sorted_ar_list) > 0:
                        print("******************************************************")
                        self.angle = int(math.degrees(10 * (self.closest_ar.x - self.angle_cal_x) / (self.closest_ar.z)))

                    # 태그가 인식되지 않으면 heading 값을 통해 이동
                    elif len(self.sorted_ar_list) == 0: 
                        if (self.heading <= -170) or (self.heading >= 170):

                            self.flag = False
                        print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")

                        self.angle = -(self.ar_heading_threshold - self.heading)



            self.speed = 4
            self.publishCtrlCmd(self.speed, self.angle, self.flag)
            print('')


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

        self.flag = True
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


    def headingCB(self, msg):
        self.heading = msg.data

    def trafficCB(self, msg):
        self.red_light_count = msg.data[0]
        self.green_light_count = msg.data[1]

        if (self.green_light_count >= 300) and (self.red_light_count <= 5):
            self.traffic_light = 'Green'
        else:  
            self.traffic_light = 'Red'

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
