#!/usr/bin/env python3

import rospy

from xycar_msgs.msg import xycar_motor  # xycar 모터 메시지 모듈 임포트

from ar_track_alvar_msgs.msg import AlvarMarkers

from std_msgs.msg import Float64

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



        self.ctrl_cmd_pub = rospy.Publisher('/xycar_motor_ar', xycar_motor, queue_size=1)

        self.rate = rospy.Rate(30)  # 30hz

        self.ctrl_cmd_msg_ar = xycar_motor()

        # for ar tag
        self.sorted_ar_list = []
        self.k = 2
        self.l = 30
        self.angle_cal = 0.017

        self.speed = 0
        self.flag = True
        self.angle = 0

        self.heading = 0.0

        self.is_traffic_passed = False

        self.closest_ar = None

        self.ar_heading_threshold = -90.0

        self.ar_z_threshold = 6.0 # 일정 거리 이내의 ar 태그만 인식하게 하기 위함.

        self.traffic_light = -1

        self.stop_distance = 3.0

        self.is_stopped = False


    def run(self):
        while not rospy.is_shutdown():

            print('AR리스트: ', self.sorted_ar_list)
            print('가까운AR: ', self.closest_ar)
            print('로봇헤딩:  ', self.heading)

            print('')

            self.speed = 4
            self.publishCtrlCmd(self.speed, self.angle, self.flag)
            self.rate.sleep()

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg_ar.speed = motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg_ar.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_msg_ar.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg_ar)  # 명령 퍼블리시

    def arCB(self, msg):

        self.sorted_ar_list = []

        if not msg.markers:
            self.flag = False
            return

        self.flag = True
        # Calculate distances and sort markers by distance
        for marker in msg.markers:
            new_ar = ArTag(marker)
            if new_ar.z <= self.ar_z_threshold: # 일정 거리 이내에 있는 ar 태그만 append
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


    
if __name__ == '__main__':
    try:
        node = ArTagDriver()
        node.run()
    except rospy.ROSInterruptException:
        pass
