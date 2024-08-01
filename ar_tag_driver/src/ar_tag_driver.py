#!/usr/bin/env python

import rospy

from xycar_msgs.msg import xycar_motor  # xycar 모터 메시지 모듈 임포트

from ar_track_alvar_msgs.msg import AlvarMarkers

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


        self.ctrl_cmd_pub = rospy.Publisher('/xycar_motor_ar', xycar_motor, queue_size=1)

        self.rate = rospy.Rate(30)  # 30hz

        self.ctrl_cmd_msg_ar = xycar_motor()

        # for ar tag
        self.sorted_ar_list = []
        self.k = 2
        self.l = 30
        self.angle_cal = 0.017

        self.speed = 0
        self.flag = False
        self.angle = 0


    def run(self):
        while not rospy.is_shutdown():
            print('AR Tag self.angle: ', self.angle)
            self.speed = 4
            self.publishCtrlCmd(self.speed, self.angle, self.flag)
            self.rate.sleep()

    def publishCtrlCmd(self, motor_msg, servo_msg, flag):
        self.ctrl_cmd_msg_ar.speed = motor_msg  # 모터 속도 설정
        self.ctrl_cmd_msg_ar.angle = servo_msg  # 조향각 설정
        self.ctrl_cmd_msg_ar.flag = flag
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg_ar)  # 명령 퍼블리시

    def arCB(self, msg):
        if not msg.markers:
            self.flag = False
            return

        self.flag = True
        # Calculate distances and sort markers by distance
        self.sorted_ar_list = []
        for marker in msg.markers:
            self.sorted_ar_list.append(ArTag(marker))

        # 처음에는 distance를 position.x, y, z를 통해 구해야 한다고 생각했었는데, just position.z값 만으로 정렬
        self.sorted_ar_list.sort(key=lambda x: x.z)

        # 가장 가까운 ar tag 
        closest_ar = self.sorted_ar_list[0]

        self.angle =  int(self.k * math.degrees(closest_ar.x - self.angle_cal)) - self.l



    
if __name__ == '__main__':
    try:
        node = ArTagDriver()
        node.run()
    except rospy.ROSInterruptException:
        pass
