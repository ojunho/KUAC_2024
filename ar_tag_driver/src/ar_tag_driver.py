#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from ar_track_alvar_msgs.msg import AlvarMarkers

from geometry_msgs.msg import Pose

import math

import tf



class ArTagDriver:
    def __init__(self):
        rospy.init_node('ar_tag_driver_node', anonymous=True)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.arCB, queue_size= 1)


        self.rate = rospy.Rate(10)  # 10hz

        # for ar tag
        self.sorted_ar_list = []
        self.arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
        self.k = 2
        self.l = 30
        self.angle_cal = 0.017

        

    def run(self):
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self.pub.publish(hello_str)
            self.rate.sleep()

    # for ar tag

    def arCB(self, msg):
        if not msg.markers:
            rospy.loginfo("No markers detected.")
            return

        # Calculate distances and sort markers by distance
        self.sorted_ar_list = []
        for marker in msg.markers:
            dist = self.distance_from_origin(marker.pose.pose)
            self.sorted_ar_list.append((dist, marker))

        self.sorted_ar_list.sort(key=lambda x: x[0])

    def distance_from_origin(self, pose):
        """Calculate the Euclidean distance from the origin to the given pose."""
        return math.sqrt(pose.position.x**2 + pose.position.y**2 + pose.position.z**2)


    def ar_data(self):
        (ar_roll, ar_pitch, ar_yaw) = tf.euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
        angle = int(k * np.degrees(arData["DX"] - angle_cal)) - l

        return ar_roll, ar_pitch, ar_yaw, angle, arData["DX"], arData["DZ"]

    
if __name__ == '__main__':
    try:
        node = ArTagDriver()
        node.run()
    except rospy.ROSInterruptException:
        pass