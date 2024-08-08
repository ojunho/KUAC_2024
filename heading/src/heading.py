#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf
import math

class ImuHeadingCalculator:
    def __init__(self):
        self.heading = None
        self.heading_pub = rospy.Publisher("/heading", Float64, queue_size=1)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imuCB)


    def imuCB(self, msg):
        # 쿼터니언을 사용하여 roll, pitch, yaw 계산
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler

        self.heading = yaw * 180.0 / math.pi

        # Heading을 퍼블리시
        heading_msg = Float64()
        heading_msg.data = self.heading
        self.heading_pub.publish(heading_msg)

if __name__ == "__main__":
    rospy.init_node("imu_heading_calculator")
    imuHeadingCalculator = ImuHeadingCalculator()
    rospy.spin()
