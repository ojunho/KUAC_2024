#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker

class Object:
    def __init__(self, centerX, centerY):
        self.centerX = centerX
        self.centerY = centerY

    def __str__(self):
        return f"(X={self.centerX:.2f}, Y={self.centerY:.2f})"
    

class ConeArray:
    def __init__(self):
        self.cones = []
        self.size = 0

    def __str__(self):
        cones_str = ", ".join([f"({cone.centerX}, {cone.centerY})" for cone in self.cones])
        return f"ConeArray(size={self.size}, cones=[{cones_str}])"
    

pivot_reset_area = [0.4, 0.0, 0.7, -0.7]

class WaypointMaker:
    def __init__(self):
        self.objects = []
        self.leftCones = ConeArray()
        self.rightCones = ConeArray()
        self.leftPivot = self.reset_pivot("LEFT")
        self.rightPivot = self.reset_pivot("RIGHT")
        
        rospy.Subscriber("raw_obstacles_rubbercone", Obstacles, self.update_objects)
        self.maker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=500)

    def reset_pivot(self, left="LEFT"):
        if left == "LEFT":
            return Object(0, 0.5)
        else:
            return Object(0, -0.5)
        
    def update_objects(self, data):
        objects = [Object(circle.center.x, circle.center.y) for circle in data.circles]
        self.objects = sorted(objects, key=lambda x:self.get_distance(Object(0, 0), x))
        
    def get_distance(self, obj1, obj2):
        return ((obj1.centerX - obj2.centerX) ** 2 + (obj1.centerY - obj2.centerY) ** 2) ** 0.5
    
    def set_left_right_cone_info(self):
        leftCones = []
        rightCones = []

        left_point = self.leftPivot
        right_point = self.rightPivot

        for cone in self.objects:
            leftPivotToConeDistance = self.get_distance(left_point, cone)
            rightPivotToConeDistance = self.get_distance(right_point, cone)

            if leftPivotToConeDistance > 0.75 and rightPivotToConeDistance > 0.75:
                continue

            if leftPivotToConeDistance <= rightPivotToConeDistance:
                # 콘이 왼쪽 라인에 해당함
                leftCones.append(cone)
                left_point = cone
            else:
                # 콘이 오른쪽 라인에 해당함
                rightCones.append(cone)
                right_point = cone

        if len(leftCones) == 0:
            leftCones = [self.leftPivot]
        else:
            self.leftPivot = leftCones[0]
        if len(rightCones) == 0:
            rightCones = [self.rightPivot]
        else:
            self.rightPivot = rightCones[0]

        self.leftCones.cones = leftCones
        self.leftCones.size = len(leftCones)
        self.leftPivot = self.leftCones.cones[0]
        self.rightCones.cones = rightCones
        self.rightCones.size = len(rightCones)
        self.rightPivot = self.rightCones.cones[0]

        if self.leftPivot.centerX >= pivot_reset_area[0] or self.leftPivot.centerY <= pivot_reset_area[1] or self.leftPivot.centerY >= pivot_reset_area[2]:
            self.leftPivot = self.reset_pivot("LEFT")
        if self.rightPivot.centerX >= pivot_reset_area[0] or self.rightPivot.centerY <= pivot_reset_area[3] or self.rightPivot.centerY >= pivot_reset_area[1]:
            self.rightPivot = self.reset_pivot("RIGHT")

        if self.leftCones.size < 5:
            last_value = self.leftCones.cones[-1]
            self.leftCones.cones.extend([last_value] * (5 - self.leftCones.size))
        else:
            self.leftCones.cones = self.leftCones.cones[:5]
        if self.rightCones.size < 5:
            last_value = self.rightCones.cones[-1]
            self.rightCones.cones.extend([last_value] * (5 - self.rightCones.size))
        else:
            self.rightCones.cones = self.rightCones.cones[:5]
            
        for i, cone in enumerate(self.leftCones.cones):
            self.publish_point_marker(cone, i, 0.1, 1.0, 0.68, 0.8)
        for i, cone in enumerate(self.rightCones.cones):
            self.publish_point_marker(cone, i+10, 0.1, 0.63, 0.82, 1.0)

    def set_waypoint_info(self):
        midpoints = []
        for i in range(5):
            l_obj = self.leftCones.cones[i]
            r_obj = self.rightCones.cones[i]
            midpoint = Object((l_obj.centerX + r_obj.centerX) / 2,
                              (l_obj.centerY + r_obj.centerY) / 2)
            midpoints.append(midpoint)
            self.publish_point_marker(midpoint, i+100, 0.05, 0.8, 0.7, 0.85)

        # waypoints = self.interpolate_objects(midpoints)
        # for i, point in enumerate(waypoints):
        #     self.publish_point_marker(point, i+200, 0.01, 1.0, 1.0, 1.0)

    def interpolate_objects(self, objects, points_per_segment=50):
        interpolated_objects = []

        for i in range(len(objects) - 1):
            start_obj = objects[i]
            end_obj = objects[i + 1]

            for j in range(points_per_segment):
                t = j / (points_per_segment - 1)
                interpolated_centerX = (1 - t) * start_obj.centerX + t * end_obj.centerX
                interpolated_centerY = (1 - t) * start_obj.centerY + t * end_obj.centerY
                interpolated_objects.append(Object(interpolated_centerX, interpolated_centerY))

        return interpolated_objects
    
    def publish_point_marker(self, point, id, scale=0.1, r=1.0, g=1.0, b=1.0):
        marker = Marker()
        marker.header.frame_id = "laser_frame"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pivot_markers"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.centerX
        marker.pose.position.y = point.centerY
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale  # 구의 크기
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        self.maker_pub.publish(marker)
            

def main():
    rospy.init_node("waypoint_maker")
    waypoint_maker = WaypointMaker()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        waypoint_maker.set_left_right_cone_info()
        waypoint_maker.set_waypoint_info()

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.RosInterruptException:
        pass
