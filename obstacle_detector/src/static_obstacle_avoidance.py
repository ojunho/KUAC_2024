import rospy
from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32
import math

class StaticAvoidance():
    def __init__(self):
        rospy.Subscriber("/raw_obstacles", Obstacles, self.obstacleCB)
        self.obstacles = []
        self.is_static = False
        self.steer = 0

        self.static_pub = rospy.Publisher("static_steer", Int32, queue_size=5)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_static:
                if len(self.obstacles) == 0:
                    # if 초음파 100 이상:
                    #     self.is_static = False
                    #     self.steer = 0
                    pass
            else:
                if len(self.obstacles) < 3:
                    tangent = self.obstacles[0][1] / self.obstacles[0][0]
                    degree = math.atan(tangent) * 180 / math.pi

                    if degree > 0: # 왼쪽 장애물
                        if degree < 30:
                            self.steer = 30
                        elif degree < 60:
                            self.steer = -20
                        else:
                            self.steer = 0
                    else:
                        if degree > -30:
                            self.steer = -30
                        elif degree > -60:
                            self.steer = 20
                        else:
                            self.steer = 0

            self.steer_pub.publish(self.steer)
            rate.sleep()
                        

    def obstacleCB(self, msg):
        obstacles = []
        for circle in msg.circles:
            x = round(circle.center.x, 3)
            y = round(circle.center.y, 3)
            obstacles.append([x, y])

        if len(obstacles) > 0:
            self.obstacles = sorted(obstacles, key=lambda c: math.sqrt(c[0]**2 + c[1]**2))
        else:
            self.obstacles = []


if __name__ == '__main__':
    rospy.init_node('static_obstacle_avoidance', anonymous=True)
    try:
        static_obstacle_avoidance = StaticAvoidance()
    except rospy.ROSInterruptException:
        pass
