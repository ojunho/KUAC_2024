class KMUFoscar:
    def __init__(self):
        # mission variables
        # self.curr_mission = "LaneDetection" # 현재 수행 중인 미션
        self.static_obstacle_distance = None
        self.static_obstacle_lcr = None # -1:Left , 0:Center , 1:Right

        # humanoid variables
        self.imu_x = None
        self.imu_y = None
        self.imu_z = None
        
        # object detection boolean variables
        self.is_obstacle = False