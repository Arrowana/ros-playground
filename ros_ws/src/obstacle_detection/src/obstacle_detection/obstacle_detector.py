import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObstacleDetector:

    def __init__(self):
        self._width_clearance = 0.5
        self._minimum_distance = 1.0

        self._angle_width_clearance = math.atan(self._width_clearance / 2 / self._minimum_distance)
        print(self._angle_width_clearance)

    def get_obstacle(self, scan):
        obstacle_msg = Bool()
        obstacle_msg.data = self._is_obstacle(scan)
        return obstacle_msg

    def _is_obstacle(self, scan):
        angle = scan.angle_min

        for laser_range in scan.ranges:
            print(angle, laser_range)
            if -self._angle_width_clearance < angle < self._angle_width_clearance:
                print(laser_range)
                if laser_range < self._minimum_distance:
                    return True
            angle += scan.angle_increment
        return False

    def get_obstacle_msg_class(self):
        return Bool
