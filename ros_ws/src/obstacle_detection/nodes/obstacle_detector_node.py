#!/usr/bin/env python
import rospy
from obstacle_detection.obstacle_detector import ObstacleDetector
from sensor_msgs.msg import LaserScan


class ObstacleDetectorNode:

    def __init__(self):
        rospy.init_node('obstacle_detector')
        self._obstacle_detector = ObstacleDetector()

        self._obstacle_publisher = rospy.Publisher(
            'obstacle',
            self._obstacle_detector.get_obstacle_msg_class(),
            queue_size=-1
        )
        rospy.Subscriber('scan', LaserScan, self._on_scan)

    def _on_scan(self, scan):
        obstacle = self._obstacle_detector.get_obstacle(scan)
        self._obstacle_publisher.publish(obstacle)

    def main(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

if __name__== "__main__":
    ObstacleDetectorNode().main()
