#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Vector3, PoseStamped
from sensor_msgs.msg import Image, LaserScan
import numpy as np

class AutoNav: 

    def __init__(self, nodeName, updRate):
        rospy.init_node(nodeName)
        self._rosRate = rospy.Rate(updRate)
        self._subscription = []
        self._subscription.append(
            rospy.Subscriber("move_base_simple/goal", PoseStamped, self._cbGoal, self))

    def __del__(self):
        for s in self._subscription: s.unregister()

    def spin(self):
        while not rospy.is_shutdown():
            print("test")
            self._rosRate.sleep()
    
    @staticmethod
    def _cbGoal(msg, s):
        pass

if __name__ == "__main__":
    a = AutoNav("auto_nav_node", 2)
    a.spin()
    del a