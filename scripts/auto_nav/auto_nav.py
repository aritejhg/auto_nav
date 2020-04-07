#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Vector3, PoseStamped
from sensor_msgs.msg import Image, LaserScan
import numpy as np
import matplotlib.pyplot as plt
import time

# # Cython nav_map_handling.pyx (Too slow, do not use)
# import pyximport
# pyximport.install()
# from nav_map_handling import MapDilateErode

from navMapProc_wrp import MapDilateErode

class AutoNav: 

    def __init__(self, nodeName, updRate, pltMap=False):
        rospy.init_node(nodeName)
        self._rosRate = rospy.Rate(updRate)
        self._plt = pltMap    
        self._subscription = []
        self._subscription.append(
            rospy.Subscriber("move_base_simple/goal", PoseStamped, self._cbGoal, self))
        self._subscription.append(
            rospy.Subscriber("map", OccupancyGrid, self._cbMap, self))

    def __del__(self):
        if self._plt: plt.ioff()
        for s in self._subscription: s.unregister()

    def spin(self):
        if self._plt:
            plt.ion()
            plt.tight_layout()
            plt.show()
        while not rospy.is_shutdown():
            try:
                self._rosRate.sleep()
            except rospy.exceptions.ROSInterruptException:
                break
    
    # CALLBACK FUNCTIONS

    @staticmethod
    def _cbGoal(msg, s):
        print(msg.pose.position.x, msg.pose.position.y)
        print(msg.pose.orientation.x, msg.pose.orientation.y)
        print(msg.header.stamp.to_time())
    
    @staticmethod
    def _cbMap(msg, s):
        print("Map received")
        # Convert map into numpy array
        mWidth = msg.info.width
        mHeight = msg.info.height
        mData = np.array([msg.data]).reshape(mWidth,mHeight,order='F')
        # Mark points with 0 for unmapped, 1 for empty, 2 for wall
        mData = np.uint8((mData > 0).choose(mData + 1,2))
        # Smooth the map by dilation and erosion
        t = time.time()
        mData = MapDilateErode(mData)
        print(time.time() - t)
        # Plot the map
        if s._plt:
            plt.imshow(mData, cmap='gray')
            plt.draw_all()
            plt.pause(0.00000000001)

    # UTILITY FUNCTIONS


if __name__ == "__main__":
    a = AutoNav("auto_nav_node", 2, pltMap=True)
    a.spin()