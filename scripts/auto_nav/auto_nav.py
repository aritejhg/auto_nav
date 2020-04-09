#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Vector3, PoseStamped
from sensor_msgs.msg import Image, LaserScan
import numpy as np
import matplotlib.pyplot as plt
import time
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# # Cython nav_map_handling.pyx (Too slow, do not use)
# import pyximport
# pyximport.install()
# from nav_map_handling import MapDilateErode

from navMapProc_wrp import MapDilateErode, FindNavGoal

class AutoNav: 

    def __init__(self, nodeName, updRate, pltMap=False):
        rospy.init_node(nodeName)
        self._rosRate = rospy.Rate(updRate)
        self._plt = pltMap
        self._tfBuffer = tf2_ros.Buffer()
        self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
        rospy.sleep(1.0)
        self._subscription = []
        self._subscription.append(
            rospy.Subscriber("move_base_simple/goal", PoseStamped, self._cbGoal, self))
        self._subscription.append(
            rospy.Subscriber("map", OccupancyGrid, self._cbMap, self))
        self._publisher = {}
        self._publisher['navgoal'] = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        self._goalSeq = 0   # next available sequence number for goal

    def __del__(self):
        if self._plt: plt.ioff()
        for s in self._subscription: s.unregister()

    def spin(self):
        if self._plt:
            plt.ion()
            plt.show()
        while not rospy.is_shutdown():
            try:
                self._rosRate.sleep()
            except rospy.exceptions.ROSInterruptException:
                break
    
    # CALLBACK FUNCTIONS

    @staticmethod
    def _cbGoal(msg, s):
        s._goalSeq = msg.header.seq + 1
        print(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        print(roll, pitch, yaw)
        # print(msg.header.stamp.to_time())
        # print(msg.header.seq, msg.header.frame_id)
    
    @staticmethod
    def _cbMap(msg, s):
        print("Map received")
        # Retrieve robot position
        trans = s._tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
        robotX = (trans.transform.translation.x - msg.info.origin.position.x) / msg.info.resolution
        robotY = (trans.transform.translation.y - msg.info.origin.position.y) / msg.info.resolution
        # print(trans.transform.translation, msg.info.origin.position, msg.info.resolution)
        # print(robotX, robotY)
        # Convert map into numpy array
        mWidth = msg.info.width
        mHeight = msg.info.height
        mData = np.array([msg.data]).reshape(mWidth,mHeight,order='F')
        # Mark points with 0 for unmapped, 1 for empty, 2 for wall
        mData = np.uint8((mData > 0).choose(mData + 1,2))
        # Smooth the map by dilation and erosion
        mData = MapDilateErode(mData)
        # Find the navigation goal
        (done, goalList) = FindNavGoal(mData, robotX, robotY)
        print(done, goalList)
        # Publish the goal to the nav stack 
        if not done:
            posecmd = PoseStamped()
            posecmd.header.frame_id = "map"
            posecmd.header.stamp = rospy.Time.now()
            posecmd.pose.position.x = goalList[0][0] * msg.info.resolution + msg.info.origin.position.x
            posecmd.pose.position.y = goalList[0][1] * msg.info.resolution + msg.info.origin.position.y
            posecmd.pose.position.z = 0
            quat = quaternion_from_euler(0, 0, goalList[0][2])
            # print(posecmd.pose.position)
            # print(euler_from_quaternion(quat))
            (posecmd.pose.orientation.x, posecmd.pose.orientation.y, posecmd.pose.orientation.z, posecmd.pose.orientation.w) = quat
            s._publisher['navgoal'].publish(posecmd)
        # Plot the map
        if s._plt:
            plt.imshow(mData, cmap='gray')
            plt.draw_all()
            plt.pause(0.00000000001)

    # UTILITY FUNCTIONS


if __name__ == "__main__":
    a = AutoNav("auto_nav_node", 2, pltMap=True)
    a.spin()