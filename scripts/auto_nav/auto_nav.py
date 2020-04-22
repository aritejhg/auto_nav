#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Vector3, PoseStamped, Pose
from sensor_msgs.msg import Image, LaserScan
from actionlib_msgs.msg import GoalID
import numpy as np
import matplotlib.pyplot as plt
import time
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
        self._subscription.append(
            rospy.Subscriber("auto_nav/seen_target", String, self._cbSeenTarget, self))
        self._shootPos = Pose()
        self._atShootPos = False
        self._completed = False
        self._publisher = {}
        self._publisher['navgoal'] = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        self._publisher['navgoal_cancel'] = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)
        self._publisher['navgoal_cancel'].publish(GoalID())
        self._publisher['navdone'] = rospy.Publisher("auto_nav/navdone", String, queue_size=1)
        self._goalSeq = 0   # next available sequence number for goal
        self._lastGoal = (False, [])

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
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quat)
        print(msg.pose.position.x, msg.pose.position.y, yaw)
    
    @staticmethod
    def _cbMap(msg, s):
        print("Map received")
        # Retrieve robot position
        trans = s._tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
        robotX = (trans.transform.translation.x - msg.info.origin.position.x) / msg.info.resolution
        robotY = (trans.transform.translation.y - msg.info.origin.position.y) / msg.info.resolution
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
        # Publish the goal to the nav stack
        if done:
            distToShootPos = ((robotX - s._shootPos.position.x)**2 + (robotY - s._shootPos.position.y)**2)**0.5
            if distToShootPos < 3:
                s._atShootPos = True
            if not s._atShootPos:
                print("Finished mapping, going to shooting position.")
                posecmd = PoseStamped()
                posecmd.header.frame_id = "map"
                posecmd.header.stamp = rospy.Time.now()
                posecmd.header.seq = s._goalSeq
                posecmd.pose = s._shootPos
                s._publisher['navgoal'].publish(posecmd)
            elif not s._completed:
                print("Auto navigation done.")
                s._completed = True
                cancelGoal = GoalID()
                s._publisher['navgoal_cancel'].publish(cancelGoal)
                s._publisher['navdone'].publish("DONE")
        else:
            posecmd = PoseStamped()
            posecmd.header.frame_id = "map"
            posecmd.header.stamp = rospy.Time.now()
            posecmd.header.seq = s._goalSeq
            posecmd.pose.position.x = goalList[0][0] * msg.info.resolution + msg.info.origin.position.x
            posecmd.pose.position.y = goalList[0][1] * msg.info.resolution + msg.info.origin.position.y
            posecmd.pose.position.z = 0
            quat = quaternion_from_euler(0, 0, goalList[0][2])
            (posecmd.pose.orientation.x, posecmd.pose.orientation.y, posecmd.pose.orientation.z, posecmd.pose.orientation.w) = quat
            s._publisher['navgoal'].publish(posecmd)
        # Plot the map
        if s._plt:
            plt.imshow(mData, cmap='gray')
            plt.draw_all()
            plt.pause(0.00000000001)

    @staticmethod
    def _cbSeenTarget(msg, s):
        # The current pose will be marked for the bot to go back after finishing mapping
        # Each time the a message on this topic is received, the pose will be updated
        print("Position recorded.")
        trans = s._tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
        p = Pose()
        p.position = trans.translation
        p.orientation = trans.rotation

if __name__ == "__main__":
    a = AutoNav("auto_nav_node", 2, pltMap=True)
    a.spin()
