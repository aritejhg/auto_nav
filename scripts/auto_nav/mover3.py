#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math
import cmath
import numpy as np

# constants
rotate_speed = 0.1
linear_speed = 0.1

# global variables
yaw = 0.0

# function to check if keyboard input is a number
def isnumber(value):
    try:
        int(value)
        return True
    except ValueError:
        return False


# function to set the global variable yaw using the odometry information
def get_rotation(msg):
	# use global variable yaw
    global yaw
    orientation_quat =  msg.pose.pose.orientation
    orientation_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


# function to rotate the TurtleBot
def rotatebot(rot_angle):
	# use global variable yaw
    global yaw
    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # set the update rate to 1 Hz
    rate = rospy.Rate(1)

    # get current yaw angle
    current_yaw = yaw
    # log the info
    rospy.loginfo(['Current: ' + str(math.degrees(current_yaw))])
    # we are going to use complex numbers to avoid problems when the angles go from
    # 360 to 0, or from -180 to 180
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    # calculate desired yaw
    target_yaw = current_yaw + math.radians(rot_angle)
    # convert to complex notation
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
    rospy.loginfo(['Desired: ' + str(math.degrees(cmath.phase(c_target_yaw)))])
    # divide the two complex numbers to get the change in direction
    c_change = c_target_yaw / c_yaw
    # get the sign of the imaginary component to figure out which way we have to turn
    c_change_dir = np.sign(c_change.imag)
    # set linear speed to zero so the TurtleBot rotates on the spot
    twist.linear.x = 0.0
    # set the direction to rotate
    twist.angular.z = c_change_dir * rotate_speed
    # start rotation
    pub.publish(twist)

    # we will use the c_dir_diff variable to see if we can stop rotating
    c_dir_diff = c_change_dir
    rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
    # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
    # becomes -1.0, and vice versa
    while(c_change_dir * c_dir_diff > 0):
        # get the current yaw in complex form
        c_yaw = complex(math.cos(yaw),math.sin(yaw))
        rospy.loginfo(['While Yaw: ' + str(math.degrees(yaw))])
        # get difference in angle between current and target
        c_change = c_target_yaw / c_yaw
        # get the sign to see if we can stop
        c_dir_diff = np.sign(c_change.imag)
        rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
        rate.sleep()

    rospy.loginfo(['End Yaw: ' + str(math.degrees(yaw))])
    # set the rotation speed to 0
    twist.angular.z = 0.0
    # stop the rotation
    pub.publish(twist)


# function to read keyboard input and move the TurtleBot accordingly
def mover():
    twist = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('mover', anonymous=True)
    rospy.Subscriber('odom', Odometry, get_rotation)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        cmd_char = str(raw_input("Keys w/x -/+int s: "))

        if isnumber(cmd_char):
            # rotate by specified angle
            rotatebot(int(cmd_char))
        else:
            if cmd_char == 's':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif cmd_char == 'w':
                twist.linear.x = linear_speed
                twist.angular.z = 0.0
            elif cmd_char == 'x':
                twist.linear.x = -linear_speed
                twist.angular.z = 0.0

            pub.publish(twist)

        rospy.loginfo(cmd_char)
        rate.sleep()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
