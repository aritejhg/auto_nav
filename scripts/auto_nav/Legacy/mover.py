#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def mover():
    # initialize node
    rospy.init_node('mover', anonymous=True)

    # create Twist object
    twist = Twist()

    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # set the update rate
    rate = rospy.Rate(10) # 10 Hz

    # while not CTRL-C
    while not rospy.is_shutdown():
    	# get keyboard input
    	cmd_char = str(raw_input("Keys w/x a/d s: "))
    	
    	# check which key was entered
    	if cmd_char == 's':
            # stop moving
	    twist.linear.x = 0.0
	    twist.angular.z = 0.0
    	elif cmd_char == 'w':
            # move forward
	    twist.linear.x = 0.1
	    twist.angular.z = 0.0
    	elif cmd_char == 'x':
            # move backward
	    twist.linear.x = -0.1
	    twist.angular.z = 0.0
    	elif cmd_char == 'a':
            # turn counter-clockwise
	    twist.linear.x = 0.0
	    twist.angular.z = 1.0
    	elif cmd_char == 'd':
            # turn clockwise
	    twist.linear.x = 0.0
	    twist.angular.z = -1.0

	# publish twist to move the TurtleBot
	pub.publish(twist)

	# log the info
        rospy.loginfo(cmd_char)
        
        # wait until it is time to run again
        rate.sleep()


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
