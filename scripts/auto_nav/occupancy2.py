#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import tf2_ros
from PIL import Image
from tf.transformations import euler_from_quaternion

# constants
occ_bins = [-1, 0, 100, 101]

# create global variables
rotated = Image.fromarray(np.array(np.zeros((1,1))))


def callback(msg, tfBuffer):
    global rotated

    # create numpy array
    occdata = np.array([msg.data])
    # compute histogram to identify percent of bins with -1
    occ_counts = np.histogram(occdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
    rospy.loginfo('Width: %i Height: %i',msg.info.width,msg.info.height)
    rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)

    # find transform to convert map coordinates to base_link coordinates
    # lookup_transform(target_frame, source_frame, time)
    trans = tfBuffer.lookup_transform('base_link', 'map', rospy.Time(0))
    rospy.loginfo(['Trans: ' + str(trans.transform.translation)])
    rospy.loginfo(['Rot: ' + str(trans.transform.rotation)])
    # convert quaternion to Euler angles
    orientation_list = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    rospy.loginfo(['Yaw: R: ' + str(yaw) + ' D: ' + str(np.degrees(yaw))])

    # make occdata go from 0 instead of -1, reshape into 2D
    oc2 = occdata + 1
    # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
    oc3 = (oc2>1).choose(oc2,2)
    # reshape to 2D array using column order
    odata = np.uint8(oc3.reshape(msg.info.width,msg.info.height,order='F'))
    # create image from 2D array using PIL
    img = Image.fromarray(odata)
    ######  Translate the map centre to the robot
    trans2 = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
    robotX = (trans2.transform.translation.x - msg.info.origin.position.x) / msg.info.resolution
    robotY = (trans2.transform.translation.y - msg.info.origin.position.y) / msg.info.resolution
    diffX = robotX - 0.5 * img.size[0]
    diffY = robotY - 0.5 * img.size[1]
    img = img.transform(img.size, Image.AFFINE, (1, 0, diffY, 0, 1, diffX), fill=0)
    ######
    # rotate by 180 degrees to invert map so that the forward direction is at the top of the image
    rotated = img.rotate(np.degrees(yaw)+180)
    # show image using grayscale map
    plt.imshow(rotated,cmap='gray')
    ######
    # plt.plot(img.size[0]/2, img.size[1]/2, 'y+')  # Optionally mark the centre
    ######
    plt.draw_all()
    # pause to make sure the plot gets created
    plt.pause(0.00000000001)


def occupancy():
    # initialize node
    rospy.init_node('occupancy', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1.0)

    # subscribe to map occupancy data
    rospy.Subscriber('map', OccupancyGrid, callback, tfBuffer)

    plt.ion()
    plt.show()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        occupancy()
    except  rospy.ROSInterruptException:
        pass
