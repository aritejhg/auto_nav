#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
from PIL import Image

# constants
occ_bins = [-1, 0, 100, 101]

def callback(msg):
    # create numpy array
    occdata = np.array([msg.data])
    # occdata is -1 (unexplored), 0 (occupied), or value between 0 and 100 (probability occupied)
    # compute histogram to identify percent of bins with -1
    occ_counts = np.histogram(occdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    # log the info
    rospy.loginfo('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i', occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)

    # make occdata go from 0 to 2 so we can use uint8, which won't be possible if we have
    # negative values
    # first make negative values 0
    occ2 = occdata + 1
    # now change all the values above 1 to 2
    occ3 = (occ2>1).choose(occ2,2)
    # convert into 2D array using column order
    odata = np.uint8(occ3.reshape(msg.info.width,msg.info.height,order='F'))
    # create image from 2D array using PIL
    img = Image.fromarray(odata)
    # show the image using grayscale map
    plt.imshow(img,cmap='gray')
    plt.draw_all()
    # pause to make sure the plot gets created
    plt.pause(0.00000000001)


def occupancy():
    # initialize node
    rospy.init_node('occupancy', anonymous=True)

    # subscribe to map occupancy data
    rospy.Subscriber('map', OccupancyGrid, callback)

    # create matplotlib figure
    plt.ion()
    plt.show()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        occupancy()
    except  rospy.ROSInterruptException:
        pass
