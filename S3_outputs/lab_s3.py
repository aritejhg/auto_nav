# laser_range and lr2i
import re
import numpy as np
f = open('lidar.txt')
line = f.readlines()[13]
f.close()
laser_range = np.array(re.findall('\d+.\d*',line))

# odom
quat = np.loadtxt('odom.txt', skiprows=14, delimiter=':', usecols=1, max_rows=4) 

# Map data
%cd ~/catkin_ws/src/auto_nav/scripts/auto_nav/S3_outputs
f = open('map.txt')
line = f.readlines()[23]
f.close()
l = []
for t in line.split(','):
    try:
        l.append(float(t))
    except ValueError:
        pass
for _ in range(384**2 - len(l)) :
    l.append(0.)
occdata = np.array(l)

