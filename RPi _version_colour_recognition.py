import time
import cv2
import numpy as np
import math
from picamera.array import PiRGBArray
from picamera import PiCamera

# setting the raspberry pi resolution and frame rate
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30

# PIRGBarray-reads the frames form RasPi camera as NumPy array(takes in camera object and size of resolution)
rawCapture = PiRGBArray(camera, size=(640, 480))
width = rawCapture.get(cv2.CAP_PROP_FRAME_WIDTH)
height = rawCapture.get(cv2.CAP_PROP_FRAME_HEIGHT)
#centre of image
centre = (int(width/2), int(height/2))
# allow the camera to warmup
time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array

	hsv_frame = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

	#lower mask(0-10)
	lower_red = np.array([0,100,100])
	upper_red = np.array([10,255,255])
	lower_red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
	#upper mask(170-180)
	lower_red = np.array([160,100,100])
	upper_red = np.array([170,255,255])
	upper_red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)

	red_mask = lower_red_mask + upper_red_mask
	red_image= cv2.bitwise_and(image, image, mask=red_mask)


	#counts total number of red pixel
	tot_pixel = hsv_frame.size
	colour_pixel = np.count_nonzero(red_image)
	percentage = round(colour_pixel * 100 / tot_pixel, 2)
    
	if percentage != float(0):
		print('percentage is', percentage)
	'''if detected percentage is !=0, and percentage <4 move bot towards the target
	if percentage is at least 4 %, shoot'''

	gray = cv2.cvtColor(red_image, cv2.COLOR_BGR2GRAY)
	ret, threshImg = cv2.threshold(gray, 27, 255, cv2.THRESH_BINARY)
	drawnImage = red_image
    # find the contours in the thresholded image...
	contours, high = cv2.findContours(threshImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	
	for c in contours:
		if cv2.contourArea(c) < 4000:
			continue
		(x,y,w,h)= cv2.boundingRect(c)
		cv2.rectangle(drawnImage,(x,y),(x+w,y+h),(255,0,0),2)

		Centroid = (int((x + x + w) /2), int((y + y + h) /2))
		cv2.circle(drawnImage, Centroid, 1, (54, 255, 164), 4)
		cv2.circle(drawnImage, centre, 1, (115, 0, 255), 4)
		
		dist_from_cent = [centre[0]-Centroid[0], centre[1]-Centroid[1]]
		print ('centre_dist =', dist_from_cent)

		'''if dist_from_centre[0] <0:
			turn bot towards right
		elif dist_from_centre[0]>20:
			turn bot left 
		elif dist_from_centre[1] <0:
			move bot up 
		elif dist_from_centre[1] >0:
			move bot down
		else:
			shoot '''

	cv2.imshow(winName, drawnImage)

	# get the key from the keyboard
	key = cv2.waitKey(1) & 0xFF
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
rawCapture.release()
cv2.destroyAllWindows()
