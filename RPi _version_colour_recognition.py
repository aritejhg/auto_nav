#! /usr/bin/env python

import time
import cv2
import numpy as np
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO
import math

#pin setup
GPIO.setmode (GPIO.BCM):
led = 27
GPIO.setup(led,GPIO.OUT)
gate =16
GPIO.setup(gate,GPIO.OUT)
tmotor = 12
GPIO.setup (tmotor, GPIO.OUT)
tmotorpwm= GPIO.PWM(tmotor,50)
tmotorpwm.start(0)
lmotor = 18
GPIO.setup(lmotor,GPIO.OUT)
lmotorpwm=GPIO.pwm(lmotor,50)
lmotorpwm.start(5)
Trig_sr04=23
GPIO.setup(Trig_sr04, GPIO.OUT)
GPIO.output(Trig_sr04, False)
Echo_sr04=24
GPIO.setup(Echo_sr04, GPIO.IN)

#for rotation of turtlebot
pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate= rospy.Rate(1)
rot = Twist()
vel_msg=()

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

#rotation object
rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rot = Twist()

#to publish the percentage of the red colour detected to auto_nav
target_pub = rospy.Publisher("auto_nav/seen_target", String, queue_size=1)
percentage_1 = float(0)

def callback(msg):
	print(msg)


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
	
	#for auto_nav to save the coordinates of the position where highest percentage of red is detected
    	if percentage > percentage_1 and percentage >= 4 :
		percentage_1 = percentage
		target_pub.publish(String("colour detected"))
	#print('percentage is', percentage)


	gray = cv2.cvtColor(red_image, cv2.COLOR_BGR2GRAY)
	ret, threshImg = cv2.threshold(gray, 27, 255, cv2.THRESH_BINARY)
	drawnImage = red_image
	
	# to get the "Done" message from auto_nav to continue target detection and shooting
	nav_done = rospy.Subscriber("auto_nav/navdone", String, callback)
	
	if nav_done == "Done":
  		# find the contours in the thresholded image...
		contours, high = cv2.findContours(threshImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		for c in contours:
			if cv2.contourArea(c) < 4000:
				continue
			(x,y,w,h)= cv2.boundingRect(c)
			cv2.rectangle(drawnImage,(x,y),(x+w,y+h),(255,0,0),2)
			
			#make led flash once to indicate detection
			GPIO.output(led,GPIO.HIGH)
			time.sleep(1)
			GPIO.output(led,GPIO.LOW)
			time.sleep(1)
			
			Centroid = (int((x + x + w) /2), int((y + y + h) /2))
			cv2.circle(drawnImage, Centroid, 1, (54, 255, 164), 4)
			cv2.circle(drawnImage, centre, 1, (115, 0, 255), 4)
			
			#Find distance using SR04
			GPIO.output(Trig_sr04, True)
			time.sleep(0.00001)
			GPIO.output(Trig_sr04, False)
			while GPIO.input(Echo_sr04)==0:
				pulse_start = time.time()
			while GPIO.input(Echo_sr04)==1:
				pulse_end = time.time()
			distance = (pulse_end - pulse_start) x 17150
			while distance >250:
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 0
 				vel_msg.linear.x=0.1
				pub.publish(vel_msg)
				time.sleep(1.3)
				vel_msg.linear.x=0.0
				pub.publish(vel_msg)
				distance-=13
				
			dist_from_cent = [centre[0]-Centroid[0], centre[1]-Centroid[1]
					  
			sleeptime=(asin(dist_from_cent[0]/(2*distance)))*0.087)
                	if dist_from_centre[0] <-8:
				rot.angular.z=-0.2
			  	pub.publish(rot)
			  	time.sleep(sleeptime)
			  	rot.angular.z=0
			  	pub.publish(rot)
			elif dist_from_centre[0]>8:
			  	rot.angular.z=0.2
			  	pub.publish(rot)
			  	time.sleep(sleeptime)
			  	rot.angular.z=0
			  	pub.publish(rot)
					  
			sleeptime2=asin(dist_from_cent[1]/(2*distance))*0.0115
			if dist_from_centre[1] <-3:
			 	tmotorpwm.ChangeDutyCycle(8.0)
			  	time.sleep(sleeptime2)
			  	tmotorpmw.ChangeDutyCycle(7.5)
			  	time.sleep(1)
			elif dist_from_centre[1] >0:
			  	tmotorpwm.ChangeDutyCycle(8.0)
			  	time.sleep(sleeptime2)
			  	tmotorpmw.ChangeDutyCycle(7.5)
			  	time.sleep(1)
					
				
			#aiming done led blinks twice
			for f in range(2):
				GPIO.output(led,GPIO.HIGH)
				time.sleep(1)
				GPIO.output(led,GPIO.LOW)
				time.sleep(1)
			

			#shooting
			#limiter motor control
			lmotorpwm.ChangeDutyCycle(10)
			time.sleep(1)
			GPIO.output(gate, GPIO.HIGH)#MOSFET gate control for shooting
			time.sleep(10)
			lmotorpwm.ChangeDutyCycle(5)
			time.sleep(1)
			GPIO.output(gate, GPIO.LOW)
			time.sleep(1)
			
			#shooting done, led blinks thrice]
			for k in range (3):
				GPIO.output(led,GPIO.HIGH)
				time.sleep(1)
				GPIO.output(led,GPIO.LOW)
				time.sleep(1)   

	cv2.imshow(winName, drawnImage)

	# get the key from the keyboard
	key = cv2.waitKey(1) & 0xFF
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
rawCapture.release()
lmotorpwm.stop()
tmotorpwm.stop()
GPIO.cleanup()
cv2.destroyAllWindows()
