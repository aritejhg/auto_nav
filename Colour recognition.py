import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np


# setting the raspberry pi resolution and frame rate
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30

# PIRGBarray-reads the frames form RasPi camera as NumPy array(takes in camera object and size of resolution)
rawCapture = PiRGBArray(camera, size=(640, 480))

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array

        # COLOUR RECOGNITION- converting images form BGR to HSV colour space
        hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Red color
        low_red = np.array([161, 155, 84])
        high_red = np.array([179, 255, 255])
        red_mask = cv2.inRange(hsv_frame, low_red, high_red)
        red = cv2.bitwise_and(image, image, mask=red_mask)
        
        # Blue color
        low_blue = np.array([94, 80, 2])
        high_blue = np.array([126, 255, 255])
        blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
        blue = cv2.bitwise_and(image, image, mask=blue_mask)

        # Green color
        low_green = np.array([25, 52, 72])
        high_green = np.array([102, 255, 255])
        green_mask = cv2.inRange(hsv_frame, low_green, high_green)
        green = cv2.bitwise_and(image, image, mask=green_mask)

        # Every color except white
        low = np.array([0, 42, 0])
        high = np.array([179, 255, 255])
        mask = cv2.inRange(hsv_frame, low, high)
        result = cv2.bitwise_and(image, image, mask=mask)

        
        #finds the percentage of colour pixels in an image
        tot_pixel = hsv_frame.size
        colour_pixel = np.count_nonzero(red)
        percentage = round(colour_pixel * 100 / tot_pixel, 2)
        
        print('percentage is', percentage)

        cv2.imshow("Frame", image)
        cv2.imshow("mask", mask)
        cv2.imshow("Frame", frame)
        cv2.imshow("Red", red)
        cv2.imshow("Blue", blue)
        cv2.imshow("Green", green)
        cv2.imshow("Result", result)

        key = cv2.waitKey(1)
        rawCapture.truncate(0)
        if key == 27:
            break

cv2.destroyAllWindows()




