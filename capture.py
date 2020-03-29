
import cv2
import numpy as np
cap = cv2.VideoCapture(0)
while True:

    ret, image = cap.read()
    hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Red color
    low_red = np.array([161, 155, 84])
    high_red = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv_frame, low_red, high_red)
    red = cv2.bitwise_and(image, image, mask=red_mask)

    # finds edges in the input image image and 
    # marks them in the output map edges 
    edges = cv2.Canny(hsv_frame,100,200) #not necessary
  
    # Display edges in a frame 
    cv2.imshow('Edges',edges) 
    cv2.imshow("Red", red)
    cv2.imshow('frame',image)

    #finds the percentage of colour pixels in an image
    tot_pixel = hsv_frame.size
    colour_pixel = np.count_nonzero(red)
    percentage = round(colour_pixel * 100 / tot_pixel, 2)
    
    if percentage != float(0):
        print('percentage is', percentage)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
