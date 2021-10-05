import numpy
import argparse
import cv2

image1Path = "../data/30.jpg"
#CHANGE ABOVE FOR DIFFERENT IMAGES
image1 = cv2.imread(image1Path)
red_lower = (0,0,100) #bgr for red, lower boundary
red_upper = (80,80,255) #bgr for red, upper boundary
blue_lower = (100,0,0) #bgr for blue, lower boundary
blue_upper = (255,80,80) #bgr for blue, upper boundary

red_mask = cv2.inRange(image1, red_lower, red_upper)
blue_mask = cv2.inRange(image1, blue_lower, blue_upper)

red_output = cv2.bitwise_and(image1, image1, mask=red_mask)
blue_output = cv2.bitwise_and(image1, image1, mask=blue_mask)

cv2.imshow("red detection", red_output)
cv2.imshow("blue detection", blue_output)
cv2.waitKey(0)