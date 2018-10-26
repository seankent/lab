#!/usr/bin/env python

###########
# IMPORTS #
###########
import cv2
import time


#############
# CONSTANTS #
#############
SRC = 0


##########
# CAMERA #
##########
camera = cv2.VideoCapture(1)
time.sleep(2)
grabbed, image = camera.read()
cv2.imwrite('webcam1.png', image)
camera.release()