###########
# IMPORTS #
###########
import cv2
import time

#############
# CONSTANTS #
#############
SRC = 1
FILE = 'images/distances/h40x0y100'

################
# TAKE PICTURE #
################
camera = cv2.VideoCapture(1)
time.sleep(2)
grabbed, image = camera.read()
cv2.imwrite(FILE, image)
camera.release()