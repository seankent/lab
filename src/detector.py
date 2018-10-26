#!/usr/bin/env python

###########
# IMPORTS #
###########
import numpy as np
import rospy
import cv2
from threading import Thread
from sensor_msgs.msg import Image
from lab.msg import Ball, Balls
from cv_bridge import CvBridge

#############
# CONSTANTS #
#############
# Camera source. If src is an int, the it is presumed to be the index of the wevcam/USB camera on the system 
# If src is a str, then it is assumed to to be the path to a video file
SRC = 1

# HSV green bounds
GREEN_LOWER = np.array([.11, .6, .2]) * 255
GREEN_UPPER = np.array([.25, 1, 1]) * 255

# hough circles parameters
METHOD = cv2.HOUGH_GRADIENT # defines the method to detect circles in images (only one option currently available)
DP = 1 # the inverse ratio of the accumulator resolution to the image resolution (the larger the dp gets, the smaller the accumulator array gets)
MIN_DIST = 30 # minimum distance between the center (x, y) coordinates of detected circles
PARAM1 = 50 # gradient value used to handle edge detection in the Yuen et al. method
PARAM2 = 10 # accumulator threshold value for the cv2.HOUGH_GRADIENT method (the smaller the threshold is, the more circles will be detected)
MIN_RADIUS = 5 # minimum size of the radius (in pixels)
MAX_RADIUS = 100 # maximum size of the radius (in pixels)

# Display annotated image
DISPLAY = True
COLOR = [0, 0, 255] # color of annotation [blue, green, red]
THICKNESS = 2 # thickness of annotation in pixels
SHAPE = (480, 640) # shape of image

##############
# CONTROLLER #
##############
class Detector:

    def __init__(self):
        # Create node with name 'detector'
        rospy.init_node('detector')

        # Initialize the video camera
        self.camera = cv2.VideoCapture(SRC)

        # Initiallize instance of CvBridge
        self.bridge = CvBridge()

        # Initialize publisher to publish a list of the detected balls to the topic '/balls'
        self.balls_pub = rospy.Publisher('/balls', Balls, queue_size=1)

        # A publisher which will publish an image annotated with information about the detected balls to the topic 'line/annotated_image_pub'
        self.annotated_image_pub = rospy.Publisher('/annotated_image_pub', Image, queue_size=1)

        # Boolean used to indicate if the streaming thread should be stopped
        self.stopped = False


    #############
    # STREAMING #
    #############
    def start(self): 
        """
        Start thread to detect balls.
        """
        Thread(target=self.stream).start()

    def stop(self):
        """
        Stop streaming thread.
        """
        self.stopped = True

    def stream(self):
        """
        Continually grabs the next image from the video stream and attempts to detect balls in the image. If balls are detected,
        it will publish a Balls message (containing information about the location of the balls) to to the topic '/balls'
        """
        # Keep looping infinitely until the thread is stopped
        while True:
            # If the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # Grab the next frame from the stream
            self.grabbed, self.image = self.camera.read()

            # Look for balls in self.image and publish the findings as a Balls message to the topic '/balls'
            self.detect()

    ##########
    # DETECT #
    ##########
    def detect(self):
        """
        Finds the location and radius (in pixels) of the tennis balls in self.image and publishes the result to
        the topic '/balls'
        """
        # Resize the image, blur it, and convert it to the HSV color space
        # frame = imutils.resize(frame, width=450)
        blur = cv2.GaussianBlur(self.image, (11, 11), 0) 
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # Construct a mask for the color "green", then perform a series of dilations and erosions to remove any small blobs left in the mask
        mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detect circles in the image
        circles = cv2.HoughCircles(mask, METHOD, DP, MIN_DIST, param1=PARAM1, param2=PARAM2, minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)

        # Make sure balls is a 1D list
        if circles is None:
            balls = []
        else:
            balls = circles[0]

        # Initiallize Balls message
        balls_msg = Balls()
        # Convert each ball to a Ball messages and append it to the Balls message
        for x, y, radius in balls:
            ball = Ball()
            ball.x, ball.y, ball.radius = x, y, radius
            balls_msg.balls.append(ball)

        # Publish annotated image with information 
        if DISPLAY:
            # Create copy of image
            annotated_image = self.image.copy()

            #  Loop over circles in image
            for x, y, radius in balls:
                # Draw circles on the image
                cv2.circle(annotated_image, (x, y), radius, COLOR, THICKNESS)

            # Convert annotated image into Image message
            annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
            # Publish the annotated image
            self.annotated_image_pub.publish(annotated_image_msg)

        self.balls_pub.publish(balls_msg)



if __name__ == "__main__":

    # Create Detector instance
    detector = Detector()
    # Start stream to detect balls
    detector.start()

    # spin() simply keeps python from exiting unil this node is stopped
    rospy.spin()

    # Stop the stream to detect balls
    detector.stop()

    
