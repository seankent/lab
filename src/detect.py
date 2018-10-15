###########
# IMPORTS #
###########
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from lab.msg import Circle, Circles
from cv_bridge import CvBridge

#############
# CONSTANTS #
#############
# HSV green bounds
GREEN_LOWER = np.array([29, 86, 6])
GREEN_UPPER = np.array([64, 255, 255])

# hough circles parameters
METHOD = cv2.HOUGH_GRADIENT # defines the method to detect circles in images (only one option currently available)
DP = 1 # the inverse ratio of the accumulator resolution to the image resolution (the larger the dp gets, the smaller the accumulator array gets)
MIN_DIST = 30 # minimum distance between the center (x, y) coordinates of detected circles
PARAM1 = 50 # gradient value used to handle edge detection in the Yuen et al. method
PARAM2 = 8 # accumulator threshold value for the cv2.HOUGH_GRADIENT method (the smaller the threshold is, the more circles will be detected)
MIN_RADIUS = 0 # minimum size of the radius (in pixels)
MAX_RADIUS = 100 # maximum size of the radius (in pixels)


##########
# DETECT #
##########
# initiallize instance of CvBridge
bridge = CvBridge()

def detect(image_msg):
    """
    Given an Image message, finds the location and radius (in pixels) of the circles (tennis balls) in the image and 
    publishes the result as a Circles message (list of Circle messages)
    """
    # convert Image message into an OpenCV image
    image = bridge.imgmsg_to_cv2(image_msg, 'bgr8')

    # Resize the image, blur it, and convert it to the HSV color space
    # frame = imutils.resize(frame, width=450)
    blur = cv2.GaussianBlur(image, (11, 11), 0) 
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # Construct a mask for the color "green", then perform a series of dilations and erosions to remove any small blobs left in the mask
    mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Detect circles in the image
    circles = cv2.HoughCircles(mask, METHOD, DP, MIN_DIST, param1=PARAM1, param2=PARAM2, minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)

    # initiallize Circles message
    circles_msg = Circles()
    # convert circles to Circle messages and append them to the Circles message
    if circles is not None:
        # loop over each circle in circles
        for x, y, radius in circles[0]:
            circle = Circle()
            circle.x, circle.y, circle.radius = x, y, radius
            circles_msg.circles.append(circle)

    circles_pub.publish(circles_msg)

# initiallize ROS node 'detect'
rospy.init_node('detect')
# initiallize subscriber to listen to the topic '/images'
image_sub = rospy.Subscriber('/image', Image, detect)
# initiallize publisher to publish a list of the detected circles to the topic '/circles'
circles_pub = rospy.Publisher('/circles', Circles, queue_size=1)
# disply info
print('[INFO] detect on...')

# spin() simply keeps python from exiting unil this node is stopped
rospy.spin()



