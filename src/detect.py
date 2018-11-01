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
GREEN_LOWER = np.array([28, 86, 70])
GREEN_UPPER = np.array([64, 255, 255])

# hough circles parameters
METHOD = cv2.HOUGH_GRADIENT # defines the method to detect circles in images (only one option currently available)
DP = 1 # the inverse ratio of the accumulator resolution to the image resolution (the larger the dp gets, the smaller the accumulator array gets)
MIN_DIST = 30 # minimum distance between the center (x, y) coordinates of detected circles
PARAM1 = 50 # gradient value used to handle edge detection in the Yuen et al. method
PARAM2 = 15 # accumulator threshold value for the cv2.HOUGH_GRADIENT method (the smaller the threshold is, the more circles will be detected)
MIN_RADIUS = 10 # minimum size of the radius (in pixels)
MAX_RADIUS = 100 # maximum size of the radius (in pixels)


##########
# DETECT #
##########
class Detect:

    def __init__(self):
        # Create node with name 'detect'
        rospy.init_node('detect')

        # Initiallize subscriber to listen to the topic '/images'
        self.image_sub = rospy.Subscriber('/image', Image, self.image_sub_cb)

        # Initialize publisher to publish a list of the detected balls to the topic '/balls'
        self.circles_pub = rospy.Publisher('/circles', Circles, queue_size=1)

        # Initiallize instance of CvBridge
        self.bridge = CvBridge()

        # Display info
        print('[INFO] detect on ...')



    ######################
    # CALLBACK FUNCTIONS #
    ######################
    def image_sub_cb(self, image_msg):
        """
        Given an ROS Image msg, detects any tennis balls (circles) in the image and publishes its findings to the topic '/circles'

            Args:
                - image_msg: ROS Image message
        """
        # Convert Image message into an OpenCV image
        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        # Get image id
        image_id = image_msg.header.seq

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

        # Make circles into a 1D list
        if circles is None:
            circles = []
        else:
            circles = circles[0]

        # Create a Circles message
        circles_msg = Circles()
        # Set circle_message id
        circles_msg.id = image_id
        # Convert each ball to a Ball messages and append it to the Balls message
        for px, py, radius in circles:
            circle = Circle()
            circle.px, circle.py, circle.radius = px, py, radius
            circles_msg.circles.append(circle)

        self.circles_pub.publish(circles_msg)



if __name__ == "__main__":
    # Create Detect instance
    detect = Detect()

    # spin() simply keeps python from exiting unil this node is stopped
    rospy.spin()




