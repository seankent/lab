###########
# IMPORTS #
###########
import cv2
import numpy as np
import rospy
import Queue
from sensor_msgs.msg import Image
from lab.msg import Circle, Circles
from cv_bridge import CvBridge
from threading import Thread

#############
# CONSTANTS #
#############
COLOR = [0, 0, 255] # Color of annotation [blue, green, red]
THICKNESS = 2 # Thickness of annotation in pixels
SHAPE = (480, 640) # Shape of image

###########
# DISPLAY #
###########
# Initiallize instance of CvBridge
bridge = CvBridge()

class Display:
    def __init__(self):
        # Initiallize ROS node 'display'
        rospy.init_node('display')

        # Initiallize subscriber to listen to the topic '/images'
        self.image_sub = rospy.Subscriber('/image', Image, self.image_sub_cb)
        # Current image
        self.image = None
        # ID of the current image
        self.image_id = None

        self.circles_sub = rospy.Subscriber('/circles', Circles, self.circles_sub_cb)
        # List of circles detected in the current image [(px, py, radius), (px, py, radius), ...]
        self.circles = None

        # Initiallize publisher to publish a list of the detected circles to the topic '/circles'
        self.annotated_image_pub = rospy.Publisher('/annotated_image', Image, queue_size=1)

        # Initiallize the variable used to indicate if the thread should be stopped
        self.stopped = False

        # Blocks the reception of a new image until the annotated image has been published
        self.blocked = False

        # Display info
        print('[INFO] display on ...')

    ######################
    # CALLBACK FUNCTIONS #
    ######################
    def image_sub_cb(self, image_msg):
        if not self.blocked:
            # Convert Image message into an OpenCV image/update self.image
            self.image = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            # Set image_id
            self.image_id = image_msg.header.seq
            # Block update of self.image
            self.blocked = True

    def circles_sub_cb(self, circles_msg):
        if circles_msg.id == self.image_id:
            # updated self.circles
            self.circles = []
            for circle in circles_msg.circles:
                self.circles.append((circle.px, circle.py, circle.radius))

            # Publish image
            self.display()
            # Unblock image update
            self.blocked = False

    ###########
    # DISPLAY #
    ###########
    def display(self):      
        # Create copy of image
        annotated_image = self.image.copy()

        # Loop over circles in image
        for px, py, radius in self.circles:
            # draw circles on the image
            cv2.circle(annotated_image, (px, py), radius, COLOR, THICKNESS)

        # convert annotated image into Image message
        annotated_image_msg = bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
        # publish the annotated image
        self.annotated_image_pub.publish(annotated_image_msg)
    

if __name__ == "__main__":
    # Create Display instance
    display = Display()

    # spin() simply keeps python from exiting unil this node is stopped
    rospy.spin()
