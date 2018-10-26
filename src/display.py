###########
# IMPORTS #
###########
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from lab.msg import Circle, Circles
from cv_bridge import CvBridge
from threading import Thread

#############
# CONSTANTS #
#############
COLOR = [0, 0, 255] # color of annotation [blue, green, red]
THICKNESS = 2 # thickness of annotation in pixels
SHAPE = (480, 640) # shape of image

###########
# DISPLAY #
###########
# initiallize instance of CvBridge
bridge = CvBridge()

class Display:
    def __init__(self):
        # initiallize ROS node 'display'
        rospy.init_node('display')

        # current image, initiallize to black image
        self.image = None
        # initiallize subscriber to listen to the topic '/images'
        self.image_sub = rospy.Subscriber('/image', Image, self.image_cb)

        # list of circles in the image [circle1, circle2, ...] where circle has attributes x, y, and radius
        self.circles = []
        # initiallize subscriber to listen to the topic '/circles'
        circles_sub = rospy.Subscriber('/circles', Circles, self.circles_cb)

        # initiallize publisher to publish a list of the detected circles to the topic '/circles'
        self.annotated_image_pub = rospy.Publisher('/annotated_image', Image, queue_size=1)

        # initiallize the variable used to indicate if the thread should be stopped
        self.stopped = False

    def image_cb(self, image_msg):
        # convert Image message into an OpenCV image/update self.image
        self.image = bridge.imgmsg_to_cv2(image_msg, 'bgr8')

    def circles_cb(self, circles_msg):
        # updated self.circles
        self.circles = circles_msg.circles

    def start(self):
        # start the thread to publish annotated image
        Thread(target=self.display).start()

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def display(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # check if image has been recieved
            if self.image is None:
                continue
                
            # create copy of image
            annotated_image = self.image.copy()

            # loop over circles in image
            for circle in self.circles:
                x, y, radius = circle.x, circle.y, circle.radius
                # draw circles on the image
                cv2.circle(annotated_image, (circle.x, circle.y), circle.radius, COLOR, THICKNESS)

            # convert annotated image into Image message
            annotated_image_msg = bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
            # publish the annotated image
            self.annotated_image_pub.publish(annotated_image_msg)
    




# initiallize display
display = Display()
# start display thread
display.start()
# disply info
print('[INFO] display on...')

# loop until node is shut
while not rospy.is_shutdown():
    # spin() simply keeps python from exiting unil this node is stopped
    rospy.spin()

display.stop()