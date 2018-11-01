#!/usr/bin/env python

###########
# IMPORTS #
###########
import cv2
import rospy
import fps
from threading import Thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


#############
# CONSTANTS #
#############
# If src is an int, the it is presumed to be the index of the wevcam/USB camera on the system. If src is a str, then
# it is assumed to to be the path to a video file
SRC = 0

# Highest number the image id can reach
MAX_ID = 2**32


##########
# CAMERA #
##########
class Camera:
    def __init__(self):
        # Initiallize ROS node 'camera'
        rospy.init_node('camera')
        
        # Initialize the video camera steam and read the first frame from the stream
        self.camera = cv2.VideoCapture(SRC)

        # Initiallize the variable used to indicate if the thread should be stopped
        self.stopped = False

        # initiallize publisher to publish images from camera
        self.image_pub = rospy.Publisher('/image', Image, queue_size=1)

        # Initiallize instance of CvBridge
        self.bridge = CvBridge()

        # Initallize FPS counter to keep track of the frame rate
        self.fps = fps.FPS()

        self.id = 0

    #############
    # STREAMING #
    #############
    def start(self):
        """
        Start thread to detect balls.
        """
        # Start thread to continually publish the video stream over the topic '/image'
        Thread(target=self.stream).start()
        
        # Start timer to track frame rate
        self.fps.start()
        # Disply info
        print('[INFO] camera on...')


    def stop(self):
        """
        Stop streaming thread.
        """
        # Stop thread
        self.stopped = True

        # Stop FPS counter
        self.fps.stop()
        # Disply frame rate info
        print('[INFO] elasped time: {:.2f}'.format(self.fps.elapsed()))
        print('[INFO] approx. FPS: {:.2f}'.format(self.fps.fps()))


    def stream(self):
        """
        Continually grabs the next image from the video stream and publish it as a ROS Image to the topic '/image'
        """
        # Keep looping infinitely until the thread is stopped
        while True:
            # If the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # Grab the next frame from the stream
            grabbed, image = self.camera.read()

            # Convert image to ROS message 
            image_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            # Set image_msg id
            image_msg.header.seq = self.id

            # Publish image
            self.image_pub.publish(image_msg)
            
            # Increment id number (wrap it if it exceeds MAX_ID)
            self.id += 1; self.id %= MAX_ID

            # Update FPS counter
            self.fps.update()


            


if __name__ == '__main__':
    # Initiallize camera
    camera = Camera()
    # Start streaming images
    camera.start()

    # spin() simply keeps python from exiting unil this node is stopped
    rospy.spin()

    # Stop streaming images
    camera.stop()
