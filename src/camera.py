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
SRC = 0


##########
# CAMERA #
##########
class Camera:
	def __init__(self, src=0):
		# initialize the video camera steam and read the first frame from the stream
		# if src is an int, the it is presumed to be the index of the wevcam/USB camera on the system. If src is a str, then
		# it is assumed to to be the path to a video file
		self.camera = cv2.VideoCapture(src)
		self.grabbed, self.image = self.camera.read()

		# initiallize the variable used to indicate if the thread should be stopped
		self.stopped = False

	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update).start()

	def update(self):
		# keep looping infinitely until the thread is stopped
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				return

			# otherwise, read the next frame from the stream
			self.grabbed, self.image = self.camera.read()

	def read(self):
		# return the frame most recently read
		return self.image

	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True


# initiallize camera
camera = Camera(SRC)
# initiallize ROS node 'camera'
rospy.init_node('camera')
# initiallize publisher to publish images from camera
image_pub = rospy.Publisher('/image', Image, queue_size=1)
# initiallize instance of CvBridge
bridge = CvBridge()

# start camera stream
camera.start()
# initallize FPS counter to keep track of the frame rate
fps = fps.FPS()
fps.start()
# disply info
print('[INFO] camera on...')

while not rospy.is_shutdown():
	# grab the frame from the threaded video stream
	image = camera.read()
	# convert image to a Image message
	image_msg = bridge.cv2_to_imgmsg(image, 'bgr8')
	# publish image
	image_pub.publish(image_msg)

	# update FPS counter
	fps.update()

# stop camera stream
camera.stop()
# stop FPS counter
fps.stop()
# display fame rate
print('')
print('[INFO] elasped time: {:.2f}'.format(fps.elapsed()))
print('[INFO] approx. FPS: {:.2f}'.format(fps.fps()))