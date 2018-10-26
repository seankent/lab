##################
# WORKING PARAMS #
##################
# works on: iphone videos/images
METHOD = cv2.HOUGH_GRADIENT # defines the method to detect circles in images (only one option currently available)
DP = 1 # the inverse ratio of the accumulator resolution to the image resolution (the larger the dp gets, the smaller the accumulator array gets)
MIN_DIST = 30 # minimum distance between the center (x, y) coordinates of detected circles
PARAM1 = 50 # gradient value used to handle edge detection in the Yuen et al. method
PARAM2 = 8 # accumulator threshold value for the cv2.HOUGH_GRADIENT method (the smaller the threshold is, the more circles will be detected)
MIN_RADIUS = 0 # minimum size of the radius (in pixels)
MAX_RADIUS = 100 # maximum size of the radius (in pixels)