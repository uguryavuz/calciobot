#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Author: Ugur Y. Yavuz
# Date: 11/09/2021
# Final Project: "CalcioBot" for COSC 281 (21F, Dartmouth College).

# Import of relevant libraries.
from __future__ import division, print_function
import numpy as np                 # Math
import rospy, tf, cv2              # ROS API, transformations, image processign
from cv_bridge import CvBridge, CvBridgeError  # Conversion between cv2 and ROS images

# Messages
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from nav_msgs.msg import OccupancyGrid

# Topics
SCAN_TOPIC = 'scan'
RGB_TOPIC = 'camera/rgb/image_raw'
DEPTH_TOPIC = 'camera/depth/image_raw'
GRID_TOPIC = 'map'

# Loop frequency (in 1/s i.e. Hertz; for msg sending main loop.)
FREQUENCY = 30

# OpenCV things?
# For color ranges: https://stackoverflow.com/questions/36817133/identifying-the-range-of-a-color-in-hsv-using-opencv/51686953
# OpenCV uses H: 0 to 179, S: 0 to 255, V: 0 to 255
blue_mask = lambda hsv_img: cv2.inRange(hsv_img, (90, 50, 70), (128, 255, 255))
orange_mask = lambda hsv_img: cv2.inRange(hsv_img, (10, 50, 70), (35, 255, 255))

class Detector():
    def __init__(self, freq=FREQUENCY):
        # Transformation listener
        self._trans_lst = tf.TransformListener()

        # Topic subscribers
        self._scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self._scan_callback, queue_size=1)
        self._rgb_sub = rospy.Subscriber(RGB_TOPIC, Image, self._rgb_callback, queue_size=1)
        self._depth_sub = rospy.Subscriber(DEPTH_TOPIC, Image, self._depth_callback, queue_size=1)
        self._grid_sub = rospy.Subscriber(GRID_TOPIC, OccupancyGrid, self._grid_callback, queue_size=1)

        # Message transmission rate
        self._freq = freq
        self._rate = rospy.Rate(freq)

        # OpenCV-ROS image converter instance
        self.bridge = CvBridge()
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 1
        print(params)
        self.detector = cv2.SimpleBlobDetector_create()

    def _get_current_T(self, parent, child):
        """Get the most recent transformation matrix from child frame to parent frame."""
        # Find the most recent time when the transformation was computed between the two frames.
        latest_time = self._trans_lst.getLatestCommonTime(parent, child)

        # Look up the most transform -- returns translation and rotation separately.
        trans, rot = self._trans_lst.lookupTransform(parent, child, latest_time)

        # Reconstruct into two matrices, using tf library functions.
        t, R = tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot)

        # Return the homogeneous 4x4 transformation matrix.
        return t.dot(R)

    def _scan_callback(self, msg):
        pass

    def _rgb_callback(self, msg):
        try:
            # Convert ROS Image to an OpenCV image
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Convert to HSV format.
            cv_hsv = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
            
            # Mask for blue and orange colors (based on HSV)
            blue_masked = cv2.bitwise_and(cv_img, cv_img, mask=blue_mask(cv_hsv))
            orange_masked = cv2.bitwise_and(cv_img, cv_img, mask=orange_mask(cv_hsv))

            # Get grayscale versions of masked images (i.e. V channel in HSV)
            blue_masked_gray = cv2.split(blue_masked)[-1]

            # Compute connected components
            # https://stackoverflow.com/questions/35854197/how-to-use-opencvs-connected-components-with-stats-in-python
            thresh = cv2.threshold(blue_masked_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresh, 8, cv2.CV_32S)
            print("num_labels={}".format(num_labels))
            # print("labels={}".format(labels))
            # print("stats={}".format(stats))
            # print("centroids={}".format(centroids))

            # TODO:
            # 1. Select white component and get centroid.
            # 2. Synchronize with depth image using ApproximateTimeSynchronizer.
            # 3. Find average depth of pixels in component.
            # -- If not too dissimilar, ignore?
            # 4. Assign the centroid at given depth as the location.
            
            # cv2.imwrite('blue.png', thresh)

            
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def _depth_callback(self, msg):
        pass
        # try:
        #     cv_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        #     cv2.imwrite('depth.png', cv_depth)
        # except CvBridgeError as e:
        #     rospy.logerr(e)

    
    def _grid_callback(self, msg):
        pass

    def spin(self):
        while not rospy.is_shutdown():
            pass
            self._rate.sleep()


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('detector')
    rospy.sleep(2)

    # Initialize Detector instance
    detector = Detector()

    # If interrupted, DO STH
    # rospy.on_shutdown()

    # Spin
    try:
        rospy.loginfo("Detector is spinnning.")
        detector.spin()
    except:
        rospy.logerr("ROS node interrupted.")

    

