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
from sensor_msgs.msg import LaserScan, Image, CompressedImage, CameraInfo
from image_geometry import PinholeCameraModel
from nav_msgs.msg import OccupancyGrid
import message_filters as msf

# Topics
SCAN_TOPIC = 'scan'
GRID_TOPIC = 'map'
RGB_TOPIC = 'camera/rgb/image_raw'
DEPTH_TOPIC = 'camera/depth/image_raw'
CAM_TOPIC = 'camera/rgb/camera_info'

# Loop frequency (in 1/s i.e. Hertz; for msg sending main loop.)
FREQUENCY = 30

# OpenCV things?
# For color ranges: https://stackoverflow.com/questions/36817133/identifying-the-range-of-a-color-in-hsv-using-opencv/51686953
# OpenCV uses H: 0 to 179, S: 0 to 255, V: 0 to 255
blue_mask = lambda hsv_img: cv2.inRange(hsv_img, (90, 50, 70), (128, 255, 255))
orange_mask = lambda hsv_img: cv2.inRange(hsv_img, (10, 50, 70), (40, 255, 255))

# Image
IMG_WIDTH, IMG_HEIGHT = 640, 480

class Detector():
    def __init__(self, freq=FREQUENCY):
        # Transformation listener
        self._trans_lst = tf.TransformListener()

        # Topic subscribers
        self._scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self._scan_callback, queue_size=1)
        # self._rgb_sub = rospy.Subscriber(RGB_TOPIC, Image, self._rgb_callback, queue_size=1)
        self._rgb_sub = msf.Subscriber(RGB_TOPIC, Image)
        # self._depth_sub = rospy.Subscriber(DEPTH_TOPIC, Image, self._depth_callback, queue_size=1)
        self._depth_sub = msf.Subscriber(DEPTH_TOPIC, Image)
        #
        self._cam_sub = msf.Subscriber(CAM_TOPIC, CameraInfo)
        self._rgbd_sub = msf.ApproximateTimeSynchronizer([self._rgb_sub, self._depth_sub, self._cam_sub], queue_size=1, slop=0.2)
        self._rgbd_sub.registerCallback(self._rgbd_callback)
        self._grid_sub = rospy.Subscriber(GRID_TOPIC, OccupancyGrid, self._grid_callback, queue_size=1)

        # Message transmission rate
        self._freq = freq
        self._rate = rospy.Rate(freq)

        # OpenCV-ROS image converter instance
        self._bridge = CvBridge()

        # Detection variables
        self._camera_model = PinholeCameraModel()
        self._depth_tol_variance = 0.015

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

    def _rgbd_callback(self, rgb_msg, depth_msg, cam_msg):
        # Update camera info
        self._camera_model.fromCameraInfo(cam_msg)
        try:
            # Convert ROS Image to OpenCV images.
            cv_img = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
            cv_depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            # Convert to HSV format.
            cv_hsv = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)

            # Mask for blue and orange colors (based on HSV)
            blue_masked = cv2.bitwise_and(cv_img, cv_img, mask=blue_mask(cv_hsv))
            orange_masked = cv2.bitwise_and(cv_img, cv_img, mask=orange_mask(cv_hsv))

            for masked in [blue_masked, orange_masked]:
                # Get grayscale versions of masked images (i.e. V channel in HSV)
                masked_gray = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)

                # Compute connected components
                # https://stackoverflow.com/questions/35854197/how-to-use-opencvs-connected-components-with-stats-in-python
                thresh = cv2.threshold(masked_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresh, 8, cv2.CV_32S)

                # Iterate through non-background components
                for i in range(1, num_labels):
                    # Read information about component
                    x, y, w, h, area = [stats[i, stat] for stat in [cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT, cv2.CC_STAT_AREA]]
                    cX, cY = centroids[i]

                    # Get depths from each non-background point in the rectangle surrounding component.
                    depths_of_shape = [cv_depth[cur_y][cur_x] for cur_x in range(min(x, IMG_WIDTH-1), min(x+w, IMG_WIDTH-1)+1) for cur_y in range(min(y, IMG_HEIGHT-1), min(y+h, IMG_HEIGHT-1)+1) if not np.array_equal(masked_gray[cur_y][cur_x], [0, 0, 0])]
                    # Compute average non-NaN depth, and variance.
                    # TODO: 1. use variance to discard bogus avg depths.
                    #       2. use distance to warn how reasonable guess is.
                    avg_dos = np.nanmean(depths_of_shape)
                    var_dos = np.nanvar(depths_of_shape)
                    # print("cX={}, cY={}, depth={}, depth_variance={}".format(cX, cY, avg_dos, var_dos))

                    # Compute the x, y of the point in camera frame that corresponds to the centroid pixel, using the camera model.
                    cpt_x, cpt_y = self._camera_model.projectPixelTo3dRay((int(cX), int(cY)))[:2]

                    # The object is thought to be centered at: (cpt_x, cpt_y, avg_dos) in the camera frame -- convert this to a point in a global frame, e.g. map.
                    print("I see a {} centered at {}".format("blue cube" if masked is blue_masked else "orange goal", self._get_current_T('map', 'camera_rgb_frame').dot(np.array([cpt_x, cpt_y, avg_dos, 1]))[:2]))

                # Miscellaneous tests
                # output = cv2.cvtColor(blue_masked_gray, cv2.COLOR_GRAY2RGB)
                # print('Black ratio: {}'.format(sum([np.array_equal(output[cur_y][cur_x], [0, 0, 0]) for cur_x in range(min(x, IMG_WIDTH-1), min(x+w, IMG_WIDTH-1)+1) for cur_y in range(y, min(y+h, IMG_HEIGHT-1)+1)]) / (w*h)))
                # cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 3)
                # cv2.circle(output, (int(cX), int(cY)), 4, (0, 0, 255), -1)
                # cv2.imwrite('blue.png', output)

        except CvBridgeError as e:
            rospy.logerr(e)

    def _scan_callback(self, msg):
        pass

    def _rgb_callback(self, msg):
        pass

    def _depth_callback(self, msg):
        pass

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
