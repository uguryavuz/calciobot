#!/usr/bin/env python


#Author:Viney Regunath and Jeff Cho

#import necessary packages

#Python libraries
import sys,time


import numpy as np


#ros libraries
import rospy
import roslib

#OpenCV
import cv2

#ros messaages
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from robot_controls import ObjectPose

#image message of cube/ball
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

class BallPub:

    def __init__(self):
        #Initialize ros publisher and ros subscriber

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.callback_odom, queue_size =1)

        #camera parameters inspired by code bases - check if this is necessary for ours!
        self.camera_x = 0.095
        self.camera_y = 0s
        #we are using a rospy service to get info about object's xy coordinates
        self.obj_srv = rospy.Service('/object_pose_srv', ObjectPose, self.handle_object_pose)


        self.odom_pose = Pose2D()
