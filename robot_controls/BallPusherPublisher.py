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


from ObjectPose.srv import ObjPose,, ObjPoseResponse
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

        #pose info of robot and object
        self.odom_pose = Pose2D()
        self.obj_pose =Pose2D()

        self.objpose_cal = False
        self.objpose_status = False
        self
        self.odom = False
        self

    def handle_obj_pose(self,req):
        self.obj_dist = req.dist

        #means time to change status
        self.objpose_status = True

        #if odom is collected
        if self.odom:
            #semaphore status
            self.objpose_cal = True
            theta_to_right = self.odom_pose.theta
            self.obj_pose.x = self.odom_pose.x + math.cos(theta_to_right)*(self.camera_x + self.ball_dist)
            self.obj_pose.y =  self.odom_pose.y + math.sin(theta_to_right) * (self.camera_x + self.ball_dist)
            self.obj_pose.theta = 0.0
            self.objpose_cal = False
            rospy.loginfo("****Object Pose **********")
            rospy.loginfo(self.obj_pose.x)
            rospy.loginfo(self.obj_pose.y)
            rospy.loginfo("****Odom Poses**********")
            rospy.loginfo(self.odom_pose.x)
            rospy.loginfo(self.odom_pose.y)

            rospy.loginfo("****MetaData**********")
            rospy.loginfo(math.cos(theta_to_right)*(self.camera_x + self.ball_dist))
            rospy.loginfo(math.sin(theta_to_right)*(self.camera_x + self.ball_dist))
            rospy.logininfo(self.odom_pose.theta)


        print("This is outside the srv block code")


        resp = ObjPoseResponse()
        resp.x = self.obj_pose.x
        resp.y = self.obj_pose.y



        #callback function for odom
        def callback_odom(self, odom_data):
            rospy.loginfo("callback_odom")
            #semaphore status
            if self.objpose_cal == False:

                self.odom = True
                #update coordinates of robot and object
                self.odom_pose.x = odom_data.pose.pose.position.x
                self.odom_pose.y = odom_data.pose.pose.position.y
                quaternion = odom_data.pose.pose.orientation
                explicit_quat = [quaternion.x,quaternion.y, quaternion.z, quaternion.w]
                euler = tf.transformations.euler_from_quarternion(explicit_quat)
                self.odom_pose.theta = euler[2]


def main(args):
    ''' Initializes and cleanup ros mode'''
    rospy.init_node('obj_pose', log_level = rospy.DEBUG)
    ic = ball_pose()
    objpose_pub = rospy.Publisher('/obj_pose_pub', Pose2D, queue_size=10)
    try:
        rate = rospy.Rate(10) #10 hz
        while not rospy.is_shutdown():
            if(ic.objpose_status):
                objpose_pub.publish(ic.obj_pose)

            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down ROS feature module")

if __main__ == '__main__':
    main(sys.argv)
