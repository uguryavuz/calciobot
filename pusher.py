#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Author: Ugur Y. Yavuz
# Date: 11/14/2021
# Final Project: "CalcioBot" for COSC 281 (21F, Dartmouth College).

# Import of relevant libraries.
from __future__ import division, print_function
from enum import Enum
import numpy as np
import rospy, tf

# Messsages
from geometry_msgs.msg import Twist

# Topics
CMD_VEL_TOPIC = 'cmd_vel'

# Frame names
MAP_FRAME = 'map'
BASE_LINK_FRAME = 'base_link'

# Loop frequency (in 1/s i.e. Hertz; for msg sending main loop.)
FREQUENCY = 30

# Miscellaneous
WARN_INIT_DIST = 1

# Small function to map angles between -pi and pi.
short_angle = lambda angle: (angle - 2 * np.pi) if angle > np.pi else ((angle + 2 * np.pi) if angle < -np.pi else angle)

# FSM
class fsm(Enum):
    IDLE = 1
    REORIENTING = 2
    PATH_FOLLOWING = 3

# PD controller
class PD:
    # Initialize class
    def __init__(self, kp=1, kd=1, dt=1/FREQUENCY):
        """The constructor for the PD controller."""
        # Hold parameters as class variables.
        self._p = kp
        self._d = kd
        self._dt = dt

        # Store most recent error.
        self._cur_err = None

        # Store most recently computed control.
        # This should be available for outside reads, and is therefore not named as a private variable.
        self.u = 0

    # Take a step given the error, and update controls.
    def step(self, err):
        # Compute proportional and derivative gain (if there is a previously stored error)
        p_gain = self._p * err   
        d_gain = self._d * (((err - self._cur_err) / self._dt) if (self._cur_err != None) else 0)  

        # Compute new control and store error.
        self.u = p_gain + d_gain
        self._cur_err = err

    # Reset
    def reset(self):
        self._cur_err = None
        self.u = 0

class Pusher():
    def __init__(self, lin_vel=0.06, ang_vel=0, freq=FREQUENCY):
        # Path to follow
        self.path = None

        # Velocities
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        
        # Active rotation flag and message count
        self._active_rot = [False, 0, 0]  # Activeness, message count, direction (+/-1).

        # Transformation listener
        self._trans_lst = tf.TransformListener()
        
        # Topic subscribers/publishers
        self._cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

        # Message transmission rate
        self._freq = freq
        self._rate = rospy.Rate(freq)

        # Controller
        self._pd = PD(kp=1, kd=4)

        # FSM
        self._fsm = fsm.IDLE

    def _get_current_T(self, parent, child):
        """Get the most recent transformation matrix from child frame to parent frame."""
        # Find the most recent time when the transformation was computed between the two frames.
        self._trans_lst.waitForTransform(parent, child, rospy.Time(), rospy.Duration(4.0))
        latest_time = self._trans_lst.getLatestCommonTime(parent, child)

        # Look up the most transform -- returns translation and rotation separately.
        trans, rot = self._trans_lst.lookupTransform(parent, child, latest_time)

        # Reconstruct into two matrices, using tf library functions.
        t, R = tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot)

        # Return the homogeneous 4x4 transformation matrix.
        return t.dot(R)

    def move_msg(self, lin_vel=None, ang_vel=None):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Set default arguments
        if lin_vel is None: lin_vel = self.lin_vel
        if ang_vel is None: ang_vel = self.ang_vel

        # Publish Twist message.
        twist_msg = Twist()
        twist_msg.linear.x = lin_vel
        twist_msg.angular.z = ang_vel
        self._cmd_pub.publish(twist_msg)

    def rotate_to(self, angle, ang_vel=np.deg2rad(30)):
        """Rotate to given angle in map frame"""
        rel_angle = short_angle(angle - tf.transformations.rotation_from_matrix(self._get_current_T(MAP_FRAME, BASE_LINK_FRAME))[0])
        self._active_rot[1] = int((abs(rel_angle) / ang_vel) * self._freq)
        self._active_rot[2] = np.sign(rel_angle)
        self._fsm = fsm.REORIENTING

    def stop_msg(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def expand_path(self, path, add_count=5):
        # Expand path's start by add_count, by adding points along approximate initial slope.
        init_delta = np.mean(np.gradient([pt[0] for pt in path])[:add_count]), np.mean(np.gradient([pt[1] for pt in path])[:add_count])
        expanded_path = path[:]
        for _ in range(add_count):
            expanded_path.insert(0, (expanded_path[0][0] - init_delta[0], expanded_path[0][1] - init_delta[1]))
        return expanded_path

    def _get_current_xy(self):
        return tuple(self._get_current_T(MAP_FRAME, BASE_LINK_FRAME).dot(np.array([0, 0, 0, 1]))[:2])
    
    def _get_closest_pt_in_path(self):
        cur_pt = self._get_current_xy()
        return sorted(self.path, key=lambda pt: np.linalg.norm(np.array(pt) - np.array(cur_pt)))[0]

    def follow_path(self, path):
        if np.linalg.norm(np.array(self._get_current_xy()) - np.array(path[0])) > WARN_INIT_DIST:
            print("WARNING: You are more than {} m away from the initial point of this path.".format(WARN_INIT_DIST))
        self.path = path
        self._pd.reset()
        cur_pt = self._get_current_xy()
        # print(np.rad2deg(np.arctan2(path[1][1] - cur_pt[1], path[1][0] - cur_pt[0])))
        self.rotate_to(np.arctan2(path[1][1] - cur_pt[1], path[1][0] - cur_pt[0]))

    def _get_cur_err(self):
        closest_pt = self._get_closest_pt_in_path()
        closest_idx = self.path.index(closest_pt)
        return np.cross(np.array(self._get_current_xy()) - np.array(closest_pt), np.gradient(np.array(self.path))[0][closest_idx])[()]
    
    def spin(self):
        while self._fsm != fsm.IDLE:
            if self._fsm == fsm.PATH_FOLLOWING:
                if self._get_closest_pt_in_path() in self.path[-2:]:
                    self.stop_msg()
                    self.path = None
                    self._pd.reset()
                    self._fsm = fsm.IDLE
                    continue
                
                self.move_msg(ang_vel=self._pd.u * 15)
                # print(self._pd.u)
                self._pd.step(self._get_cur_err())
            elif self._fsm == fsm.REORIENTING:
                # If not active -- set the flag to active and reset msg index.
                if not self._active_rot[0]:
                    i, self._active_rot[0] = 0, True
                # If active, iterate through message count to send necessary number of messages.
                elif i < self._active_rot[1]:
                    self.move_msg(lin_vel=0, ang_vel=np.deg2rad(30)*self._active_rot[2])
                    i += 1
                # If exhausted, motion is complete -- set the flag to False and change state to idle.
                else:
                    self.stop_msg()
                    self._active_rot[0] = False
                    self._fsm = fsm.PATH_FOLLOWING
            # Sleep to maintain constant time interval between loop iterations.
            self._rate.sleep()


    