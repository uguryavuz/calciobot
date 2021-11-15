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

# FSM
class fsm(Enum):
    IDLE = 1
    PATH_FOLLOWING = 2

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
    def __init__(self, lin_vel=0.2, ang_vel=0, freq=FREQUENCY):
        # Path to follow
        self.path = None

        # Velocities
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel

        # Transformation listener
        self._trans_lst = tf.TransformListener()
        
        # Topic subscribers/publishers
        self._cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

        # Message transmission rate
        self._freq = freq
        self._rate = rospy.Rate(freq)

        # Controller
        self._pd = PD(kp=1, kd=0.9)

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

    def _publish_move_msg(self, lin_vel=None, ang_vel=None):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Set default arguments
        if lin_vel is None: lin_vel = self.lin_vel
        if ang_vel is None: ang_vel = self.ang_vel

        # Publish Twist message.
        twist_msg = Twist()
        twist_msg.linear.x = lin_vel
        twist_msg.angular.z = ang_vel
        self._cmd_pub.publish(twist_msg)

    def _publish_stop_msg(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def expand_path(self, path, add_count=5):
        # Expand path's start by add_count, by adding points along approximate initial slope.
        init_delta = np.mean(np.gradient([pt[0] for pt in path])[:add_count]), np.mean(np.gradient([pt[1] for pt in path])[:add_count])
        self.expanded_path = path[:]
        for _ in range(add_count):
            self.expanded_path.insert(0, (self.expanded_path[0][0] - init_delta[0], self.expanded_path[0][1] - init_delta[1]))

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
        self._fsm = fsm.PATH_FOLLOWING

    def _get_cur_err(self):
        closest_pt = self._get_closest_pt_in_path()
        closest_idx = self.path.index(closest_pt)
        # print(np.array(self._get_current_xy()) - np.array(closest_pt))
        # print(np.gradient(np.array(self.path))[0][closest_idx])
        return np.cross(np.array(self._get_current_xy()) - np.array(closest_pt), np.gradient(np.array(self.path))[0][closest_idx])[()]
    
    def spin(self):
        while self._fsm == fsm.PATH_FOLLOWING:
            if self._get_closest_pt_in_path() == self.path[-1]:
                self.path = None
                self._pd.reset()
                self._fsm = fsm.IDLE
                continue
            self._publish_move_msg(ang_vel=self._pd.u * 40)
            print(self._pd.u)
            self._pd.step(self._get_cur_err())
            # Sleep to maintain constant time interval between loop iterations.
            self._rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pfollower')
    rospy.sleep(2)
    pusher = Pusher()
    rospy.on_shutdown(pusher._publish_stop_msg)
    try:
        # pusher.follow_path([(0, 0), (0.10, 0.5), (0.20, 0.5), (0.30, 0.5), (0.40, 1)])
        # pusher.follow_path([(-1.9999999552965164, 2.0000001043081284), (-1.8999999538064003, 2.0000001043081284), (-1.7999999523162842, 2.0000001043081284), (-1.6999999508261681, 2.0000001043081284), (-1.5999999493360519, 2.0000001043081284), (-1.4999999478459358, 2.0000001043081284), (-1.4999999478459358, 1.9000001028180122), (-1.4999999478459358, 1.8000001013278961), (-1.4999999478459358, 1.70000009983778), (-1.4999999478459358, 1.6000000983476639), (-1.4999999478459358, 1.5000000968575478), (-1.4999999478459358, 1.4000000953674316), (-1.4999999478459358, 1.3000000938773155), (-1.4999999478459358, 1.2000000923871994), (-1.4999999478459358, 1.1000000908970833), (-1.4999999478459358, 1.0000000894069672), (-1.4999999478459358, 0.90000008791685104), (-1.4999999478459358, 0.80000008642673492), (-1.4999999478459358, 0.7000000849366188), (-1.4999999478459358, 0.60000008344650269), (-1.4999999478459358, 0.50000008195638657), (-1.4999999478459358, 0.40000008046627045), (-1.4999999478459358, 0.30000007897615433), (-1.4999999478459358, 0.20000007748603821), (-1.4999999478459358, 0.10000007599592209), (-1.4999999478459358, 7.4505805969238281e-08), (-1.4999999478459358, -0.09999992698431015), (-1.4999999478459358, -0.19999992847442627), (-1.4999999478459358, -0.29999992996454239), (-1.4999999478459358, -0.39999993145465851), (-1.4999999478459358, -0.49999993294477463), (-1.4999999478459358, -0.59999993443489075), (-1.4999999478459358, -0.69999993592500687), (-1.4999999478459358, -0.79999993741512299), (-1.4999999478459358, -0.89999993890523911), (-1.4999999478459358, -0.99999994039535522), (-1.4999999478459358, -1.0999999418854713), (-1.4999999478459358, -1.1999999433755875), (-1.4999999478459358, -1.2999999448657036), (-1.3999999463558197, -1.2999999448657036), (-1.2999999448657036, -1.2999999448657036), (-1.1999999433755875, -1.2999999448657036), (-1.0999999418854713, -1.2999999448657036), (-0.99999994039535522, -1.2999999448657036), (-0.89999993890523911, -1.2999999448657036), (-0.79999993741512299, -1.2999999448657036), (-0.69999993592500687, -1.2999999448657036), (-0.59999993443489075, -1.2999999448657036), (-0.49999993294477463, -1.2999999448657036), (-0.39999993145465851, -1.2999999448657036), (-0.29999992996454239, -1.2999999448657036), (-0.29999992996454239, -1.1999999433755875), (-0.29999992996454239, -1.0999999418854713), (-0.29999992996454239, -0.99999994039535522), (-0.29999992996454239, -0.89999993890523911), (-0.29999992996454239, -0.79999993741512299), (-0.29999992996454239, -0.69999993592500687), (-0.29999992996454239, -0.59999993443489075), (-0.29999992996454239, -0.49999993294477463), (-0.29999992996454239, -0.39999993145465851), (-0.29999992996454239, -0.29999992996454239), (-0.29999992996454239, -0.19999992847442627), (-0.29999992996454239, -0.09999992698431015), (-0.29999992996454239, 7.4505805969238281e-08), (-0.29999992996454239, 0.10000007599592209), (-0.29999992996454239, 0.20000007748603821), (-0.29999992996454239, 0.30000007897615433), (-0.29999992996454239, 0.40000008046627045), (-0.29999992996454239, 0.50000008195638657), (-0.29999992996454239, 0.60000008344650269), (-0.29999992996454239, 0.7000000849366188), (-0.19999992847442627, 0.7000000849366188), (-0.19999992847442627, 0.80000008642673492), (-0.09999992698431015, 0.80000008642673492), (-0.09999992698431015, 0.90000008791685104), (7.4505805969238281e-08, 0.90000008791685104), (7.4505805969238281e-08, 1.0000000894069672), (7.4505805969238281e-08, 1.1000000908970833), (0.10000007599592209, 1.1000000908970833), (0.20000007748603821, 1.1000000908970833), (0.20000007748603821, 1.2000000923871994), (0.30000007897615433, 1.2000000923871994), (0.30000007897615433, 1.3000000938773155), (0.40000008046627045, 1.3000000938773155), (0.40000008046627045, 1.4000000953674316), (0.40000008046627045, 1.5000000968575478), (0.50000008195638657, 1.5000000968575478), (0.60000008344650269, 1.5000000968575478), (0.60000008344650269, 1.6000000983476639), (0.7000000849366188, 1.6000000983476639), (0.7000000849366188, 1.70000009983778), (0.80000008642673492, 1.70000009983778), (0.80000008642673492, 1.8000001013278961), (0.80000008642673492, 1.9000001028180122), (0.90000008791685104, 1.9000001028180122), (1.0000000894069672, 1.9000001028180122), (1.0000000894069672, 2.0000001043081284), (1.1000000908970833, 2.0000001043081284), (1.1000000908970833, 2.1000001057982445), (1.2000000923871994, 2.1000001057982445), (1.2000000923871994, 2.2000001072883606), (1.2000000923871994, 2.3000001087784767), (1.3000000938773155, 2.3000001087784767), (1.4000000953674316, 2.3000001087784767), (1.4000000953674316, 2.4000001102685928), (1.5000000968575478, 2.4000001102685928), (1.5000000968575478, 2.500000111758709), (1.6000000983476639, 2.500000111758709), (1.6000000983476639, 2.6000001132488251), (1.6000000983476639, 2.7000001147389412), (1.70000009983778, 2.7000001147389412), (1.8000001013278961, 2.7000001147389412), (1.8000001013278961, 2.8000001162290573), (1.9000001028180122, 2.8000001162290573), (1.9000001028180122, 2.9000001177191734), (2.0000001043081284, 2.9000001177191734), (2.0000001043081284, 3.0000001192092896)][::-1])
        # print([(i * 0.1, i * 0.1) for i in range(11)])
        pusher.follow_path([(2.0000001043081284, 1.0000000894069672), (1.9000001028180122, 1.0000000894069672), (1.8000001013278961, 1.0000000894069672), (1.70000009983778, 1.0000000894069672), (1.6000000983476639, 1.0000000894069672), (1.5000000968575478, 1.0000000894069672), (1.4000000953674316, 1.0000000894069672), (1.3000000938773155, 1.0000000894069672), (1.2000000923871994, 1.0000000894069672), (1.1000000908970833, 1.0000000894069672), (1.0000000894069672, 1.0000000894069672), (0.90000008791685104, 1.0000000894069672), (0.80000008642673492, 1.0000000894069672), (0.7000000849366188, 1.0000000894069672), (0.60000008344650269, 1.0000000894069672), (0.50000008195638657, 1.0000000894069672), (0.40000008046627045, 1.0000000894069672), (0.30000007897615433, 1.0000000894069672), (0.20000007748603821, 1.0000000894069672), (0.10000007599592209, 1.0000000894069672), (7.4505805969238281e-08, 1.0000000894069672), (-0.09999992698431015, 1.0000000894069672), (-0.19999992847442627, 1.0000000894069672), (-0.29999992996454239, 1.0000000894069672), (-0.39999993145465851, 1.0000000894069672), (-0.39999993145465851, 1.1000000908970833), (-0.49999993294477463, 1.1000000908970833), (-0.49999993294477463, 1.2000000923871994), (-0.49999993294477463, 1.3000000938773155), (-0.59999993443489075, 1.3000000938773155), (-0.69999993592500687, 1.3000000938773155), (-0.69999993592500687, 1.4000000953674316), (-0.79999993741512299, 1.4000000953674316), (-0.79999993741512299, 1.5000000968575478), (-0.89999993890523911, 1.5000000968575478), (-0.89999993890523911, 1.6000000983476639), (-0.89999993890523911, 1.70000009983778), (-0.99999994039535522, 1.70000009983778), (-1.0999999418854713, 1.70000009983778), (-1.1999999433755875, 1.70000009983778), (-1.2999999448657036, 1.70000009983778), (-1.3999999463558197, 1.70000009983778), (-1.4999999478459358, 1.70000009983778), (-1.5999999493360519, 1.70000009983778), (-1.6999999508261681, 1.70000009983778), (-1.7999999523162842, 1.70000009983778), (-1.8999999538064003, 1.70000009983778), (-1.9999999552965164, 1.70000009983778), (-2.0999999567866325, 1.70000009983778), (-2.1999999582767487, 1.70000009983778), (-2.1999999582767487, 1.8000001013278961), (-2.1999999582767487, 1.9000001028180122), (-2.1999999582767487, 2.0000001043081284), (-2.1999999582767487, 2.1000001057982445), (-2.1999999582767487, 2.2000001072883606), (-2.1999999582767487, 2.3000001087784767), (-2.1999999582767487, 2.4000001102685928), (-2.1999999582767487, 2.500000111758709), (-2.1999999582767487, 2.6000001132488251), (-2.1999999582767487, 2.7000001147389412), (-2.1999999582767487, 2.8000001162290573), (-2.1999999582767487, 2.9000001177191734), (-2.2999999597668648, 2.9000001177191734), (-2.2999999597668648, 3.0000001192092896)])
        pusher.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

    