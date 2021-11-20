#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Author: Ugur Y. Yavuz
# Date: 11/13/2021
# Final Project: "CalcioBot" for COSC 281 (21F, Dartmouth College).

# Import of relevant libraries.
from __future__ import division, print_function
import rospy                        # ROS API
from detector import Detector       # Detector module
from pather import Pather           # Pathfinding module
from pusher import Pusher           # Pushing/path-following module
from std_msgs.msg import String     # To process user messages to the driver topic.s

# Driver topic
DRIVER_TOPIC = 'calcio_driver'

# Loop frequency (in 1/s i.e. Hertz; for msg sending main loop.)
FREQUENCY = 30

class Driver():
    def __init__(self, freq=FREQUENCY):
        # Object detector
        # Start and target will be the locations estimated for the cube and the goal respectively.
        # Active = 0 means looking for the cube, 1 means for the goal, and 2 means idle.
        self._detector_info = {'start': None, 'target': None, 'active': 0}
        self._detector = Detector(self._detector_info)

        # Pathfinder
        self._pather = Pather()

        # Pusher
        self._pusher = Pusher()

        # Listen for user input
        self._driver_sub = rospy.Subscriber(DRIVER_TOPIC, String, self._update_callback, queue_size=1)

        # Message transmission rate
        self._freq = freq
        self._rate = rospy.Rate(freq)

    # Return whether the detector is actively detecting
    def is_detecting(self):
        return self._detector_info['active'] != 2

    # Callback for user messages, specifically processed during detection.
    # Any message is processed as an approval by the user of suggested estimates for locations.
    def _update_callback(self, msg):
        # Keep track of initial status, so as to abort when it is switched mid-operation.
        active_stat = self._detector_info['active']
        if active_stat == self._detector_info['active'] == 0:
            if self._detector_info['start'] is not None:
                self._detector_info['active'] = 1 if self._detector_info['active'] == 0 else self._detector_info['active']
                if self._detector_info['active'] == 1:
                    print('Start (i.e. blue cube location) set to {}.'.format(self._detector_info['start']))
                    rospy.sleep(1)
            else:
                print('No start detected -- no updates.')
        elif active_stat == self._detector_info['active'] == 1:
            if self._detector_info['target'] is not None:
                self._detector_info['active'] = 2 if self._detector_info['active'] == 1 else self._detector_info['active']
                if self._detector_info['active'] == 2:
                    print('Target (i.e. yellow goal location) set to {}.'.format(self._detector_info['target']))
                    rospy.sleep(1)
            else:
                print('No target detected -- no updates.')

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('driver')
    rospy.sleep(2)
    driver = Driver()

    
    try:
        while driver._pather.grid is None: continue

        rospy.loginfo("Driver is spinning.")
        print('Detecting! Send "OK" message to {} to register most recent estimate.'.format(DRIVER_TOPIC))
        while driver.is_detecting() or not driver._pather.grid.is_free(driver._pather.grid.coord_to_grid(driver._detector_info['target'])):
            if not driver.is_detecting():
                print('ATTENTION: Target is not an empty cell and therefore will be emptied.')
                driver._detector_info['target'] = None
                driver._detector_info['active'] = 1
        print('Pathing!')

        path_main = driver._pather.find_path_for_coords(driver._detector_info['start'], driver._detector_info['target'], start_ignore_window=5)
        print('Navigating back to the cube to start pushing.')

        rospy.sleep(2)

        path_main_exp = driver._pusher.expand_path(path_main)
        path_back = driver._pather.find_path_for_coords(driver._pusher._get_current_xy(), path_main_exp[0], type=1)

        driver._pusher.follow_path(path_back)
        driver._pusher.spin()
        driver._pusher.follow_path(path_main)
        driver._pusher.spin()
        print('Great success')

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

   

