#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Author: Ugur Y. Yavuz
# Date: 11/13/2021
# Final Project: "CalcioBot" for COSC 281 (21F, Dartmouth College).

# Import of relevant libraries.
from __future__ import division, print_function
import rospy
from detector import Detector
from pather import GridPather
from std_msgs.msg import String
from v_controller import PFollow

from PID_controller import PID

# Driver topic
DRIVER_TOPIC = 'calcio_driver'

# Loop frequency (in 1/s i.e. Hertz; for msg sending main loop.)
FREQUENCY = 30

class Driver():
    def __init__(self, freq=FREQUENCY):
        # Object detector
        self._detector_info = {'start': None, 'target': None, 'active': 0}
        self._detector = Detector(self._detector_info)

        # Pathfinder
        self._pather_info = {'path': None}
        self._pather = GridPather()

        #path follower
        self._vcont = PFollow()

        #jeff's pathfollower
        #self._pid = PID()

        # Listen for user input
        self._driver_sub = rospy.Subscriber(DRIVER_TOPIC, String, self._update_callback, queue_size=1)

        # Message transmission rate
        self._freq = freq
        self._rate = rospy.Rate(freq)

    def is_detecting(self):
        return self._detector_info['active'] != 2

    def is_pathing(self):
        return not self.is_detecting() and self._pather_info['path'] is None

    def _update_callback(self, msg):
        active_stat = self._detector_info['active']
        if active_stat == self._detector_info['active'] == 0:
            if self._detector_info['start'] is not None:
                self._detector_info['active'] = 1 if self._detector_info['active'] == 0 else self._detector_info['active']
                if self._detector_info['active'] == 1:
                    print('Start set to {}.'.format(self._detector_info['start']))
                    rospy.sleep(1)
            else:
                print('No start detected -- no updates.')
        elif active_stat == self._detector_info['active'] == 1:
            if self._detector_info['target'] is not None:
                self._detector_info['active'] = 2 if self._detector_info['active'] == 1 else self._detector_info['active']
                if self._detector_info['active'] == 2:
                    print('Target set to {}.'.format(self._detector_info['target']))
                    rospy.sleep(1)
            else:
                print('No target detected -- no updates.')

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('driver')
    rospy.sleep(2)
    driver = Driver()
    while driver._pather.grid is None: continue

    # try:
    rospy.loginfo("Driver is spinning.")
    print('Detecting! Send "OK" message to {} to register most recent estimate.'.format(DRIVER_TOPIC))
    while driver.is_detecting() or not driver._pather.grid.is_free(driver._pather.grid.coord_to_grid(driver._detector_info['target'])):
        if not driver.is_detecting():
            print('ATTENTION: Target is not an empty cell and therefore will be emptied.')
            driver._detector_info['target'] = None
            driver._detector_info['active'] = 1
    print('Pathing!')
    path = driver._pather.find_path_for_coords(driver._detector_info['start'], driver._detector_info['target'])
    driver._pather.publish_pose_markers_from_path(path)
    print("Time to follow the path")
    driver._vcont.follow(path)
    print('Great success')
    # except:
    #     rospy.logerr("ROS node interrupted.")
sudo
