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
from PID_controller import Pid

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
        #self._vcont = PFollow()

        #jeff's pathfollower
        self._pid = Pid()

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
<<<<<<< HEAD
    # print('Detecting! Send "OK" message to {} to register most recent estimate.'.format(DRIVER_TOPIC))
    # while driver.is_detecting() or not driver._pather.grid.is_free(driver._pather.grid.coord_to_grid(driver._detector_info['target'])):
    #     if not driver.is_detecting():
    #         print('ATTENTION: Target is not an empty cell and therefore will be emptied.')
    #         driver._detector_info['target'] = None
    #         driver._detector_info['active'] = 1
    # print('Pathing!')
    # path = driver._pather.find_path_for_coords(driver._detector_info['start'], driver._detector_info['target'], start_ignore_window=5)
    # driver._pather.publish_pose_markers_from_path(path)
    # print('Great success')

    # driver._pather.publish_pose_markers_from_path([(70, 60), (69, 60), (68, 60), (67, 60), (66, 60), (65, 60), (64, 60), (63, 60), (62, 60), (61, 60), (60, 60), (59, 60), (58, 60), (57, 60), (56, 60), (55, 60), (54, 60), (53, 60), (52, 60), (51, 60), (50, 60), (49, 60), (48, 60), (47, 60), (46, 60), (46, 61), (45, 61), (45, 62), (45, 63), (44, 63), (43, 63), (43, 64), (42, 64), (42, 65), (41, 65), (41, 66), (41, 67), (40, 67), (39, 67), (38, 67), (37, 67), (36, 67), (35, 67), (34, 67), (33, 67), (32, 67), (31, 67), (30, 67), (29, 67), (28, 67), (28, 68), (28, 69), (28, 70), (28, 71), (28, 72), (28, 73), (28, 74), (28, 75), (28, 76), (28, 77), (28, 78), (28, 79), (27, 79), (27, 80)])
    print([driver._pather.grid.grid_to_coord(pt) for pt in [(70, 60), (69, 60), (68, 60), (67, 60), (66, 60), (65, 60), (64, 60), (63, 60), (62, 60), (61, 60), (60, 60), (59, 60), (58, 60), (57, 60), (56, 60), (55, 60), (54, 60), (53, 60), (52, 60), (51, 60), (50, 60), (49, 60), (48, 60), (47, 60), (46, 60), (46, 61), (45, 61), (45, 62), (45, 63), (44, 63), (43, 63), (43, 64), (42, 64), (42, 65), (41, 65), (41, 66), (41, 67), (40, 67), (39, 67), (38, 67), (37, 67), (36, 67), (35, 67), (34, 67), (33, 67), (32, 67), (31, 67), (30, 67), (29, 67), (28, 67), (28, 68), (28, 69), (28, 70), (28, 71), (28, 72), (28, 73), (28, 74), (28, 75), (28, 76), (28, 77), (28, 78), (28, 79), (27, 79), (27, 80)]])


=======
    print('Detecting! Send "OK" message to {} to register most recent estimate.'.format(DRIVER_TOPIC))
    while driver.is_detecting() or not driver._pather.grid.is_free(driver._pather.grid.coord_to_grid(driver._detector_info['target'])):
        if not driver.is_detecting():
            print('ATTENTION: Target is not an empty cell and therefore will be emptied.')
            driver._detector_info['target'] = None
            driver._detector_info['active'] = 1
    print('Pathing!')
    path = driver._pather.find_path_for_coords(driver._detector_info['start'], driver._detector_info['target'])
    driver._pather.publish_pose_markers_from_path(path)
    #convert path indices to coordinates
    path = [driver._pather.grid.grid_to_coord(point) for point in path]
    print("Path is", path)
    print("Time to follow the path")
    #driver._vcont.follow(path)
    driver._pid.follow(path)

    print('Great success')
>>>>>>> e6c18a9ab6d59e0c03676650f06083a9f7098c1b
    # except:
    #     rospy.logerr("ROS node interrupted.")
