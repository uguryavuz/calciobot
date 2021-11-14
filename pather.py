#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Author: Qiyao Zuo, Ugur Y. Yavuz
# Date: 11/12/2021
# Final Project: "CalcioBot" for COSC 281 (21F, Dartmouth College).

# Import of relevant libraries.
from __future__ import division, print_function
import numpy as np                       # Math
import rospy, tf                         # ROS API, transformations
import networkx as nx                    # For search algorithms
from scipy.ndimage import median_filter  # For path smoothing

# Messages
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

# Topics
SCAN_TOPIC = 'scan'
MAP_TOPIC = 'map'
ODOM_TOPIC = 'odom'
MARKER_TOPIC = 'visualization_marker_array'

# Frame names
MAP_FRAME = 'map'
OCCUPANCY_THOLD = 50

# Loop frequency (in 1/s i.e. Hertz; for msg sending main loop.)
FREQUENCY = 30

# Class to store grid
class Grid:
    # Initialize grid instance
    def __init__(self, occupancy_grid_data, width, height, resolution, origin_position, origin_rotation_q):
        self.width, self.height = width, height
        self.grid = np.reshape(occupancy_grid_data, (height, width))
        self.resolution = resolution
        self.origin_position, self.origin_rotation_q = origin_position, origin_rotation_q

        # Compute 4x4 homogeneous 4x4 transformation matrix.
        t = tf.transformations.translation_matrix([origin_position.x, origin_position.y, origin_position.z])
        R = tf.transformations.quaternion_matrix([origin_rotation_q.x, origin_rotation_q.y, origin_rotation_q.z, origin_rotation_q.w])
        self.tr_matrix = t.dot(R)

    def cell_at(self, x, y):
        return self.grid[y, x]

    # Turn the grid indices into global coordinates.
    def grid_to_coord(self, xytuple):
        return tuple(self.tr_matrix.dot(np.array([xytuple[0] * self.resolution, xytuple[1] * self.resolution, 0, 1])))[:2]

    # Turn the global coordinates into closest grid indices.
    def coord_to_grid(self, xytuple):
        return tuple(map(lambda coord: int(np.round(coord / self.resolution)), np.linalg.inv(self.tr_matrix).dot(np.array([xytuple[0], xytuple[1], 0, 1]))[:2]))

    # Returns if a cell in the grid with given x, y indices is not occupied.
    def is_free(self, xytuple):
        return 0 <= self.grid[xytuple[1], xytuple[0]] < OCCUPANCY_THOLD

class GridPather():
    def __init__(self, freq=FREQUENCY):
        # Transformation listener
        self._trans_lst = tf.TransformListener()

        # Topic subscribers/publishers
        self._map_sub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self._map_callback, queue_size=1)
        self._marker_pub = rospy.Publisher(MARKER_TOPIC, MarkerArray, queue_size=1)
        
        # Marker array to be published, and ID counter for individual markers.
        self._marker_array = MarkerArray()
        self._marker_id_cnt = 0

        # Variable containing occupancy grid.
        self.grid = None

        # Message transmission rate
        self._freq = freq
        self._rate = rospy.Rate(freq)

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

    # Update grid information published to the 'map' topic.
    def _map_callback(self, msg):
        self.grid = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin.position, msg.info.origin.orientation)

    # Add new marker to marker array class variable
    def _add_marker(self, x, y, theta):
        # Set marker instance and header.
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = MAP_FRAME

        # Define shape and assign unique id currently in id_counter.
        marker_msg.action = Marker.ADD
        marker_msg.type = Marker.ARROW
        marker_msg.id = self._marker_id_cnt
        # Increment id_counter.
        self._marker_id_cnt += 1

        # Assign specified pose to marker.
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0, 0, theta)
        marker_msg.pose.orientation.x = q[0]
        marker_msg.pose.orientation.y = q[1]
        marker_msg.pose.orientation.z = q[2]
        marker_msg.pose.orientation.w = q[3]

        # Specify color and transparency.
        marker_msg.color.r = 1
        marker_msg.color.a = 1

        # Specify scale
        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.03
        marker_msg.scale.z = 0.03

        # Add to markers array
        self._marker_array.markers.append(marker_msg)

    # 
    def publish_pose_markers_from_path(self, path, filter_size=3):
        coords_in_path = [self.grid.grid_to_coord(point) for point in path]
        smooth_coords = median_filter(np.array([coords_in_path[0]] * filter_size + coords_in_path + [coords_in_path[-1]] * filter_size), size=filter_size, mode='reflect')[(filter_size-1):-(filter_size-1)]

        for i, cur_coord in enumerate(smooth_coords):
            # Compute previous, current and next point.
            prev_coord = smooth_coords[i] if i == 0 else smooth_coords[i-1]
            next_coord = smooth_coords[i] if i == len(smooth_coords)-1 else smooth_coords[i+1]
            
            # Compute angle between the previous and next points using arctan, then append the pose to the list of poses.
            angle = np.arctan2([next_coord[1] - prev_coord[1]], [next_coord[0] - prev_coord[0]])[0]
            self._add_marker(cur_coord[0], cur_coord[1], angle)
            
        # Publish array, then empty it and reset counter.
        self._marker_pub.publish(self._marker_array)
        self._marker_array, self._marker_id_cnt = [], 0

    # Heuristic 1: distance to target
    def dist_to_target(self, point, target):
        return np.linalg.norm(np.array(point) - np.array(target))

    # Heuristic 2: distance to target -- but stay away from walls within a square of radius window. k is the weight assign to distance to wall.
    def cautious_dist_to_target(self, point, target, window=10, k=20):
        wall_distances_in_window = [np.linalg.norm(np.array((x, y)) - np.array(point)) for x in range(max(0, point[0]-window), min(self.grid.width-1, point[0]+window)+1) for y in range(max(0, point[1]-window), min(self.grid.height-1, point[1]+window)+1) if not self.grid.is_free((x, y))]
        min_dist_to_wall = np.inf if len(wall_distances_in_window) == 0 else min(wall_distances_in_window)
        return self.dist_to_target(point, target) * (1 + k / min_dist_to_wall)

    # Apply path searching using A*
    def find_path(self, start, end):
        while self.grid is None: continue
        G = nx.grid_2d_graph(self.grid.height, self.grid.width)
        G.remove_nodes_from([(x, y) for x in range(self.grid.width) for y in range(self.grid.height) if not self.grid.is_free((x, y))])
        try:
            return nx.astar_path(G, start, end, heuristic=self.cautious_dist_to_target)
        except nx.NetworkXNoPath as e:
            rospy.logerr(e)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('pather')
    rospy.sleep(2)

    # Initialize Detector instance
    pather = GridPather()

    # If interrupted, DO STH
    # rospy.on_shutdown()

    # Spin
    try:
        rospy.loginfo("Pather is spinnning.")
        while pather.grid is None: continue
        start = pather.grid.coord_to_grid((-2, 2))
        target = pather.grid.coord_to_grid((2, 3))
        print(start, target)
        path = pather.find_path(start, target)
        pather.publish_pose_markers_from_path(path)
        rospy.spin()
    except:
        rospy.logerr("ROS node interrupted.")