#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Qiyao Zuo
# Date: October 22, 2021
__author__ = "Qiyao Zuo"
__course__ = "Principles of Robot Design and Programming (FA21)"
__assignment__ = "pa3 planning"
# A* algorithm reference: "https://www.annytab.com/a-star-search-algorithm-in-python/"

# Import of python modules.
import math # use of pi.
import random
from random import randrange
import numpy as np
from collections import deque
import networkx as nx


# import of relevant libraries.
import rospy # module for ROS APIs
import tf
from geometry_msgs.msg import Twist, PoseStamped # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan'
DEFAULT_ODOM_TOPIC = 'odom'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

dimension=0
data=[]

class RandomWalk():
    def __init__(self):
        """Constructor."""

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
        self.marker_pub = rospy.Publisher("markers", Marker, queue_size=1)
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()
        #self.map


    def publish_marker(self, translation, quaternion, index):
        """add marker to markers array"""
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = "map"

        marker_msg.action = Marker.ADD
        marker_msg.type = Marker.ARROW
        marker_msg.id = index # note to add multiple markers, the ID should be changed

        marker_msg.pose.position.x = translation[1]*0.1 - 5.0
        marker_msg.pose.position.y = translation[0]*0.1 - 5.0
        marker_msg.pose.position.z = 0

        marker_msg.pose.orientation.x = quaternion[0]
        marker_msg.pose.orientation.y = quaternion[1]
        marker_msg.pose.orientation.z = quaternion[2]
        marker_msg.pose.orientation.w = quaternion[3]

        marker_msg.color.r = 1.0
        marker_msg.color.a = 1
        marker_msg.scale.x = 0.15
        marker_msg.scale.y = 0.15
        marker_msg.scale.z = 0.1

        self.marker_array.markers.append(marker_msg)


    def map_callback(self, msg):
        self.map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution)
        global dimension
        global data
        dimension = msg.info.width
        data=msg.data


    def show_path(self, path):
        '''get the path and publish markers'''

        if(path):
            i=0
            for p in path[:len(path)-1]:
                #print(p)
                yaw= get_rotation(p,path[i+1])
                quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
                self.publish_marker(p, quaternion, i)
                i+=1
            self.publisher.publish(self.marker_array)
        else:
            print("no path found")


def get_rotation(pos, next):
    '''get the rotation from current to the next grid'''
    x=next[0]-pos[0]
    y=next[1]-pos[1]
    return math.atan2(y,x)

class Grid:
    '''grid class stores grid map information'''
    def __init__(self, occupancy_grid_data, width, height, resolution):
        self.grid = np.reshape(occupancy_grid_data, (height, width))
        self.resolution = resolution
        self.data = occupancy_grid_data
        self.dimension=width

    def cell_at(self, x, y):
        return self.grid[y, x]

    def path_search(self, start, end):
        '''search for a path and store in index in a array'''
        # path = astar(self.data, start, end)
        G=nx.grid_graph(dim=[self.dimension,self.dimension])
        ebunch = G.edges()
        G.remove_edges_from(ebunch)

        edges=[]

        for row_index in range(self.dimension):
            for element_index in range(self.dimension):

                if self.data[index_to_array(row_index,element_index)] != 0: #if this gird is occupied
                    continue

                for neighbors in [(0, 1), (0, -1), (-1, 0), (1, 0), (-1, -1), (1, 1), (-1,1),(1,-1)]: # Adjacent squares

                    node_position =[row_index + neighbors[0],element_index + neighbors[1]]

                    # Make sure within range
                    if node_position[0] > (self.dimension - 1) or node_position[0] < 0 or node_position[1] > (self.dimension -1) or node_position[1] < 0:
                        continue

                    # Make sure walkable terrains
                    if self.data[index_to_array(node_position[0],node_position[1])] != 0:
                        continue

                    edges.append(((row_index,element_index), (node_position[0],node_position[1])))


        G.add_edges_from(edges)  # using a list of edge tuples
        path=nx.astar_path(G,start,end,heuristic=dis_to_wall)
        print(path)
        return path



def index_to_array(i,j):
    return i * dimension+j



def dis_to_wall(a, b):
    dis_to_target=dist(a,b)
    for dis in range(1,11):
        for neighbors in [(0, 1), (0, -1), (-1, 0), (1, 0), (-1, -1), (1, 1), (-1,1),(1,-1)]: # Adjacent squares

                        node_position =[a[0] + dis * neighbors[0], a[1] + dis * neighbors[1]]

                        # Make sure within range
                        if node_position[0] > (dimension - 1) or node_position[0] < 0 or node_position[1] > (dimension -1) or node_position[1] < 0:
                            dis_wall= dist(a, (a[0] + dis * neighbors[0], a[1] + dis * neighbors[1]))
                            return -dis_wall*6 + dis_to_target

                        # Make sure walkable terrains
                        if data[index_to_array(node_position[0],node_position[1])] != 0:
                            dis_wall= dist(a, (a[0] + dis * neighbors[0], a[1] + dis * neighbors[1]))
                            return -dis_wall*6 + dis_to_target



    return dis_to_target

def dist(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("random_walk",log_level=rospy.DEBUG)



    # Initialization of the class for the random walk.
    random_walk = RandomWalk()

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(4)
    global dimension


    # Robot random walks.
    try:
        path=random_walk.map.path_search((80, 63),(60, 20))
        path=random_walk.path_search((80, 63),(60, 20))
        random_walk.show_path(path)
        print("publish")

    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

    rospy.sleep(2)


if __name__ == "__main__":
    """Run the main function."""
    main()
