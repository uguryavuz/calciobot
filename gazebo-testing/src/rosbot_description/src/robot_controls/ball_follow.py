#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: TODO: Jeffrey Cho
# Date: TODO: 11/12/21

# Import of python modules.
import math # use of pi.

import random

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# delta time
dt = 0.1 # seconds

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.25 # m/s
#ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
# MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
#MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
DESIRED_DISTANCE = 0.5 # m what we will use to calculate error


# Field of view in radians that is checked in front of the robot (feel free to tune)
#MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi;
#MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi;
#MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi;
#MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi;

# RADIUS OF CUBE/BALL
RADIUS = 1

MIN_SCAN_ANGLE_RAD = -math.pi / 4
MAX_SCAN_ANGLE_RAD = math.pi / 4

KP = 1

KD = 38

class Pid():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, delt_time = dt, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD]):
        """Constructor."""
        self.dt = delt_time
        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = 0 # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle
        # self.distance = distance
        
        self.errors = []

        # Flag used to control the behavior of the robot.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: assumption: the one at angle 0 corresponds to the front.
        
        
        #if not self._close_obstacle:
            # Find the minimum range value between min_scan_angle and
            # max_scan_angle
            # If the minimum range value is closer to min_threshold_distance, change the flag self._close_obstacle
            # Note: You have to find the min index and max index.
            # Please double check the LaserScan message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
            ####### TODO: ANSWER CODE BEGIN #######

        min_index = int((MIN_SCAN_ANGLE_RAD - msg.angle_min)/msg.angle_increment)
        max_index = int((MAX_SCAN_ANGLE_RAD - msg.angle_min)/msg.angle_increment)

        # slicing the list for our new angle ranges
        new_list = msg.ranges[min_index:max_index+1]

        # min_range_value
        min_value = min(new_list)

        self.errors.append(self.distance - min_value)
        # if too close to obstacle
        # turn counterclockwise
    
        # if min_value < self.distance:
                #self._close_obstacle = True

        mid = int(len(new_list) // 2)
        # calculate the theta to turn

        min_index = new_list.index(min_value)

        distance = abs(mid - min_index)

        # turn clockwise
        #if msg.range.index(min_value) > msg.ranges[mid]:
        angle = math.atan((RADIUS)/distance)
                    #angle = math.atan((MIN_THRESHOLD_DISTANCE + RADIUS)/distance)

        self.angular_velocity = 90 - angle

        if msg.range.index(min_value) > msg.ranges[mid]:
            self.angular_velocity = abs(self.angular_velocity)

        if msg.range.index(min_value) < msg.ranges[mid]:
            self.angular_velocity = -1 * abs(self.angular_velocity)

        else:
            self.angular_velocity = 0
            # else turn counterclockwise
                
               

            # else turn clockwise
            #if min_value > self.distance:
            #    self._close_obstacle = True
            ####### ANSWER CODE END #######

    #def turn_clock(self, av):
        # positive radians
    #def turn_counter(self, av):
        # negative radians
    def ball_follow(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C
            
            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random amount of time
            # after which the flag is set again to False.
            # Use the function move already implemented, passing the default velocities saved in the corresponding class members.

            ####### TODO: ANSWER CODE BEGIN #######

            # If the flag is False, the robot should move forward
            # if not self._close_obstacle:
            #self.move(self.linear_velocity, 0)
            
            # else:

                # stop
                # self.stop()

                # randomly pick new angular velocity between positive and negative pi/2
                
                #random_angular_velocity = random.uniform(-1*math.pi, math.pi)
                # if len(self.errors) < 2:
                #     self.angular_velocity = KP * self.errors[-1] 

                # else:
                #     self.angular_velocity = KP * self.errors[-1] + KD * ((self.errors[-1] - self.errors[-2])/self.dt)

                
                #For example, if the value of the angle is fin the interval (-pi, 0) it should rotate counterclockwise
                #(i.e., positive angular velocity), while if angle is in the interval (0, pi) it should rotate clockwise 
                #(i.e., negative angular velocity).


            self.move(LINEAR_VELOCITY, self.angular_velocity)

                #self.move(0, random_angular_velocity)
                #self._close_obstacle = False
            ####### ANSWER CODE END #######

            rate.sleep()
        

def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("Follow_Wall")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    PID = Pid()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(PID.stop)

    # Robot random walks.
    try:
        PID.ball_follow()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
