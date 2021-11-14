#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: TODO: Jeffrey Cho
# Date: TODO: 10/13/21

# Import of python modules.
import math # use of pi.

import random
from numpy.lib.function_base import interp
from scipy import interpolate
import numpy as np
import tf

from tf.transformations import euler_from_quaternion


# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan'
DEFAULT_ODOM_TOPIC = 'odom'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# delta time
dt = 0.1 # seconds

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.2 # m/s
#ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
# MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
DESIRED_DISTANCE = 0 # m what we will use to calculate error


# Field of view in radians that is checked in front of the robot (feel free to tune)
#MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi;
#MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi;
#MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi;
#MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi;

MIN_SCAN_ANGLE_RAD = -1 * math.pi;
MAX_SCAN_ANGLE_RAD = + math.pi;

path = [(-1.9999999552965164, 2.0000001043081284), (-1.8999999538064003, 2.0000001043081284), (-1.7999999523162842, 2.0000001043081284), (-1.6999999508261681, 2.0000001043081284), (-1.5999999493360519, 2.0000001043081284), (-1.4999999478459358, 2.0000001043081284), (-1.4999999478459358, 1.9000001028180122), (-1.4999999478459358, 1.8000001013278961), (-1.4999999478459358, 1.70000009983778), (-1.4999999478459358, 1.6000000983476639), (-1.4999999478459358, 1.5000000968575478), (-1.4999999478459358, 1.4000000953674316), (-1.4999999478459358, 1.3000000938773155), (-1.4999999478459358, 1.2000000923871994), (-1.4999999478459358, 1.1000000908970833), (-1.4999999478459358, 1.0000000894069672), (-1.4999999478459358, 0.90000008791685104), (-1.4999999478459358, 0.80000008642673492), (-1.4999999478459358, 0.7000000849366188), (-1.4999999478459358, 0.60000008344650269), (-1.4999999478459358, 0.50000008195638657), (-1.4999999478459358, 0.40000008046627045), (-1.4999999478459358, 0.30000007897615433), (-1.4999999478459358, 0.20000007748603821), (-1.4999999478459358, 0.10000007599592209), (-1.4999999478459358, 7.4505805969238281e-08), (-1.4999999478459358, -0.09999992698431015), (-1.4999999478459358, -0.19999992847442627), (-1.4999999478459358, -0.29999992996454239), (-1.4999999478459358, -0.39999993145465851), (-1.4999999478459358, -0.49999993294477463), (-1.4999999478459358, -0.59999993443489075), (-1.4999999478459358, -0.69999993592500687), (-1.4999999478459358, -0.79999993741512299), (-1.4999999478459358, -0.89999993890523911), (-1.4999999478459358, -0.99999994039535522), (-1.4999999478459358, -1.0999999418854713), (-1.4999999478459358, -1.1999999433755875), (-1.4999999478459358, -1.2999999448657036), (-1.3999999463558197, -1.2999999448657036), (-1.2999999448657036, -1.2999999448657036), (-1.1999999433755875, -1.2999999448657036), (-1.0999999418854713, -1.2999999448657036), (-0.99999994039535522, -1.2999999448657036), (-0.89999993890523911, -1.2999999448657036), (-0.79999993741512299, -1.2999999448657036), (-0.69999993592500687, -1.2999999448657036), (-0.59999993443489075, -1.2999999448657036), (-0.49999993294477463, -1.2999999448657036), (-0.39999993145465851, -1.2999999448657036), (-0.29999992996454239, -1.2999999448657036), (-0.29999992996454239, -1.1999999433755875), (-0.29999992996454239, -1.0999999418854713), (-0.29999992996454239, -0.99999994039535522), (-0.29999992996454239, -0.89999993890523911), (-0.29999992996454239, -0.79999993741512299), (-0.29999992996454239, -0.69999993592500687), (-0.29999992996454239, -0.59999993443489075), (-0.29999992996454239, -0.49999993294477463), (-0.29999992996454239, -0.39999993145465851), (-0.29999992996454239, -0.29999992996454239), (-0.29999992996454239, -0.19999992847442627), (-0.29999992996454239, -0.09999992698431015), (-0.29999992996454239, 7.4505805969238281e-08), (-0.29999992996454239, 0.10000007599592209), (-0.29999992996454239, 0.20000007748603821), (-0.29999992996454239, 0.30000007897615433), (-0.29999992996454239, 0.40000008046627045), (-0.29999992996454239, 0.50000008195638657), (-0.29999992996454239, 0.60000008344650269), (-0.29999992996454239, 0.7000000849366188), (-0.19999992847442627, 0.7000000849366188), (-0.19999992847442627, 0.80000008642673492), (-0.09999992698431015, 0.80000008642673492), (-0.09999992698431015, 0.90000008791685104), (7.4505805969238281e-08, 0.90000008791685104), (7.4505805969238281e-08, 1.0000000894069672), (7.4505805969238281e-08, 1.1000000908970833), (0.10000007599592209, 1.1000000908970833), (0.20000007748603821, 1.1000000908970833), (0.20000007748603821, 1.2000000923871994), (0.30000007897615433, 1.2000000923871994), (0.30000007897615433, 1.3000000938773155), (0.40000008046627045, 1.3000000938773155), (0.40000008046627045, 1.4000000953674316), (0.40000008046627045, 1.5000000968575478), (0.50000008195638657, 1.5000000968575478), (0.60000008344650269, 1.5000000968575478), (0.60000008344650269, 1.6000000983476639), (0.7000000849366188, 1.6000000983476639), (0.7000000849366188, 1.70000009983778), (0.80000008642673492, 1.70000009983778), (0.80000008642673492, 1.8000001013278961), (0.80000008642673492, 1.9000001028180122), (0.90000008791685104, 1.9000001028180122), (1.0000000894069672, 1.9000001028180122), (1.0000000894069672, 2.0000001043081284), (1.1000000908970833, 2.0000001043081284), (1.1000000908970833, 2.1000001057982445), (1.2000000923871994, 2.1000001057982445), (1.2000000923871994, 2.2000001072883606), (1.2000000923871994, 2.3000001087784767), (1.3000000938773155, 2.3000001087784767), (1.4000000953674316, 2.3000001087784767), (1.4000000953674316, 2.4000001102685928), (1.5000000968575478, 2.4000001102685928), (1.5000000968575478, 2.500000111758709), (1.6000000983476639, 2.500000111758709), (1.6000000983476639, 2.6000001132488251), (1.6000000983476639, 2.7000001147389412), (1.70000009983778, 2.7000001147389412), (1.8000001013278961, 2.7000001147389412), (1.8000001013278961, 2.8000001162290573), (1.9000001028180122, 2.8000001162290573), (1.9000001028180122, 2.9000001177191734), (2.0000001043081284, 2.9000001177191734), (2.0000001043081284, 3.0000001192092896)]

KP = 1

KD = 30

def calculate_distance(path):
    distance = 0
    for i in range(1, len(path)):
        distance+=(math.sqrt((path[i-1][0]- path[i][0])**2 + (path[i - 1][1] - path[i][1])**2))
    
    return distance

def B_spline(path):
    x = []
    y = []
    for point in path:
        x.append(point[0])
        y.append(point[1])

    tck, rest = interpolate.splprep([x, y])
    u = np.linspace(0, 1, num=100)
    smooth = interpolate.splev(u, tck)

    return smooth

KP = 1

KD = 38





class Pid():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, delt_time = dt, distance = DESIRED_DISTANCE, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD]):
        """Constructor."""
        self.dt = delt_time
        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        self._odom = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = 0.2 # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle
        self.distance = distance
        self.index = 0
        self.errors = []
        self.yaw = 0

        

        self._on_path = False
        # Flag used to control the behavior of the robot.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.
    
    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def rotate_rel(self, theta):
        start_time = rospy.get_rostime()
        duration = rospy.Duration(abs(theta) / self.angular_velocity)

        while rospy.get_rostime() - start_time < duration:
            self.move(0, self.angular_velocity if theta > 0 else -self.angular_velocity)
            self.rate.sleep()

        self.stop()

    def rotate_abs(self, theta):
        """ Rotate the robot to the specified absolute angle. """
        direction = 1 if theta > self.orientation[2] else -1
        delta_theta = abs(theta - self.orientation[2]) % (2 * math.pi)

        self.rotate_rel(delta_theta * direction)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    

    def _odom_callback(self, msg):

        # defiinig odom callback
        robot_pose = msg.pose.pose

        # calculating transformation matrix
        quarternion = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
        euler = tf.transformation.euler_from_quarternion(quarternion)
        euler = tf.transformations.euler_from_quaternion(quarternion)
        self.yaw = euler[2]

        x = robot_pose.orientation.x
        y = robot_pose.orientation.y

        


        #total_distance = calculate_distance(path)
        #smooth = B_spline(path, total_distance)



        total_distance = self.calculate_distance(self.path)
        smooth = self.B_spline(self.path, total_distance)

        x_values = smooth[0]
        y_values = smooth[1]

        error = math.sqrt((x - x_values[self.index])**2 + (y -y_values[self.index])**2)
        self.errors.append(error)

        self.index+=1 
        error = math.sqrt((x - x_values[self.index]) **2 + (y -y_values[self.index])**2 )
        self.errors.append(error)

        self.index+=1

        # printing transformation matrix
        print("The euler", euler)

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: assumption: the one at angle 0 corresponds to the front.
        
        
        #if not self._close_obstacle:


       # if not self._close_obstacle:
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
            
        # slicing the list for our new angle ranges
        new_list = msg.ranges[min_index:max_index+1]

        # min_range_value
        min_value = min(new_list)

        #self.errors.append(self.distance - min_value)
        # if too close to obstacle
        # turn counterclockwise
    
        
            

        # if min_value < MIN_THRESHOLD_DISTANCE:
        #     self._close_obstacle = True


            # else turn clockwise
            #if min_value > self.distance:
            #    self._close_obstacle = True
            ####### ANSWER CODE END #######

    #def turn_clock(self, av):
        # positive radians
    #def turn_counter(self, av):
        # negative radians
    def spin(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.

        smooth = B_spline(path)
        x_values = smooth[0]
        y_values = smooth[1]

        for i in range(1, 100):
        #while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C
            
        #while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C

            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random amount of time
            # after which the flag is set again to False.
            # Use the function move already implemented, passing the default velocities saved in the corresponding class members.

            ####### TODO: ANSWER CODE BEGIN #######

            # If the flag is False, the robot should move forward
            #if not self._close_obstacle:
            distance = math.sqrt((self.orientation.x - x_values[i - 1])**2 + (self.orientation.y - y_values[i - 1])**2)
            if distance != 0:
                if len(self.errors) < 2:
                    self.angular_velocity = KP * self.errors[-1] 

                else:
                    self.angular_velocity = KP * self.errors[-1] + KD * ((self.errors[-1] - self.errors[-2])/self.dt)

               
                theta = math.atan2(y_values[i] - y_values[i-1], x_values[i] - x_values[i-1]) - self.yaw
                length = math.sqrt((x_values[i] - x_values[i - 1])**2 + (y_values[i] - y_values[i - 1])**2)

                self.rotate_abs(theta)
                self.move(LINEAR_VELOCITY, 0)
            else:
                theta = math.atan2(y_values[i] - y_values[i-1], x_values[i] - x_values[i-1]) - self.yaw
                length = math.sqrt((x_values[i] - x_values[i - 1])**2 + (y_values[i] - y_values[i - 1])**2)

                self.rotate_abs(theta)
                self.move(LINEAR_VELOCITY, 0)


            
            #else:

                # stop
            #self.stop()

            # randomly pick new angular velocity between positive and negative pi/2
            
            #random_angular_velocity = random.uniform(-1*math.pi, math.pi)
            # if len(self.errors) < 2:
            #     self.angular_velocity = KP * self.errors[-1] 

            # else:
            #     self.angular_velocity = KP * self.errors[-1] + KD * ((self.errors[-1] - self.errors[-2])/self.dt)

            
            #For example, if the value of the angle is fin the interval (-pi, 0) it should rotate counterclockwise
            #(i.e., positive angular velocity), while if angle is in the interval (0, pi) it should rotate clockwise 
            #(i.e., negative angular velocity).

            
            #self.move(LINEAR_VELOCITY, self.angular_velocity)

                #self.move(0, random_angular_velocity)
                #self._close_obstacle = False
            ####### ANSWER CODE END #######

            rate.sleep()
        

    def main():
        """Main function."""

        # 1st. initialization of node.
        rospy.init_node("Follow_Path")

        # Sleep for a few seconds to wait for the registration.
        rospy.sleep(2)

        # Initialization of the class for the random walk.
        PID = Pid()

        # If interrupted, send a stop command before interrupting.
        rospy.on_shutdown(PID.stop)

        # Robot random walks.
        try:
            PID.spin()
        except rospy.ROSInterruptException:
            rospy.logerr("ROS node interrupted.")
            if not self._close_obstacle:
                self.move(self.linear_velocity, 0)

            else:

                # stop
                self.stop()

                # randomly pick new angular velocity between positive and negative pi/2

                #random_angular_velocity = random.uniform(-1*math.pi, math.pi)
                if len(self.errors) < 2:
                    self.angular_velocity = KP * self.errors[-1]

                else:
                    self.angular_velocity = KP * self.errors[-1] + KD * ((self.errors[-1] - self.errors[-2])/self.dt)


                #For example, if the value of the angle is fin the interval (-pi, 0) it should rotate counterclockwise
                #(i.e., positive angular velocity), while if angle is in the interval (0, pi) it should rotate clockwise
                #(i.e., negative angular velocity).


                self.move(LINEAR_VELOCITY, self.angular_velocity)

                #self.move(0, random_angular_velocity)
                self._close_obstacle = False
            ####### ANSWER CODE END #######

            rate.sleep()

    def follow(self,path):

        print("PID follower activated")
        self.path= path
        total_distance = self.calculate_distance(path)
        smooth = self.B_spline(path, total_distance)
        x_values = smooth[0]
        y_values = smooth[1]
        plt.plot(x_values, y_values)
        self.spin()


def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("Follow_Path")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    PID = Pid()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(PID.stop)

    # Robot random walks.
    try:
        path = [(-1.9999999552965164, 2.0000001043081284), (-1.8999999538064003, 2.0000001043081284), (-1.7999999523162842, 2.0000001043081284), (-1.6999999508261681, 2.0000001043081284), (-1.5999999493360519, 2.0000001043081284), (-1.4999999478459358, 2.0000001043081284), (-1.4999999478459358, 1.9000001028180122), (-1.4999999478459358, 1.8000001013278961), (-1.4999999478459358, 1.70000009983778), (-1.4999999478459358, 1.6000000983476639), (-1.4999999478459358, 1.5000000968575478), (-1.4999999478459358, 1.4000000953674316), (-1.4999999478459358, 1.3000000938773155), (-1.4999999478459358, 1.2000000923871994), (-1.4999999478459358, 1.1000000908970833), (-1.4999999478459358, 1.0000000894069672), (-1.4999999478459358, 0.90000008791685104), (-1.4999999478459358, 0.80000008642673492), (-1.4999999478459358, 0.7000000849366188), (-1.4999999478459358, 0.60000008344650269), (-1.4999999478459358, 0.50000008195638657), (-1.4999999478459358, 0.40000008046627045), (-1.4999999478459358, 0.30000007897615433), (-1.4999999478459358, 0.20000007748603821), (-1.4999999478459358, 0.10000007599592209), (-1.4999999478459358, 7.4505805969238281e-08), (-1.4999999478459358, -0.09999992698431015), (-1.4999999478459358, -0.19999992847442627), (-1.4999999478459358, -0.29999992996454239), (-1.4999999478459358, -0.39999993145465851), (-1.4999999478459358, -0.49999993294477463), (-1.4999999478459358, -0.59999993443489075), (-1.4999999478459358, -0.69999993592500687), (-1.4999999478459358, -0.79999993741512299), (-1.4999999478459358, -0.89999993890523911), (-1.4999999478459358, -0.99999994039535522), (-1.4999999478459358, -1.0999999418854713), (-1.4999999478459358, -1.1999999433755875), (-1.4999999478459358, -1.2999999448657036), (-1.3999999463558197, -1.2999999448657036), (-1.2999999448657036, -1.2999999448657036), (-1.1999999433755875, -1.2999999448657036), (-1.0999999418854713, -1.2999999448657036), (-0.99999994039535522, -1.2999999448657036), (-0.89999993890523911, -1.2999999448657036), (-0.79999993741512299, -1.2999999448657036), (-0.69999993592500687, -1.2999999448657036), (-0.59999993443489075, -1.2999999448657036), (-0.49999993294477463, -1.2999999448657036), (-0.39999993145465851, -1.2999999448657036), (-0.29999992996454239, -1.2999999448657036), (-0.29999992996454239, -1.1999999433755875), (-0.29999992996454239, -1.0999999418854713), (-0.29999992996454239, -0.99999994039535522), (-0.29999992996454239, -0.89999993890523911), (-0.29999992996454239, -0.79999993741512299), (-0.29999992996454239, -0.69999993592500687), (-0.29999992996454239, -0.59999993443489075), (-0.29999992996454239, -0.49999993294477463), (-0.29999992996454239, -0.39999993145465851), (-0.29999992996454239, -0.29999992996454239), (-0.29999992996454239, -0.19999992847442627), (-0.29999992996454239, -0.09999992698431015), (-0.29999992996454239, 7.4505805969238281e-08), (-0.29999992996454239, 0.10000007599592209), (-0.29999992996454239, 0.20000007748603821), (-0.29999992996454239, 0.30000007897615433), (-0.29999992996454239, 0.40000008046627045), (-0.29999992996454239, 0.50000008195638657), (-0.29999992996454239, 0.60000008344650269), (-0.29999992996454239, 0.7000000849366188), (-0.19999992847442627, 0.7000000849366188), (-0.19999992847442627, 0.80000008642673492), (-0.09999992698431015, 0.80000008642673492), (-0.09999992698431015, 0.90000008791685104), (7.4505805969238281e-08, 0.90000008791685104), (7.4505805969238281e-08, 1.0000000894069672), (7.4505805969238281e-08, 1.1000000908970833), (0.10000007599592209, 1.1000000908970833), (0.20000007748603821, 1.1000000908970833), (0.20000007748603821, 1.2000000923871994), (0.30000007897615433, 1.2000000923871994), (0.30000007897615433, 1.3000000938773155), (0.40000008046627045, 1.3000000938773155), (0.40000008046627045, 1.4000000953674316), (0.40000008046627045, 1.5000000968575478), (0.50000008195638657, 1.5000000968575478), (0.60000008344650269, 1.5000000968575478), (0.60000008344650269, 1.6000000983476639), (0.7000000849366188, 1.6000000983476639), (0.7000000849366188, 1.70000009983778), (0.80000008642673492, 1.70000009983778), (0.80000008642673492, 1.8000001013278961), (0.80000008642673492, 1.9000001028180122), (0.90000008791685104, 1.9000001028180122), (1.0000000894069672, 1.9000001028180122), (1.0000000894069672, 2.0000001043081284), (1.1000000908970833, 2.0000001043081284), (1.1000000908970833, 2.1000001057982445), (1.2000000923871994, 2.1000001057982445), (1.2000000923871994, 2.2000001072883606), (1.2000000923871994, 2.3000001087784767), (1.3000000938773155, 2.3000001087784767), (1.4000000953674316, 2.3000001087784767), (1.4000000953674316, 2.4000001102685928), (1.5000000968575478, 2.4000001102685928), (1.5000000968575478, 2.500000111758709), (1.6000000983476639, 2.500000111758709), (1.6000000983476639, 2.6000001132488251), (1.6000000983476639, 2.7000001147389412), (1.70000009983778, 2.7000001147389412), (1.8000001013278961, 2.7000001147389412), (1.8000001013278961, 2.8000001162290573), (1.9000001028180122, 2.8000001162290573), (1.9000001028180122, 2.9000001177191734), (2.0000001043081284, 2.9000001177191734), (2.0000001043081284, 3.0000001192092896)]

        PID.follow(path)
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
    total_distance = calculate_distance(path)
    smooth = B_spline(path, total_distance)

    x_values = smooth[0]
    y_values = smooth[1]
    plt.plot(x_values, y_values)



    #total_distance = calculate_distance(path)
    #smooth = B_spline(path, total_distance)

    #x_values = smooth[0]
    #y_values = smooth[1]
    #plt.plot(x_values, y_values)
    #for i in range(len(smooth)):
