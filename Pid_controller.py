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
# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry


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
LINEAR_VELOCITY = 0.25 # m/s
#ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
# MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
DESIRED_DISTANCE = 0.5 # m what we will use to calculate error


# Field of view in radians that is checked in front of the robot (feel free to tune)
#MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi;
#MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi;
#MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi;
#MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi;

MIN_SCAN_ANGLE_RAD = -1 * math.pi
MAX_SCAN_ANGLE_RAD = + math.pi

KP = 1

KD = 38

path = [(-1.9999999552965164, 2.0000001043081284), (-1.8999999538064003, 2.0000001043081284), (-1.7999999523162842, 2.0000001043081284), (-1.6999999508261681, 2.0000001043081284), (-1.5999999493360519, 2.0000001043081284), (-1.4999999478459358, 2.0000001043081284), (-1.4999999478459358, 1.9000001028180122), (-1.4999999478459358, 1.8000001013278961), (-1.4999999478459358, 1.70000009983778), (-1.4999999478459358, 1.6000000983476639), (-1.4999999478459358, 1.5000000968575478), (-1.4999999478459358, 1.4000000953674316), (-1.4999999478459358, 1.3000000938773155), (-1.4999999478459358, 1.2000000923871994), (-1.4999999478459358, 1.1000000908970833), (-1.4999999478459358, 1.0000000894069672), (-1.4999999478459358, 0.90000008791685104), (-1.4999999478459358, 0.80000008642673492), (-1.4999999478459358, 0.7000000849366188), (-1.4999999478459358, 0.60000008344650269), (-1.4999999478459358, 0.50000008195638657), (-1.4999999478459358, 0.40000008046627045), (-1.4999999478459358, 0.30000007897615433), (-1.4999999478459358, 0.20000007748603821), (-1.4999999478459358, 0.10000007599592209), (-1.4999999478459358, 7.4505805969238281e-08), (-1.4999999478459358, -0.09999992698431015), (-1.4999999478459358, -0.19999992847442627), (-1.4999999478459358, -0.29999992996454239), (-1.4999999478459358, -0.39999993145465851), (-1.4999999478459358, -0.49999993294477463), (-1.4999999478459358, -0.59999993443489075), (-1.4999999478459358, -0.69999993592500687), (-1.4999999478459358, -0.79999993741512299), (-1.4999999478459358, -0.89999993890523911), (-1.4999999478459358, -0.99999994039535522), (-1.4999999478459358, -1.0999999418854713), (-1.4999999478459358, -1.1999999433755875), (-1.4999999478459358, -1.2999999448657036), (-1.3999999463558197, -1.2999999448657036), (-1.2999999448657036, -1.2999999448657036), (-1.1999999433755875, -1.2999999448657036), (-1.0999999418854713, -1.2999999448657036), (-0.99999994039535522, -1.2999999448657036), (-0.89999993890523911, -1.2999999448657036), (-0.79999993741512299, -1.2999999448657036), (-0.69999993592500687, -1.2999999448657036), (-0.59999993443489075, -1.2999999448657036), (-0.49999993294477463, -1.2999999448657036), (-0.39999993145465851, -1.2999999448657036), (-0.29999992996454239, -1.2999999448657036), (-0.29999992996454239, -1.1999999433755875), (-0.29999992996454239, -1.0999999418854713), (-0.29999992996454239, -0.99999994039535522), (-0.29999992996454239, -0.89999993890523911), (-0.29999992996454239, -0.79999993741512299), (-0.29999992996454239, -0.69999993592500687), (-0.29999992996454239, -0.59999993443489075), (-0.29999992996454239, -0.49999993294477463), (-0.29999992996454239, -0.39999993145465851), (-0.29999992996454239, -0.29999992996454239), (-0.29999992996454239, -0.19999992847442627), (-0.29999992996454239, -0.09999992698431015), (-0.29999992996454239, 7.4505805969238281e-08), (-0.29999992996454239, 0.10000007599592209), (-0.29999992996454239, 0.20000007748603821), (-0.29999992996454239, 0.30000007897615433), (-0.29999992996454239, 0.40000008046627045), (-0.29999992996454239, 0.50000008195638657), (-0.29999992996454239, 0.60000008344650269), (-0.29999992996454239, 0.7000000849366188), (-0.19999992847442627, 0.7000000849366188), (-0.19999992847442627, 0.80000008642673492), (-0.09999992698431015, 0.80000008642673492), (-0.09999992698431015, 0.90000008791685104), (7.4505805969238281e-08, 0.90000008791685104), (7.4505805969238281e-08, 1.0000000894069672), (7.4505805969238281e-08, 1.1000000908970833), (0.10000007599592209, 1.1000000908970833), (0.20000007748603821, 1.1000000908970833), (0.20000007748603821, 1.2000000923871994), (0.30000007897615433, 1.2000000923871994), (0.30000007897615433, 1.3000000938773155), (0.40000008046627045, 1.3000000938773155), (0.40000008046627045, 1.4000000953674316), (0.40000008046627045, 1.5000000968575478), (0.50000008195638657, 1.5000000968575478), (0.60000008344650269, 1.5000000968575478), (0.60000008344650269, 1.6000000983476639), (0.7000000849366188, 1.6000000983476639), (0.7000000849366188, 1.70000009983778), (0.80000008642673492, 1.70000009983778), (0.80000008642673492, 1.8000001013278961), (0.80000008642673492, 1.9000001028180122), (0.90000008791685104, 1.9000001028180122), (1.0000000894069672, 1.9000001028180122), (1.0000000894069672, 2.0000001043081284), (1.1000000908970833, 2.0000001043081284), (1.1000000908970833, 2.1000001057982445), (1.2000000923871994, 2.1000001057982445), (1.2000000923871994, 2.2000001072883606), (1.2000000923871994, 2.3000001087784767), (1.3000000938773155, 2.3000001087784767), (1.4000000953674316, 2.3000001087784767), (1.4000000953674316, 2.4000001102685928), (1.5000000968575478, 2.4000001102685928), (1.5000000968575478, 2.500000111758709), (1.6000000983476639, 2.500000111758709), (1.6000000983476639, 2.6000001132488251), (1.6000000983476639, 2.7000001147389412), (1.70000009983778, 2.7000001147389412), (1.8000001013278961, 2.7000001147389412), (1.8000001013278961, 2.8000001162290573), (1.9000001028180122, 2.8000001162290573), (1.9000001028180122, 2.9000001177191734), (2.0000001043081284, 2.9000001177191734), (2.0000001043081284, 3.0000001192092896)]


class Pid():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, delt_time = dt, distance = DESIRED_DISTANCE, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD]):
        """Constructor."""
        self.yaw = 0
        self.dt = delt_time
        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        self._odom = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = 0 # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle
        self.distance = distance
        self.predict_y = self.split_variables(path)
        self.path = []

        self.x_values = self.predict_y[0]
        self.y_values = self.predict_y[1]
        self.coeffs = self.best_line_fit(self.x_values, self.y_values)
        self.predicted_values = self.predict_path(self.x_values, self.coeffs)
        self.errors = []

        self.on_path = False



        # Flag used to control the behavior of the robot.

    def predict_path(self, x_coordinates, coeffs):

        values = []
        for x in x_coordinates:
            predicted_value = coeffs[0] * x**2 + coeffs[1] * x + coeffs[2]
            values.append(predicted_value)
            return values

    def split_variables(self, path):
        x = []
        y = []
        for coordinates in path:
            x.append(coordinates[0])
            y.append(coordinates[1])

        ans = [x, y]
        return ans

    def best_line_fit(self, x_coords, y_coords):
        x_array = np.array(x_coords)
        y_array = np.array(y_coords)

        a, b, c = np.polyfit(x_array, y_array, 2)
        coeffs = [a, b, c]
        return coeffs


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

    def move_forward(self, length):
        rate = rospy.Rate(FREQUENCY)
        start_time = rospy.get_rostime()
        duration = length/LINEAR_VELOCITY

        while rospy.get_rostime() - start_time <= rospy.Duration(duration):
             self.move(LINEAR_VELOCITY, 0)
             rate.sleep()

    def _odom_callback(self, msg):

        # defiinig odom callback
        robot_pose = msg.pose.pose

        # calculating transformation matrix
        quarternion = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
        euler = tf.transformation.euler_from_quarternion(quarternion)

        x = robot_pose.orientation.x
        y = robot_pose.orientation.y

        predicted_y = self.coeffs[0] * x**2 + self.coeffs[1] * x + self.coeffs[2]

        error = predicted_y - y

        if error <= self.distance:
            self.on_path = True
        else:
            self.on_path = False

        self.errors.append(error)

        self.yaw = euler[2]

    def rotate_rel(self, theta):
        start_time = rospy.get_rostime()
        duration = rospy.Duration(abs(theta) / self.angular_velocity)

        while rospy.get_rostime() - start_time < duration:
            self.move(0, self.angular_velocity if theta > 0 else -self.angular_velocity)
            self.rate.sleep()

        self.stop()

    def rotate_abs(self, theta):
        delta_angle = theta -self.yaw
        self.rotate_rel(delta_angle)

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




    def spin(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        #while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C
        length = len(self.predicted_values)
        for i in range(1, length):

            ####### TODO: ANSWER CODE BEGIN #######
            theta = math.atan2(self.x_values[i - 1] - self.x_values[i], self.predicted_values[i] - self.predicted_values[i - 1]) - self.yaw
            length = math.sqrt((self.x_values[i] - self.x_values[i - 1])**2 + (self.predicted_values[i] - self.predicted_values[i - 1])**2)
            #length =
            # If the flag is False, the robot should move forward
            if self.on_path:
                if theta > 0:
                    self.rotate_abs(-1 * theta)
                else:
                    self.rotate_abs(theta)

                self.move_forward(length)

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
        PID.spin()
        print("It should be done")
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
