#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: TODO: Viney Regunath
# Date: TODO: 10/13/21

# Import of python modules.
import math # use of pi.
import random
from final_path_finder import RandomWalk, Grid
from geometry_msgs.msg import Point
from math import atan2

#import tf
import tf
from tf.transformations import euler_from_quaternion
# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist, Quaternion # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan'
DEFAULT_ODOM_TOPIC = 'odom'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# delta time
dt = 0.1 # seconds

# Velocities that will be used (feel free to tune)
MAX_LINEAR_VELOCITY = 1.0 # m/s
#ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
# MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
DESIRED_DISTANCE = 0.5 # m what we will use to calculate error
velocity_increment = 0.5



MIN_SCAN_ANGLE_RAD = -1 * math.pi/4
MAX_SCAN_ANGLE_RAD = math.pi/4

KP = 1

KD = 38

class Pid():
    def __init__(self, linear_velocity=MAX_LINEAR_VELOCITY, delt_time = dt, distance = DESIRED_DISTANCE, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD]):
        """Constructor."""
        self.dt = delt_time
        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Setting up subscriber receiving messages from the odom.
        self._odom = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = 0 # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle
        self.distance = distance
        #self._close_obstacle = False
        self.errors = []
        #coordinates of robot
        global x_c
        global y_c

        # Flag used to control the behavior of the robot.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)


    #helper function to make robot move to a specific point
    def gotogoal(self,x,y):

        global theta
        global x_c
        global y_c
        r = rospy.Rate(1)
        current_dist = self.calculate_distance((x,y),(x_c,y_c))


        while(current_dist>0.1):
            inc_x = x - x_c
            inc_y = y - y_c
            angle_to_goal = atan2(inc_y,inc_x)
            #print("the angle to goal is", angle_to_goal,"and current theta is", theta, "the difference betweent the two is", abs(angle_to_goal - theta))
            if abs(angle_to_goal - theta)>0.5 :
                #change rotation depending on positivity or negativity of angle_to_goal
                if(angle_to_goal<0):
                    #print('clockwise')
                    a = -0.5
                else:
                    #print("counter clockwise")
                    a = 0.5
                self.move(0.0,a)
            elif abs(angle_to_goal - theta)>0.1:
                if(angle_to_goal<0):
                    #print('clockwise tiny')
                    a = -0.1
                else:
                    #print("counter clockwise tiny")
                    a = 0.1
                self.move(0.0,a)
            else:
                self.move(0.3,0.0)
            current_dist= self.calculate_distance((x,y),(x_c,y_c))
            r.sleep()
        print("Current final position is", x,y)

    #function to calculate distance between points
    def calculate_distance(self, new_position, old_position):
        """Calculate the distance between two Points (positions)."""
        x2 = new_position[0]
        x1 = old_position[0]
        y2 = new_position[1]
        y1 = old_position[1]
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist
    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        #print("move",linear_vel,angular_vel)
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

	#helper function to get inital position of robot
    def get_init_position(self):
        """Get the initial position of the robot."""
        global x_c,y_c

        data_odom = None
        # wait for a message from the odometry topic and store it in data_odom when available
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init pose")

        # Store the received odometry "position" variable in a Point instance
        self._current_position = Point()
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z
        x_c = data_odom.pose.pose.position.x
        y_c =self._current_position.y

        return((data_odom.pose.pose.position.x,data_odom.pose.pose.position.y))
    #callback function
    def _odom_callback(self,msg):
        global roll,pitch,theta
        global x_c,y_c
        #get position from message
        orient_q= msg.pose.pose.orientation
        orient_list = [orient_q.x,orient_q.y,orient_q.z]
        NewPosition = msg.pose.pose.orientation
        #self.get_init_position()
        #print("Current position",NewPosition)
        #update global variables
        x_c = msg.pose.pose.position.x
        y_c = msg.pose.pose.position.y
        global roll, pitch, theta
        (roll,pitch,theta) = euler_from_quaternion([orient_q.x,orient_q.y,orient_q.z,orient_q.w])





def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("CalcioBot",log_level=rospy.DEBUG)

    # Sleep for a few seconds to wait for the registration.
    #rospy.sleep(2)

    # Initialization of the class for the random walk.
    PID = Pid()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(PID.stop)

    # Robot random walks.
    try:
        # Initialization of the class for the random walk.
        random_walk = RandomWalk()
        # Sleep for a few seconds to wait for the registration.
        #rospy.sleep(4)
        global dimension
        # Robot random walks.

        path = [(-1.9999999552965164, 2.0000001043081284), (-1.8999999538064003, 2.0000001043081284), (-1.7999999523162842, 2.0000001043081284), (-1.6999999508261681, 2.0000001043081284), (-1.5999999493360519, 2.0000001043081284), (-1.4999999478459358, 2.0000001043081284), (-1.4999999478459358, 1.9000001028180122), (-1.4999999478459358, 1.8000001013278961), (-1.4999999478459358, 1.70000009983778), (-1.4999999478459358, 1.6000000983476639), (-1.4999999478459358, 1.5000000968575478), (-1.4999999478459358, 1.4000000953674316), (-1.4999999478459358, 1.3000000938773155), (-1.4999999478459358, 1.2000000923871994), (-1.4999999478459358, 1.1000000908970833), (-1.4999999478459358, 1.0000000894069672), (-1.4999999478459358, 0.90000008791685104), (-1.4999999478459358, 0.80000008642673492), (-1.4999999478459358, 0.7000000849366188), (-1.4999999478459358, 0.60000008344650269), (-1.4999999478459358, 0.50000008195638657), (-1.4999999478459358, 0.40000008046627045), (-1.4999999478459358, 0.30000007897615433), (-1.4999999478459358, 0.20000007748603821), (-1.4999999478459358, 0.10000007599592209), (-1.4999999478459358, 7.4505805969238281e-08), (-1.4999999478459358, -0.09999992698431015), (-1.4999999478459358, -0.19999992847442627), (-1.4999999478459358, -0.29999992996454239), (-1.4999999478459358, -0.39999993145465851), (-1.4999999478459358, -0.49999993294477463), (-1.4999999478459358, -0.59999993443489075), (-1.4999999478459358, -0.69999993592500687), (-1.4999999478459358, -0.79999993741512299), (-1.4999999478459358, -0.89999993890523911), (-1.4999999478459358, -0.99999994039535522), (-1.4999999478459358, -1.0999999418854713), (-1.4999999478459358, -1.1999999433755875), (-1.4999999478459358, -1.2999999448657036), (-1.3999999463558197, -1.2999999448657036), (-1.2999999448657036, -1.2999999448657036), (-1.1999999433755875, -1.2999999448657036), (-1.0999999418854713, -1.2999999448657036), (-0.99999994039535522, -1.2999999448657036), (-0.89999993890523911, -1.2999999448657036), (-0.79999993741512299, -1.2999999448657036), (-0.69999993592500687, -1.2999999448657036), (-0.59999993443489075, -1.2999999448657036), (-0.49999993294477463, -1.2999999448657036), (-0.39999993145465851, -1.2999999448657036), (-0.29999992996454239, -1.2999999448657036), (-0.29999992996454239, -1.1999999433755875), (-0.29999992996454239, -1.0999999418854713), (-0.29999992996454239, -0.99999994039535522), (-0.29999992996454239, -0.89999993890523911), (-0.29999992996454239, -0.79999993741512299), (-0.29999992996454239, -0.69999993592500687), (-0.29999992996454239, -0.59999993443489075), (-0.29999992996454239, -0.49999993294477463), (-0.29999992996454239, -0.39999993145465851), (-0.29999992996454239, -0.29999992996454239), (-0.29999992996454239, -0.19999992847442627), (-0.29999992996454239, -0.09999992698431015), (-0.29999992996454239, 7.4505805969238281e-08), (-0.29999992996454239, 0.10000007599592209), (-0.29999992996454239, 0.20000007748603821), (-0.29999992996454239, 0.30000007897615433), (-0.29999992996454239, 0.40000008046627045), (-0.29999992996454239, 0.50000008195638657), (-0.29999992996454239, 0.60000008344650269), (-0.29999992996454239, 0.7000000849366188), (-0.19999992847442627, 0.7000000849366188), (-0.19999992847442627, 0.80000008642673492), (-0.09999992698431015, 0.80000008642673492), (-0.09999992698431015, 0.90000008791685104), (7.4505805969238281e-08, 0.90000008791685104), (7.4505805969238281e-08, 1.0000000894069672), (7.4505805969238281e-08, 1.1000000908970833), (0.10000007599592209, 1.1000000908970833), (0.20000007748603821, 1.1000000908970833), (0.20000007748603821, 1.2000000923871994), (0.30000007897615433, 1.2000000923871994), (0.30000007897615433, 1.3000000938773155), (0.40000008046627045, 1.3000000938773155), (0.40000008046627045, 1.4000000953674316), (0.40000008046627045, 1.5000000968575478), (0.50000008195638657, 1.5000000968575478), (0.60000008344650269, 1.5000000968575478), (0.60000008344650269, 1.6000000983476639), (0.7000000849366188, 1.6000000983476639), (0.7000000849366188, 1.70000009983778), (0.80000008642673492, 1.70000009983778), (0.80000008642673492, 1.8000001013278961), (0.80000008642673492, 1.9000001028180122), (0.90000008791685104, 1.9000001028180122), (1.0000000894069672, 1.9000001028180122), (1.0000000894069672, 2.0000001043081284), (1.1000000908970833, 2.0000001043081284), (1.1000000908970833, 2.1000001057982445), (1.2000000923871994, 2.1000001057982445), (1.2000000923871994, 2.2000001072883606), (1.2000000923871994, 2.3000001087784767), (1.3000000938773155, 2.3000001087784767), (1.4000000953674316, 2.3000001087784767), (1.4000000953674316, 2.4000001102685928), (1.5000000968575478, 2.4000001102685928), (1.5000000968575478, 2.500000111758709), (1.6000000983476639, 2.500000111758709), (1.6000000983476639, 2.6000001132488251), (1.6000000983476639, 2.7000001147389412), (1.70000009983778, 2.7000001147389412), (1.8000001013278961, 2.7000001147389412), (1.8000001013278961, 2.8000001162290573), (1.9000001028180122, 2.8000001162290573), (1.9000001028180122, 2.9000001177191734), (2.0000001043081284, 2.9000001177191734), (2.0000001043081284, 3.0000001192092896)]




        length = len(path)

        for i in range(1, length):

            ####### TODO: ANSWER CODE BEGIN #######

            if i != length-1:

                #print("Follow path")
                # If the flag is False, the robot should move forward
                #robot moves if obstacle not detected
                x= path[i][0]
                y =path[i][1]
                print("Current position",PID.get_init_position())

                PID.gotogoal(x,y)
                print("Coordinates desired are",x,y)
            else:
                x= path[i][0]
                y =path[i][1]
                print("Current position",PID.get_init_position())

                PID.gotogoal(x,y)
                print("End point reached")


    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
