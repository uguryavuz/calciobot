#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: TODO: Jeffrey Cho & Viney Regunath
# Date: TODO: 10/13/21

# Import of python modules.
import math # use of pi.
from tf import transformations
import random
from final_path_finder import RandomWalk, Grid

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan'
DEFAULT_ODOM_TOPIC = 'odom'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# START =
# END =

# delta time
dt = 0.1 # seconds

# Velocities that will be used (feel free to tune)
MAX_LINEAR_VELOCITY = 1 # m/s
#ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
# MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max
DESIRED_DISTANCE = 0.5 # m what we will use to calculate error
velocity_increment = 0.5

# Field of view in radians that is checked in front of the robot (feel free to tune)
#MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi;
#MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi;
#MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi;
#MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi;

MIN_SCAN_ANGLE_RAD = -1 * math.pi;
MAX_SCAN_ANGLE_RAD = + math.pi;

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
        # Setting up subscriber receiving messages from the laser.
        #self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        self._odom = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self._odom_callback, queue_size=1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = 0 # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle
        self.distance = distance

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

    def move_forward(self, length):
        rate = rospy.Rate(FREQUENCY)
        start_time = rospy.get_rostime()
        duration = length/MAX_LINEAR_VELOCITY

        while rospy.get_rostime() - start_time <= rospy.Duration(duration):
             #print("jeff")
             self.move(MAX_LINEAR_VELOCITY, 0)
             rate.sleep()

    def draw_round(self, radius, angular_velocity):
        rate = rospy.Rate(FREQUENCY)
        start_time = rospy.get_rostime()

        velocity = MAX_LINEAR_VELOCITY - velocity_increment
        duration = angular_velocity * radius

        # uses the ANGULAR_VELOCITY to help overcome offset from simlulation
        #duration = math.pi/angular_velocity + 0.5 * angular_velocity

        while rospy.get_rostime() - start_time <= rospy.Duration(duration):

             # moving both linear and angular velocity
             self.move(velocity, -1 * angular_velocity)
             rate.sleep()

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
        self.yaw = euler[2]

        # printing transformation matrix
        print("The euler", euler)


def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("CalcioBot")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    PID = Pid()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(PID.stop)

    # Robot random walks.
    try:

        # Initialization of the class for the random walk.
        random_walk = RandomWalk()
        # Sleep for a few seconds to wait for the registration.
        rospy.sleep(4)
        global dimension
        # Robot random walks.
        path=random_walk.map.path_search((80, 63),(60, 20))
        random_walk.show_path(path)


        #path = Grid.path_search(START, END)
        length = len(path)
        for i in range(1, length):
        #while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C

            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random amount of time
            # after which the flag is set again to False.
            # Use the function move already implemented, passing the default velocities saved in the corresponding class members.

            ####### TODO: ANSWER CODE BEGIN #######

            if i != length-1:

                print("Follow path")
                # If the flag is False, the robot should move forward
                #robot moves if obstacle not detected

                if not Pid._close_obstacle:
                    if path[i - 1][1] == path[i][1] or path[i - 1][0] == path[i][0]:
                        distance = math.sqrt((path[i - 1][1] - path[i][1])**2 + (path[i - 1][0] - path[i][0])**2)
                        Pid.move_forward(distance)

                    else:
                        y = path[i][1] - path[i - 1][1]
                        x = path[i][0] - path[i - 1][0]
                        angular_vel = math.atan2(y, x)
                        distance = math.sqrt((path[i - 1][1] - path[i][1])**2 + (path[i - 1][0] - path[i][0])**2)
                        radius = distance // 2
                        Pid.draw_round(radius, angular_vel)
                #robot will attempt to move away from obstacle - obstacle avoidance
                else:

                    # stop
                    Pid.stop()

                    x = Pid.orientation.x
                    y = Pid.orientation.y

                    if y == path[i][1] or x == path[i][0]:
                        distance = math.sqrt((y - path[i][1])**2 + (x - path[i][0])**2)
                        Pid.move_forward(distance)

                    else:
                        y1 = path[i][1] - y
                        x1 = path[i][0] - x
                        angular_vel = math.atan2(y1, x1)
                        distance = math.sqrt((y - path[i][1])**2 + (x - path[i][0])**2)
                        radius = distance // 2
                        Pid.draw_round(radius, angular_vel)

                    Pid._close_obstacle = False
            else:
                print("Goal nearby! Pushing Operation needs to be developed here!")


    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
