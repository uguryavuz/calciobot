To run the gazebo simulation:
* Execute source ``./devel/setup.sh`` in two terminals.
* Run ``roslaunch rosbot_description rosbot_rviz_gmapping.launch`` in one terminal. This should launch Gazebo in VNC.
    * Note that this also runs gmapping, which publishes an occupancy grid to map. How useful!
* Run ``roslaunch rosbot_navigation rosbot_teleop.launch`` in a second terminal. This should allow you to control the robot.