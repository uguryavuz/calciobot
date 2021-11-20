# calciobot
CS81 Final Project, 21F.

Written by Ugur Yavuz, Qiyao Zuo, Jeff Cho and Viney Regunath.

# How to launch
* Packages to be installed: ``python-networkx`` and ``python-scipy``. 
* Change directory to ``gazebo-testing``. 
* Run ``source ./devel/setup.sh`` then ``roslaunch rosbot_description rosbot_rviz_gmapping.launch``.
* In another terminal, run ``roslaunch rosbot_navigation rosbot_teleop.launch`` (also after sourcing).
* In another terminal, switch to the main directory.
* Run ``python -B driver.py``. (The ``-B`` flag prevents the imported scripts from being compiled into .pyc files.)
* Open another terminal, where you will later publish String messages to the ``calcio_driver`` topic (e.g. ``rostopic pub calcio_driver std_msgs/String "OK" -1``).

# Workflow
* You might want to open a VNC tab in a browser to observe rviz and Gazebo.
* Teleoperate the robot as the robot is detecting the blue cube. Once you are happy with the published estimates, send any non-empty string message to ``calcio_driver``. This will register the estimate as the location for the cube.
* Teleoperate the robot as the robot is detecting the yellow/orange goal. Once you are happy with the published estimates, send any non-empty string message to ``calcio_driver``.
  * Owing to the shape of the goal, sometimes you might get an information message saying that this is not an empty cell and therefore won't be registered. Getting it right might take a number of attempts. Usually, looking directly at the goal will produce a healthy estimate that can be registered.
* Once both estimates are registered, the driver will compute a path from the target object to the goal, and a second path from its current location to a point slightly behind the starting point of the first path. These paths are both published as marker arrays that can be seen in rviz.
* The driver will then make the robot follow both paths -- first the latter, then the former; and hopefully will end up pushing the target object into the goal! 

# Media
Here are some of the driver in action:

<img width="480" alt="img1" src="https://github.com/uguryavuz/calciobot/raw/070150408c832bf535db384a57dc44032d5a3690/media/img1.png">
<img width="480" alt="img2" src="https://github.com/uguryavuz/calciobot/raw/070150408c832bf535db384a57dc44032d5a3690/media/img2.png">
<img width="480" alt="img3" src="https://github.com/uguryavuz/calciobot/raw/070150408c832bf535db384a57dc44032d5a3690/media/img4.png">
<img width="480" alt="img4" src="https://github.com/uguryavuz/calciobot/raw/070150408c832bf535db384a57dc44032d5a3690/media/img3.png">
