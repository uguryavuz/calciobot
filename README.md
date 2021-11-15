# calciobot
CS81 Final Project, 21F.


1. Robot is running and detector is running.

2. User teleoperator should aim to help robot see targets.

3. If targets are seen, estimate locations are published

4. If user is happy with tests then user will provide input to end detector operation

5.  A* algorithm will judge best path from cube to goal.

6. Path will be followed till goal is reached

7. Robot will accelerate to get ball into goal.




#Running Code:


* python -b <file_name>


#Potential Packages that may need to be downloaded:

* sudo apt-get install python-networkx

* sudo apt-get install python-scipy

* sudo apt-get install python-tk

#Publish Message to calcio_driver

* rostopic pub calcio_driver std_msgs/String "OK" -1



#Workflow - 11/14

1.  Go near blue cube; if satisfies estimate then publish "OK" to calcio_driver
