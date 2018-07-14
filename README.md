# ROS REALITY BRIDGE

Hi, and welcome to ROS Reality Bridge, a virtual reality teleoperation interface for ROS enabled robots (we use a Baxter, but the code could easily be modified for any ROS robot). This readme contains all the information you need to get started.

## Overview

This package connects a ROS network to a remote windows machine running Unity, and uses sensor data sent on ROS topics to populate a virtual reality scene. The sensors we use are a Kinect v2, the wrist cameras of the robot, and the joint encoders of the robot.

## Dependencies

For this tutorial, we will focus on controlling a Baxter robot. Therefore you will need to install the Baxter SDK on you computer. You can find instructions for that here: http://sdk.rethinkrobotics.com/wiki/Workstation_Setup

If you want to send commands back from the Unity computer, we recommend you use Ein, which sits on top of the native Baxter SDK, and gives a higher level of control, like end effector Cartesian coordinate positioning. Download and install it here: http://h2r.github.io/ein/

To connect the Kinect to the ROS network, we use IAI Kinect, a ROS package found here: https://github.com/code-iai/iai_kinect2

To calibrate the Kinect, we use this package: https://github.com/ShibataLabPrivate/kinect_baxter_calibration

Finally, you will need to install rosbridge. You can install it via apt-get ```sudo apt-get install ros-indigo-rosbridge-suite```

## Running ROS Reality

To start ROS Reality, simply run the following command on your Baxter workstation:

``roslaunch ros_reality ros_reality.launch``

Great, now this computer is sending information about the ROS network. Go to your Unity computer and follow the intstructions here: https://github.com/h2r/ROS_Unity

## ``adaptation`` branch running instruction, overview, and TODOs

Running instructions:

1.) make sure you are on the adaptation branch

2.) run the following commands after you have sourced the baxter.sh file

in ``ros_reality_bridge/src`` ``roslaunch baxter_moveit_config baxter_grippers.launch``

in ``catkin_ws`` ``rosrun baxter_interface joint_trajectory_action_server.py``

in ``ros_reality_bridge/src`` ``rosrun ros_reality_bridge moveit_interface.py`` 

in ``catkin_ws`` ``roslaunch file_server publish_description_baxter.launch``

Overview:

Code in ``moveit_interface.py`` provides ROS side code to interface with moveit. It provides callbacks to visualize trajectories and to actuate joints. See ``moveit_interface.py`` for detailed comments. 

TODOs:

1.) Maybe move away from hard coded locations. In this case we would use alvar or april tags.

2.) Move away from cartesian planner. Look into an RRT based method where a user can accept or reject the plan or into the CHOMP or STOMP planners. In this case we will have to figure out how to open and close grippers as part of the motion plans that are generated or plan each segment (from waypoint to waypoint) in a piece-wise manner. 








