DEALING WITH MOVEIT:
	Enable the robot:
		rosrun baxter_tools enable_robot.py -e

	Start the joint trajectory server to control Baxter's joints:
		rosrun baxter_interface joint_trajectory_action_server.py

	Launch the rviz moveit plugin (for visualization):
		roslaunch baxter_moveit_config baxter_grippers.launch
	
	We can do planning all programmatically, handled in:
		moveIt.py

DEALING WITH WEB SERVER:
	1) Start a roscore session:
		roscore

	2) Start a rosbridge server:
		roslaunch rosbridge_server rosbridge_websocket.launch
	
	3) Now run the node:
		python unityNode.py

	Once the websocket server is running, check out the website!
		http://cs.brown.edu/people/er35/simple.html (This is being held on Brown server in my web folder)	
