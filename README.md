PART 1:
1. Navigate to where you unzipped proj3p2_Fabrizzio_Rohith.zip
2. open Visual studio code and run the file a_star_RohithVikram_Fabrizzio.py 
	a. If you want to use predefined start and goal nodes, enter 'y' when the script prompts you
	b. Otherwise, follow the prompts to enter your start and goal node. For example, when it asks you for the starting orientation, enter 30 for 30 degrees.

PART 2:
1. Move the folder /proj3p2_Fabrizzio_Rohith/part2/proj3p2_ENPM661, into your ~/catkin_ws/src
2. Run the following commands in your catkin_ws folder, all in their OWN terminal windows.
	a. roscore
	b. catkin_make
	c. roslaunch proj3p2_ENPM661 template_launch.launch
	d. rosrun proj3p2_ENPM661 publishVelocities.py (make sure its executable first)
		i. PublishVelocities.py offers a predefined start and goal node which we reccommend running. To use the predefined nodes, enter 'y' when the script asks you if you want to use the predefined nodes. 
		ii. If you opt to choose your own start and goal nodes, enter 'n' and follow the prompts. For example, when it asks you for the start x node, enter 20. If your starting node is not (10, 10, 0) then edit line 13, parameters x, y and Y from template_launch.launch before performing steps c and d.

Libraries/Dependencies:
numpy, matplotlib, cv2, queue, math, rospy, geometry_msgs, std_msgs, time

Team Members:
Fabrizzio Coronado 113100659 fcoronad
Rohith Vikram 119474198 rohithvs 

Github Link: https://github.com/f-coronado/ENPM-661-Project-3-Phase-2

Simulation Video Links:
Part1: https://youtu.be/8ahqmLnV8yU
Part2: https://youtu.be/OqlRMlshEsY
