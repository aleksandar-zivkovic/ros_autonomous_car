# EB Hobby Car #
EB Hobby Car is Elektrobit's Hobby Car. It is developing using custom HW and ROS.

## Installing catkin workspace ##
Navigate to /catkin_ws folder and enter following command:
	catkin_make install
After the intallation is successfully completed, enter the following command:
	source devel/setup.bash
* Note: This should be done every time new terminal is opened.
After this there are two ways to start the ROS nodes:
1) Using launch files:
	Navigate to catkin_ws/launch directory and enter the following command:
		roslaunch eb_imu.launch
2) Running each node independently:
	Navigate to catkin_Ws directory and launch the nodes using the following syntax:
		roslaunch <packacge_name> <node_name>
		ex. roslaunch car_interface sensor_interface.py



