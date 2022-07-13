README

%%%% Approved Paper %%%%%
 
Title: A Novel Potential Field Controller for Use on Aerial Robots
Authors: Alexander C. Woods and Hung M. La, Senior Member, IEEE

%%%% TO RUN ROS + GAZEBO %%%%%

Package Name: sjtu-drone
Launch file: launch/ardrone_waypoint.launch
Script file: src/potential_field_control.py


Add the given package folder (sjtu-drone) in your catkin_ws/src.


1. Open a terminal and navigate to cd catkin_ws. Run catkin_make.
2. Run "roslaunch sjtu_drone ardrone_waypoint.launch" to launch the robot in the custom world. If 	it doesn't work, you may run "roslaunch sjtu_drone simple.launch"
3. In another terminal, run:  "rosrun sjtu_drone potential_field_control.py"

NOTE: Uncomment the corresponding lines (17-34) in he python script depending on which Figure ofnthe original paper you wish to plot.

For example: For Fig 14, uncomment lines 25,26,98,140,160 and comment lines 90,91,134,168,173,174. Roslaunch the launch file and run the conroller python script as mentioned before. 



%%%% TO RUN MATLAB/SIMULINK %%%%%

Folder Name: ARDrone_extended_PFC
Simulink File: potential_field_controller.slx
Script to set Waypoints: getWaypoints.m
Script to set Obstacles: getObstaclepoints.m
Script to load variables & start simulation: setupPFC.m


To run the Simulink program and model,

1. Open MATLAB and change the present directory to inside "ARDrone_extended_PFC".
2. Open getWaypoints.m and add the details about the desired target points.
3. Open getObstaclepoints.m and add the X-Y coordinate details of the obstacle(s).
4. Run setupPFC.m, this should open the "potential_field_controller.slx" SIMULINK file.
5. Click the green run button on the top to run the simulation, this should open up an X-Y graph
   of the drone's positions.



If any issues in opening/running/accessing the files, please reach out to amalapak@umd.edu. 

