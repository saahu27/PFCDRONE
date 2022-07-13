function [ waypoint ] = getWaypoints(  )
%GETWAYPOINTS Generates a list of waypoints for the ARDrone
%   Each waypoint is a column vector that contains the desired position of the
%   drone, desired heading angle, and waiting time. The list of waypoints
%   is created combining the column vectors. 
%   waypointsListARDrone(:,k) = [ Xe (m), Ye (m), h (m) , waiting time (sec)

% For Figure 9
nPoints = 5;
waypointsListARDrone = zeros(5,nPoints);
waypointsListARDrone(:,1) = [2.5;-1;5;0; 2] ; 
waypointsListARDrone(:,2) = [2.5;1;5;0; 2] ; 
waypointsListARDrone(:,3) = [-2.5;1;5;0; 2] ; 
waypointsListARDrone(:,4) = [-2.5;-1;5;0; 2] ; 
waypointsListARDrone(:,5) = [2.5;-1;5;0; 2] ; 

waypoint = waypointsListARDrone ; 

end

