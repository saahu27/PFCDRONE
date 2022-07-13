function [ obstacle ] = getObstaclepoints()
    %GETWAYPOINTS Generates a list of obstacle points for the ARDrone
    %   Each obstacle point is a column vector that contains the obstacle's X & Y positions and the obstacles X & Y velocities. 
    %   The list of obstacles is created by combining the column vectors. 



    % Edit the following entries for k =1,2,...,nObsPoints
    % obstaclesListARDrone(:,k) = [ x_o (m); y_o (m); vx_o (m/s); vy_o (m/s)]
    
    % For Figure 9
    nObsPoints = 1;
    obstaclesListARDrone = zeros(2,nObsPoints);
    obstaclesListARDrone(:,1) = [1.0; 1.0] ;
 
    obstacle = obstaclesListARDrone;

end

