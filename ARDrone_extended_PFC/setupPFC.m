%% =======================================================================
%  ENPM667 Project - 01: Extended Potential Field Controller 
%  =======================================================================
%  
%  This script is based on the setupWPTrackingSim.m script in the toolbox
%  
%  Adarsh Malapaka
%  Sahruday Patti
%  =======================================================================

%%
%  Cleaning workspace
bdclose all;
clear all;
clc

%%
% Adding ARDrone library path 
addpath ../; 
%% Simulation parameters

% Flight management system sample time. This is the sample time at which
% the control law is executed. 
FMS.Ts = 0.065; 

% Time delay due to communication between drone and host computer
timeDelay = FMS.Ts*4; 


%% Vehicle model based on linear dynamics

% Loading state space representation of vehicle dynamics
setupARModel; 

%%
% Loading list of waypoints
waypoints = getWaypoints() ;

% Loading list of obstacles
obstacles = getObstaclepoints() ;
nObst = size(obstacles,2);

%% 
% Simulation time
simDT = 0.005 ;

%%
% Loading Simulink model of Extended Potential Field 
% controller for AR Drone 2.0
potential_field_controller ;



