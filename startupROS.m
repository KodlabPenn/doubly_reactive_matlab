% Needs to be run on startup to initialize ROS
% Use 'rosshutdown' to close the MATLAB node
% Assumes an active ROS master on the robot
% Author: Vasileios Vasilopoulos - vvasilo@seas.upenn.edu

% ROS Initialization
format compact
setenv('ROS_MASTER_URI','http://192.168.0.4:11311')
setenv('ROS_HOSTNAME','192.168.0.100')
rosinit