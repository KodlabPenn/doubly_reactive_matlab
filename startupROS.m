% MIT License (modified)

% Copyright (c) 2018 The Trustees of the University of Pennsylvania
% Authors:
% Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>

% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this **file** (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.


% Needs to be run on startup to initialize ROS
% Use 'rosshutdown' to close the MATLAB node
% Assumes an active ROS master on the robot

% ROS Initialization
format compact
setenv('ROS_MASTER_URI','http://192.168.0.4:11311')
setenv('ROS_HOSTNAME','192.168.0.100')
rosinit