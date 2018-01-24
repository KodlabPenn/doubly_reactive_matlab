# doubly_reactive_matlab
A MATLAB-ROS implementation of a doubly-reactive, sensor-based homing algorithm for Minitaur, using a LIDAR and range-only target localization.

## Relevant publications and packages
The scripts included in this package were used in the paper:
* Vasilopoulos, V., Arslan, O., De, A., and Koditschek, D. E., "Sensor-Based Legged Robot Homing Using Range-Only Target Localization", *IEEE International Conference on Robotics and Biomimetics* (ROBIO '17), Macau, China, December 2017, pp. 2630-2637.

The doubly-reactive operations and the functions included here are based on the papers:
* Arslan, O., and Koditschek, D. E., "Exact Robot Navigation using Power Diagrams", *IEEE International Conference on Robotics and Automation* (ICRA '16), 2016, pp. 1-8.
* Arslan, O., and Koditschek, D. E., "Sensor-based Reactive Navigation in Unknown Convex Sphere Worlds", *The 12th International Workshop on the Algorithmic Foundations of Robotics* (WAFR), 2016.

Check [here](https://github.com/vvasilo/pulson_ros) for the ROS wrapper for the PulsON P440 and P410 ultra-wideband radios from Time Domain that publishes `/minitaur/ranges/ranges`.

## Preliminaries
The main script is `ros_doubly_reactive.m` and `startupROS.m` needs to be run first for initialization. 

The script assumes an *active ROS master on the robot* and published topics streaming IMU data (`/minitaur/imu`), proprioceptive speed estimates (`/minitaur/speed`), distance to target (`/minitaur/ranges/ranges`) and LIDAR data (`/minitaur/scan`). A joystick is assumed to be connected to the desktop computer and used to stop the behavior.

The node publishes to `/minitaur/set_cmd` the desired robot behavior and to `/minitaur/set_twist` the desired linear and angular speed of the robot.

## Tuning and use
The commands, joystick buttons and collision avoidance, control, particle filter and twist filtering parameters are tuned in lines 29-76 of `ros_doubly_reactive.m`. 

Press Ctrl+C to stop and save data (i.e structs `parameters` and `saved_data`).