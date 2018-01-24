% ROS implementation of doubly reactive navigation scheme on Minitaur
%
% Run 'startupROS.m' first. The script assumes an active ROS master on 
% the robot and published topics streaming IMU data, proprioceptive 
% speed estimates, distance to target and LIDAR data. A joystick is assumed 
% to be connected to the desktop computer and used to stop the behavior.
%
% Subscribes to: 
%      /minitaur/imu - IMU data stream from the robot
%      /minitaur/speed - Proprioceptive speed estimates from the robot
%      /minitaur/ranges/ranges - Distance of robot to target
%      /minitaur/scan - LIDAR feedback
%
% Publishes to:
%      /minitaur/set_cmd - Desired behavior of robot
%      /minitaur/set_twist - Desired linear and angular speed of robot
%
% Connected joystick: Press A to stop the robot
%
% Parameters tuned in lines 29-76. Press Ctrl+C to stop and save data (i.e
% structs 'parameters' and 'saved_data').
% 
% Author: Vasileios Vasilopoulos - vvasilo@seas.upenn.edu
% ======================================================================= %

function ros_doubly_reactive()
    cleanupObj = onCleanup(@cleanMeUp);
    
    % Commands
    parameters.commands.CMD_KILL = 0;
    parameters.commands.CMD_STAND = 1;
    parameters.commands.CMD_START = 2;
    parameters.commands.CMD_SET_POSITION = 3;
    parameters.commands.CMD_SET_OPEN_LOOP = 4;
    parameters.commands.CMD_SET_GAIT_BOUND = 5;
    parameters.commands.CMD_SET_GAIT_WALK = 6;   % first you have to change 'extDes' in line 27 of "Walk.cpp" in Minitaur firmware to 2.0 (maximum 2.5) and upload the code to the robot
    parameters.commands.CMD_SIGNAL0 = 10;
    parameters.commands.CMD_SIGNAL1 = 11;
    parameters.commands.CMD_SIGNAL2 = 12;
    parameters.commands.CMD_SIGNAL3 = 13;
    
    % Joystick buttons
    button.Start = 10;
    button.A = 17;
    button.B = 18;
    button.X = 19;
    button.Y = 20;
    
    % Initialize joystick
    joy = vrjoystick(1);
    
    % Collision avoidance parameters
    parameters.laser_range = 1.5;                  % Maximum value of laser data
    parameters.RobotRadius = 0.2;                  % Robot radius  

    % Control parameters and gains
    parameters.LinCtrlGain      = 1.0;
    parameters.AngCtrlGain      = 0.1;             % (0.1 default)
    parameters.ForwardLinLimit  = 0.8;
    parameters.BackwardLinLimit = 0.;              % Constrain the robot to move only forwards
    parameters.AngLimit         = 0.4;

    % Particle filter parameters
    parameters.meas_stddev  = 0.1;                 % Measurement standard deviation
    parameters.proc_stddev  = [0.2 0.4];           % Process standard deviation
    parameters.initialGuess = [5 0];               % Initial guess for the particle filter
    parameters.cov          = 4.*eye(2);           % Initial guess covariance
    parameters.timestep     = 0.05;                % Control timestep
    parameters.particles    = 2000;                % Number of particles (5000 default)
    parameters.Neff         = 0.8;                 % Number of effective particles to trigger resampling
    
    % Filtering parameters for linear speed, yaw rate and commands
    parameters.speedfilter_range = 2;
    parameters.yawdfilter_range  = 1;
    parameters.vfilter_range     = 20;
    parameters.omegafilter_range = 20;

    % ROS Publishers and messages
    variables.ROSpublishers.pubCmd = rospublisher('/minitaur/set_cmd','std_msgs/UInt8','IsLatching',true);
    variables.ROSmessages.msgCmd = rosmessage(variables.ROSpublishers.pubCmd);
    variables.ROSpublishers.pubTwist = rospublisher('/minitaur/set_twist','geometry_msgs/Twist','IsLatching',true);
    variables.ROSmessages.msgTwist = rosmessage(variables.ROSpublishers.pubTwist);


    % Send initial commands to the robot
    variables.ROSmessages.msgCmd.Data = parameters.commands.CMD_SET_GAIT_WALK;
    send(variables.ROSpublishers.pubCmd,variables.ROSmessages.msgCmd);
    pause(0.3);

    variables.ROSmessages.msgCmd.Data = parameters.commands.CMD_STAND;
    send(variables.ROSpublishers.pubCmd,variables.ROSmessages.msgCmd);
    pause(10);

    variables.ROSmessages.msgCmd.Data = parameters.commands.CMD_START;
    send(variables.ROSpublishers.pubCmd,variables.ROSmessages.msgCmd);

    % Initial variables
    variables.yaw = 0.;
    variables.pitch = 0.;
    PGL = parameters.initialGuess;
    PGA = parameters.initialGuess;
    
    variables.speed = 0.;
    variables.speedfilter_weights = ones(parameters.speedfilter_range,1)/parameters.speedfilter_range;
    variables.speedfilter = zeros(parameters.speedfilter_range,1);
    variables.yawd = 0.;
    variables.yawdfilter_weights = ones(parameters.yawdfilter_range,1)/parameters.yawdfilter_range;
    variables.yawdfilter = zeros(parameters.yawdfilter_range,1);
    variables.v = 0.;
    variables.vfilter_weights = ones(parameters.vfilter_range,1)/parameters.vfilter_range;
    variables.vfilter = zeros(parameters.vfilter_range,1);
    variables.omega = 0.;
    variables.omegafilter_weights = ones(parameters.omegafilter_range,1)/parameters.omegafilter_range;
    variables.omegafilter = zeros(parameters.omegafilter_range,1);
    
    variables.ROSmessages.msgTwist.Linear.X  = variables.v;
    variables.ROSmessages.msgTwist.Angular.Z = variables.omega;
    send(variables.ROSpublishers.pubTwist,variables.ROSmessages.msgTwist);
    pause(10);
    
    % LIDAR data initialization
    variables.lidarRanges = [];
    variables.lidarAngles = [];
    variables.lidarMinAngle = 0.;
    variables.lidarMaxAngle = 0.;
    variables.lidarResolution = 0.;

    % Particle filter initialization
    variables.PF = robotics.ParticleFilter;
    initialize(variables.PF, parameters.particles, parameters.initialGuess, parameters.cov);    % decide number of particles and initial guess
    variables.PF.StateEstimationMethod = 'mean';                                                % state estimation method ('mean' or 'maxweight')
    variables.PF.ResamplingMethod = 'systematic';                                               % resampling method
    variables.PF.StateTransitionFcn = @state_transition_function;                               % define state transition function
    variables.PF.MeasurementLikelihoodFcn = @measurement_function;                              % define measurement function
    policy = robotics.ResamplingPolicy;                                                         % define a resampling policy object
    policy.MinEffectiveParticleRatio = parameters.Neff;                                         % ratio of particles to be discarded
    variables.PF.ResamplingPolicy = policy;                                                     % update resampling policy

    % Initialize iteration index
    variables.iteration_index = 1;

    % Initial time
    t = rostime('now');
    variables.t0 = t.seconds;
    variables.t_old = variables.t0;

    % Initialize struct for saving data
    saved_data = [];

    % Subscribers
    subImu = rossubscriber('/minitaur/imu');
    subImu.NewMessageFcn = {@imuCallback,parameters};
    subSpeed = rossubscriber('/minitaur/speed');
    subSpeed.NewMessageFcn = {@speedCallback,parameters};
    subRange = rossubscriber('/minitaur/ranges/ranges');
    subRange.NewMessageFcn = {@rangeCallback,parameters};
    subLaser = rossubscriber('/minitaur/scan');
    subLaser.NewMessageFcn = {@laserCallback,parameters};

    while (1)  
        % Read the joystick
        [axes, buttons, povs] = read(joy);
        
        % If button A is pressed, stop
        if buttons(button.A)
            variables.ROSmessages.msgCmd.Data = parameters.commands.CMD_STAND;
            send(variables.ROSpublishers.pubCmd,variables.ROSmessages.msgCmd);
        end
        
        pause(0.3);
    end
    
    function predictParticles = state_transition_function(pf, prevParticles, dT, u, proc_deviation) %#ok<INUSL>
    
        v = u(1);
        w = u(2);

        l = length(prevParticles);

        % Generate velocity samples ("poison" velocity)
        sd1 = proc_deviation(1);
        sd2 = proc_deviation(2);
        vh = v + sd1*randn(l,1);  
        wh = w + sd2*randn(l,1);

        % Add a small number to prevent div/0 error
        wh(abs(wh)<1e-19) = 1e-19;

        % Convert velocity samples to pose samples
        predictParticles(:,1) = prevParticles(:,1) + (-vh+wh.*prevParticles(:,2))*dT;
        predictParticles(:,2) = prevParticles(:,2) + (-wh.*prevParticles(:,1))*dT;

    end

    function  likelihood = measurement_function(pf, predictParticles, measurement, meas_deviation) %#ok<INUSL>
        % The measurement contains all state variables
        predictMeasurement = predictParticles;

        % Calculate observed error between predicted and actual measurement
        measurementError = bsxfun(@minus, sqrt(predictMeasurement(:,1).^2+predictMeasurement(:,2).^2), measurement);
        measurementErrorNorm = sqrt(sum(measurementError.^2, 2));

        % Normal-distributed noise of measurement
        % Assuming measurements on all components have the same error distribution 
        measurementNoise = meas_deviation;

        % Convert error norms into likelihood measure. 
        % Evaluate the PDF of the multivariate normal distribution 
        likelihood = 1/(sqrt(2*pi)*measurementNoise) * exp(-0.5 * (measurementErrorNorm.^2/measurementNoise^2));

    end

    function imuCallback(src,msg,~)
        % Get IMU measurements (pitch and yaw)
        imudata  = msg;

        % Convert from quaternion to Euler
        orientation_quat = [imudata.Orientation.X imudata.Orientation.Y imudata.Orientation.Z imudata.Orientation.W];
        orientation_eul  = quat2eul(orientation_quat);

        % Update pitch and yaw angles and yaw rate
        variables.pitch = orientation_eul(2);
        variables.yaw   = -orientation_eul(3);
        variables.yawd  = -imudata.AngularVelocity.Z;
    end

    function speedCallback(src,msg,~)
        % Get linear speed measurement
        speeddata = msg;
        variables.speed = double(speeddata.Data);
    end

    function laserCallback(src,msg,~)
        % Get LIDAR data and compensate for pitch and maximum LIDAR range
        laserdata = msg;
        laserdata.Ranges = laserdata.Ranges.*cos(variables.pitch);
        range_measurements = laserdata.Ranges;
        range_measurements(range_measurements>parameters.laser_range) = parameters.laser_range;
        range_measurements(range_measurements<parameters.RobotRadius) = parameters.laser_range;
        range_measurements(isnan(range_measurements)) = parameters.laser_range;
        
        variables.lidarRanges = double(transpose(range_measurements));
        variables.lidarAngles = double(transpose(laserdata.readScanAngles));
        variables.lidarMinAngle = double(laserdata.AngleMin);
        variables.lidarMaxAngle = double(laserdata.AngleMax);
        variables.lidarResolution = double(laserdata.AngleIncrement);
    end

    function rangeCallback(src,msg,~)
        % Get range measurement
        rangedata = msg;
        range_measurement = double(rangedata.FilteredRange)/1000;

        % Low pass speed and yaw rate
        variables.yawdfilter(parameters.yawdfilter_range) = variables.yawd;
        yawdfilter_output = filter(variables.yawdfilter_weights,1,variables.yawdfilter);
        variables.yawd = yawdfilter_output(parameters.yawdfilter_range);
        
        variables.speedfilter(parameters.speedfilter_range) = variables.speed;
        speedfilter_output = filter(variables.speedfilter_weights,1,variables.speedfilter);
        variables.speed = speedfilter_output(parameters.speedfilter_range);

        % Keep track of time
        t = rostime('now');
        variables.t_new = t.seconds;
        timestep = variables.t_new-variables.t_old;

        % Particle filter updates
        [variables.statePred, variables.covPred] = predict(variables.PF, timestep, [variables.speed,variables.yawd], parameters.proc_stddev);
        [variables.stateCorrected, variables.covCorrected] = correct(variables.PF, range_measurement', parameters.meas_stddev);
        saved_data.stateEst(variables.iteration_index,:) = variables.stateCorrected(:,1:2);
        saved_data.particles(:,:,variables.iteration_index) = variables.PF.Particles(:,1:2);

        % Estimated goal position in the body frame
        Goal = [variables.stateCorrected(1) variables.stateCorrected(2)];
        
        % Pass values for finding projected goal
        if variables.pitch > deg2rad(-2.) && ~isempty(variables.lidarRanges)
            RobotState = [0 0 0];
            R = variables.lidarRanges;
            LIDAR.Angle = variables.lidarAngles;
            LIDAR.Range = parameters.laser_range;
            LIDAR.Infinity = 1e4;
            LIDAR.MinAngle = variables.lidarMinAngle;
            LIDAR.MaxAngle = variables.lidarMaxAngle;
            LIDAR.NumSample = length(variables.lidarRanges);
            LIDAR.Resolution = variables.lidarResolution;
            [PGL, PGA1, PGA2] = projgoalLIDAR2Dunicycle(RobotState, R, LIDAR, parameters.RobotRadius, Goal);
            PGA = (PGA1 + PGA2)/2;
            
            % Find v and omega: 1) construct an adaptive law for v, 2) lowpass
            % omega to avoid abrupt turns
            v_ref = PGL(1);
            variables.v = parameters.LinCtrlGain*v_ref;
            variables.vfilter(parameters.vfilter_range) = variables.v;
            vfilter_output = filter(variables.vfilter_weights,1,variables.vfilter);
            variables.v = vfilter_output(parameters.vfilter_range);
            variables.v = max(min(variables.v,parameters.ForwardLinLimit),parameters.BackwardLinLimit);

            % Turn in place if the goal is backwards (the robot can't see anything)
            omega_ref = atan2(PGA(2),PGA(1))*(v_ref>0) + sign(atan2(PGA(2),PGA(1)))*(pi/4)*(v_ref<=0);
            variables.omega = -parameters.AngCtrlGain*omega_ref;
            variables.omegafilter(parameters.omegafilter_range) = variables.omega;
            omegafilter_output = filter(variables.omegafilter_weights,1,variables.omegafilter);
            variables.omega = omegafilter_output(parameters.omegafilter_range);
            variables.omega = max(min(variables.omega,parameters.AngLimit),-parameters.AngLimit);

            % Send commands to the robot
            variables.ROSmessages.msgTwist.Linear.X  = variables.v;
            variables.ROSmessages.msgTwist.Angular.Z = variables.omega;
            send(variables.ROSpublishers.pubTwist,variables.ROSmessages.msgTwist);

            % Store data
            saved_data.time(variables.iteration_index) = variables.t_new;
            saved_data.yaw(variables.iteration_index) = variables.yaw;
            saved_data.yawd(variables.iteration_index) = variables.yawd;
            saved_data.speed(variables.iteration_index) = variables.speed;
            saved_data.range_measurements(variables.iteration_index) = range_measurement;
            saved_data.v_commanded(variables.iteration_index) = variables.v;
            saved_data.omega_commanded(variables.iteration_index) = variables.omega;
            saved_data.lidarRanges(variables.iteration_index,:) = variables.lidarRanges;
            saved_data.lidarAngles(variables.iteration_index,:) = variables.lidarAngles;
            saved_data.PGL(variables.iteration_index,:) = PGL;
            saved_data.PGA(variables.iteration_index,:) = PGA;

            % Increase iteration index
            variables.iteration_index = variables.iteration_index+1;
        end
        
        % Stop if the robot is close
        if range_measurement<0.8
            variables.ROSmessages.msgCmd.Data = parameters.commands.CMD_STAND;
            send(variables.ROSpublishers.pubCmd,variables.ROSmessages.msgCmd);
        end
        
        % Update old values
        variables.t_old = variables.t_new;
        for i=1:(parameters.yawdfilter_range-1)
            variables.yawdfilter(i) = variables.yawdfilter(i+1);
        end
        for i=1:(parameters.speedfilter_range-1)
            variables.speedfilter(i) = variables.speedfilter(i+1);
        end
        for i=1:(parameters.vfilter_range-1)
            variables.vfilter(i) = variables.vfilter(i+1);
        end
        for i=1:(parameters.omegafilter_range-1)
            variables.omegafilter(i) = variables.omegafilter(i+1);
        end
    end

    % Catch Ctrl+C
    function cleanMeUp()
        savefile = 'navigation_data.mat';
        save(savefile,'parameters','saved_data');
    end
end