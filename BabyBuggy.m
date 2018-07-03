classdef BabyBuggy < handle
    % BabyBuggy class holds methods relevant to rosbag parsing of BabyBuggy
      % The BabyBuggy class should be instantiated, but most of movement
      % will be directed through use of the Controller object.

    properties (Access = private)
        pf
        pf_timers
        pf_prev_times
        
        left_wheel
        right_wheel
        imu
        gps
    end

    properties (Access = public)
        fig
    end

    methods (Access = public)
        % Class constructor
        function obj = BabyBuggy(num_particles)
            % Called when a BabyBuggy object is created.
            client_ip = '192.168.9.120';
            master_ip = '192.168.9.120';
            obj.initROS(client_ip, master_ip);
            
            % Create particle filter
            initial_est = zeros(9,1);
            obj.pf = particleFilter(@babyBuggyStateTransition,@babyBuggyMeasurementLikelihood);
            initialize(obj.pf, num_particles, initial_est, eye(9));
            
            % Initialize timers (wheels, imu)
            obj.pf_timers = {};
            obj.pf_prev_times = {};
            for i = 1:2
                obj.pf_timers{i} = tic;
                obj.pf_prev_times{i} = toc(obj.pf_timers{i});
            end
            
            % Initialize sensors
            obj.left_wheel = Encoder(obj, '/encoder1_raw'); 
            obj.right_wheel = Encoder(obj, '/encoder2_raw'); 
            obj.imu = IMU(obj, '/BNO055');
            obj.gps = GPS(obj, '/AdafruitGPS');
            
            % Initialize figure
            obj.fig = figure();
        end
        
        function initROS(~, client_ip, master_ip)
            % Initialize ROS
            setenv('ROS_IP', client_ip);
            setenv('ROS_MASTER_URI', ['http://',master_ip,':11311']);
            rosinit(master_ip);
        end
        
        function updateEncoder(obj)
            if (obj.left_wheel.updated == 1) && (obj.right_wheel.update == 1)
                % update particle filter for encoders
                cur_time = toc(obj.pf_timers{1});
                dt = cur_time - obj.pf_prev_times{1};
                obj.pf_prev_times{1} = cur_time;

                vl = obj.left_wheel.wheel_vel;
                vr = obj.right_wheel.wheel_vel;
                v0 = (vl + vr)/2;

                predict(obj.pf, dt, 'encoder', v0);

                % reset updated flags
                obj.left_wheel.updated = 0;
                obj.right_wheel.updated = 0;
            end
        end
        
        function updateIMU(obj)
            ang_vel = obj.imu.ang_vel;
            predict(obj.pf, dt, 'imu', ang_vel);
        end
        
        function checkGPS(obj)
            pose = obj.gps.pose;
            
            figure(obj.fig);
            hold on;
            plot(pose(1), pose(2), '.');
            hold off;
        end
    end
end