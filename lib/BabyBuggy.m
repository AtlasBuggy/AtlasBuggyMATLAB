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
        
        d_left
        d_right
        
        vel
        theta
        state
        
        pos_arr
        lat_arr
        gps_arr
    end

    properties (Access = public)
        fig
    end

    methods (Access = public)
        % Class constructor
        function obj = BabyBuggy()
            % Called when a BabyBuggy object is created.
            rosinit();
            
            % Initialize timers (wheels, imu)
            obj.pf_timers = {};
            obj.pf_prev_times = {};
            for i = 1:2
                obj.pf_timers{i} = tic;
                obj.pf_prev_times{i} = toc(obj.pf_timers{i});
            end

            % Initialize attributes
            obj.vel = 0;
            obj.state = [0;0;0];
            obj.pos_arr = [];
            obj.lat_arr = [];
            obj.gps_arr = [];
            
            obj.d_left = 0;
            obj.d_right = 0;
            
            % Initialize sensors
            obj.left_wheel = Encoder(obj, '/encoder1_raw'); 
            obj.right_wheel = Encoder(obj, '/encoder2_raw'); 
            obj.imu = IMU(obj, '/BNO055');
%             obj.gps = GPS(obj, '/GpsNavSat');
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
            if (obj.left_wheel.updated == 1) && (obj.right_wheel.updated == 1)
                cur_time = toc(obj.pf_timers{1});
                dt = cur_time - obj.pf_prev_times{1};
                obj.pf_prev_times{1} = cur_time;

                vl = obj.left_wheel.wheel_vel;
                vr = obj.right_wheel.wheel_vel;
                obj.vel = (vl + vr)/2;

                obj.d_left = obj.d_left + obj.left_wheel.d_dist;
                obj.d_right = obj.d_right + obj.right_wheel.d_dist;
                
                % reset updated flags
                obj.left_wheel.updated = 0;
                obj.right_wheel.updated = 0;
            end
        end
        
        function updateIMU(obj)
            cur_time = toc(obj.pf_timers{2});
            dt = cur_time - obj.pf_prev_times{2};
            obj.pf_prev_times{2} = cur_time;
            
            x = obj.state(1);
            y = obj.state(2);
            th = obj.imu.prev_theta;
            
            d_dist = (obj.d_left + obj.d_right)/2;
            
            x = x + d_dist * cos(th);
            y = y + d_dist * sin(th);
            
            obj.d_left = 0;
            obj.d_right = 0;
            
            obj.state = [x; y; th];
            obj.pos_arr = [obj.pos_arr; x y];
            
            figure(obj.fig);
            subplot(1,2,1);
            title('Dead Reckoning');
            
            plot(obj.pos_arr(:,1), obj.pos_arr(:,2));
        end
        
        function updateGPS(obj)
            long_lat = obj.gps.cur_gps;
            gps_pose = obj.gps.pose;
            
            obj.lat_arr = [obj.lat_arr; long_lat(1) long_lat(2)];
            obj.gps_arr = [obj.gps_arr; gps_pose(1) gps_pose(2)];
            
            figure(obj.fig);
            subplot(1,2,2);
            title('Pure GPS');
            
            plot(obj.lat_arr(:,1), obj.lat_arr(:,2));
%             plot(obj.gps_arr(:,1), obj.gps_arr(:,2));
        end
    end
end