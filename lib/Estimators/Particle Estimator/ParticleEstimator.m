classdef ParticleEstimator < handle & Estimator

    properties (Access = protected)
        robot       % Robot using the filter
        sensor      % Input info

        pose_est    % Output info
        xhat        % Output info

        timer       % Timekeeping
        prev_time   % Timekeeping
        acc_flag    % Flag for type of beacon data used
    end

    properties (Access = public)
        pf          % List of particle filters
        dist        % Measured Distance from Beacon(s)

        likelihood  % Debugging Info
        distance    % Debugging Info
        covariance  % Debugging Info
    end

    methods
        function obj = ParticleEstimator(num_particles, sensor, acc_flag)
            obj.pose_est = [0;0];
            obj.sensor = sensor;
            obj.timer = tic;
            
            obj.acc_flag = acc_flag;

            obj.pf = particleFilter(@TurtleBotStateTransition,@TurtleBotMeasurementLikelihood);
            obj.pf.initialize(num_particles,[-7 7;-7 7]);
            obj.pf.ResamplingMethod = 'multinomial'; % multinomial/residual/stratified/systematic
            obj.pf.StateEstimationMethod = 'mean'; % mean/maxWeight
            obj.pf.ResamplingPolicy.TriggerMethod = 'interval'; % ratio/interval

            obj.pf.ResamplingPolicy.SamplingInterval = 30;
            obj.pf.ResamplingPolicy.MinEffectiveParticleRatio = .9;

            t = rostime('now');
            obj.prev_time = double(t.Sec)+double(t.Nsec)*10^-9;
        end
        
        function init(obj, robot)
            obj.robot = robot;
        end
        
        function xhat = getEstimate(obj, varargin)
            % Estimates the position of specified beacon
            
            if (strcmp(obj.acc_flag, 'acc'))
                obj.distance = obj.sensor.accRead();
            else
                obj.distance = obj.sensor.read(); 
            end

            prev_u = obj.robot.getPrevU('particle');

            t = rostime('now');
            cur_time = double(t.Sec)+double(t.Nsec)*10^-9;
            dt = cur_time - obj.prev_time;
            obj.prev_time = cur_time;

            obj.xhat = obj.pf.predict(prev_u, dt);
            obj.xhat = obj.pf.correct(obj.distance);
            xhat = obj.xhat;
        end

        function setPolicy(obj, policy)
            % Sets the state estimation method of a sensor's particle
            % filter to either
            %   - 'mean'
            %   - 'maxWeight'
            
            obj.pf.setEstimationMethod = policy;
        end

        function dist = getDist(obj)
           % Returns the estimated distance from a robot to the beacon
           % specified by the sensor parameter
           dist = obj.distance;
        end

        function bpose = getRealBeaconPoseInRobotFrame(obj)
            % Transforms the specified beacon from vicon frame to robot
            % odometry frame
            pose = obj.robot.getPose();
            theta = pose(3);
            bpose = obj.sensor.getRealPose();
            bpose = bpose';
            rtw = [cos(theta) -sin(theta) pose(1);
                    sin(theta) cos(theta) pose(2);
                    0 0 1];
            bpose(3) = 1;
            bpose = inv(rtw)*bpose;
            bpose(3) = [];
        end

        function plotPoints(obj)
            % Plots all estimates as well as their point clouds in the
            % robot frame
            figure(obj.robot.fig)
            cmap = hsv(1);

            sensor = 1;
            plot(obj.pf.Particles(1,:),obj.pf.Particles(2,:),'.','Color',cmap(sensor,:),'DisplayName',['Beacon ', num2str(sensor)]);
            n = 10;
            xlim([-n n]);
            ylim([-n n]);
            hold on;
            xhatt = obj.xhat;
            plot(xhatt(1),xhatt(2),'b*');%,'Color',cmap(sensor,:),'DisplayName',['Beacon ', num2str(sensor), ' estimated position']);
            text(xhatt(1),xhatt(2),['Beacon ',num2str(sensor)]);
            beacpose = obj.getRealBeaconPoseInRobotFrame();
            [x,y] = obj.circleGen(beacpose(1),beacpose(2),0.35);
            plot(beacpose(1),beacpose(2),'k*','DisplayName',['Beacon', num2str(sensor), ' actual position']);
            plot(x,y,'b--');
            
            hold off;
            refresh
        end

        function [x1,y1] = circleGen(~,x,y,r)
            thet = 0:1/50:2*pi;
            x1 = r*cos(thet) + x;
            y1 = r*sin(thet) + y;
        end

        function pose = getGroundTruth(obj)
            % Returns robot pose in Vicon frame
            pose = obj.robot.getPose();
        end
    end
end
