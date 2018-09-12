classdef IMU < handle
    %IMU Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        rob
        imu_sub
        
        timer
        prev_time
    end
    
    properties (Access = public)
        orientation
        ang_vel
        prev_theta
    end
    
    methods
        function obj = IMU(rob, topic_name)
            %IMU Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.rob = rob;
            
            % Initialize ROS subscriber and callback
            obj.imu_sub = rossubscriber(topic_name, @imuCallback);
%             obj.imu_sub.setOnNewMessageListeners({@obj.imuCallback}); 
            
            % Initialize attributes
            obj.prev_theta = -100;
            obj.timer = tic;
            obj.prev_time = toc(obj.timer);
            
            function imuCallback(~, msg)
                %METHOD1 Summary of this method goes here
                %   Detailed explanation goes here

                x = msg.Orientation.X;
                y = msg.Orientation.Y;
                z = msg.Orientation.Z;
                w = msg.Orientation.W;
                thq = [w, x, y, z];
                thvec = quat2eul(thq,'ZYX');
                obj.orientation = thvec;

                if obj.prev_theta == -100
                    obj.prev_theta = thvec(1);
                    obj.prev_time = toc(obj.timer);
                else
                    cur_time = toc(obj.timer);
                    d_time = cur_time - obj.prev_time;
                    obj.prev_time = cur_time;

                    cur_theta = thvec(1);
                    d_theta = cur_theta - obj.prev_theta;
                    obj.prev_theta = cur_theta;

                    obj.ang_vel = d_theta / d_time;
                    obj.rob.updateIMU();
                end
            end
        end
        
        
    end
end

