classdef GPS < handle
    %GPS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        rob
        gps_sub
        
        initial_gps
        prev_pose
        
        phi1
        meter_per_degree
    end
    
    properties (Access = public)
        cur_gps
        pose
    end
    
    methods
        function obj = GPS(rob, topic_name)
            %GPS Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.rob = rob;
            
            % Initialize ROS subscriber and callback
            obj.gps_sub = rossubscriber(topic_name, @gpsCallback);
%             obj.gps_sub.setOnNewMessageListeners({@obj.gpsCallback}); 
            
            % Initialize attributes            
            obj.initial_gps = [0;0];
            obj.pose = [0;0;100];
            obj.phi1 = 0;
            obj.meter_per_degree = 40000000 / 360.0;
            
            function gpsCallback(~, msg)
                %METHOD1 Summary of this method goes here
                %   Detailed explanation goes here
                if msg.Status.Status ~= 0
                    return;
                end

                long = msg.Longitude;
                lat = msg.Latitude;
                
                obj.cur_gps = [long;lat];

                if obj.initial_gps(1) == 0
                    obj.initial_gps = [long; lat];
                else
                    cur_xy = obj.convertGPStoXY([long;lat]);
                    if obj.pose(3) == 100
                        obj.pose = [cur_xy; 0];
                        obj.prev_pose = obj.pose;
                    else
                        prev_xy = obj.prev_pose(1:2);

                        theta = atan2(cur_xy(2)-prev_xy(2), cur_xy(1)-prev_xy(1));
                        obj.pose = [cur_xy; theta];
                        obj.rob.updateGPS();
                    end
                end
                
                
            end
        end
        
        
        
        function pose = convertGPStoXY(obj, gps_coord)
            long = gps_coord(1);
            lat = gps_coord(2);
            
            lambda0 = obj.initial_gps(1);
            
            x = (long - lambda0) * cos(obj.phi1);
            y = lat;
            
            pose = obj.meter_per_degree * [x;y];
        end
        
        function pose = convertXYtoGPS(obj, xy_coord)
            x = xy_coord(1) / obj.meter_per_degree;
            y = xy_coord(2) / obj.meter_per_degree;
            
            long = obj.initial_gps(1) + x * sec(obj.phi1);
            lat = y;
            
            pose = [long; lat];
        end
    end
end

