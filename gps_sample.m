rosshutdown
close all;

rosinit();
gps_sub = rossubscriber('/GpsNavSat', @gpsCallback);

figure();
hold on;

function gpsCallback(~, msg)
    if msg.Status.Status ~= 0
        return;
    end

    long = msg.Longitude;
    lat = msg.Latitude;
    
    hold on;
    plot(long, lat);
end