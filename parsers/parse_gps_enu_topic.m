function [ gps_pose ] = parse_gps_enu_topic( enu_file, enu_datum_file )
%PARSE_GPS_TOPIC Given a file of the navsat/fix topic, parses the
%position in (lat,lng, alt)
    gps_pose = parse_odometry_topic(enu_file);
    
    figure;
    clf;
    line(gps_pose(:,1), gps_pose(:,2), gps_pose(:,3));
    view(90, 0);
    title('Ground Truth (GPS)');
    xlabel('Timestamp');
    ylabel('Relative Longitude');
    zlabel('Relative Latitude');
end

