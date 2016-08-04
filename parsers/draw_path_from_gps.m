function [ ] = draw_path_from_gps( pose_arr )
%DRAW_PATH_FROM_GPS Draws line in 3d with the traversed path given lat lng
    figure;
    clf;
    plot(pose_arr(:,1), pose_arr(:,2), pose_arr(:,3));
    title('Ground Truth (GPS)');
    xlabel('Longitude');
    ylabel('Latitude');

end

