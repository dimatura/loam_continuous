function draw_path_from_pose( pose )
%DRAW_PATH_FROM_POSE Draws line in 3d with the traversed path
    figure;
    clf;
    scatter3(pose(:,1), pose(:,2), pose(:,3),'.');
    view(180, 0);
    title('');
    xlabel('Timestamp');
    ylabel('Relative Latitude');
    zlabel('Relative Longitude');

end

