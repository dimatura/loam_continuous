function draw_path_from_pose( pose, title_str, x_str, y_str, z_str )
%DRAW_PATH_FROM_POSE Draws line in 3d with the traversed path
    figure;
    clf;
    line(pose(:,1), pose(:,2), pose(:,3));
    view(180, 0);
    title(title_str);
    xlabel(x_str);
    ylabel(y_str);
    zlabel(z_str);

end

