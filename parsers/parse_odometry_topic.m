function [ pose_arr ] = parse_odometry_topic( input_file )
%parse_odometry_topic Given a file of the odometry topic, parses the
%position in (x,y,z) and orientation in quaternions (x,y,z,w)
    
    % increase the decimal point
    format long
    
    % read the text
    text = fileread(input_file);
    
    % split into sequences
    seq = strsplit(text,'---');
    
    % define the timestamp, position (x,y,z) and orienatation in quat (x,y,z,w)
    pose_arr = zeros(size(seq,2)-1, 8);
    % disregard the last sequence
    for i=1:size(seq,2)-1
        % split before and after pose
        [rem, pose_str] = strsplit(seq{i}, 'pose');
        % get the timestamp (rem{1})
        secs_str = strsplit(rem{1}, 'secs');
        secs = sscanf(secs_str{2}, '%*s%d%*s');
        nsecs = sscanf(secs_str{3}, '%*s%d%*s');
        pose_arr(i,1) = secs+nsecs*10^-9;
        % get the position and orientation(rem{3})
        [pose, rem2] = strsplit(rem{3}, 'covariance');
        % pose now looks like so:
        %:      position:        x: 0.0405009053648       y: -0.0167003273964       z: 0.00216210098006     orientation:        x: 0.147895129587       y: -0.68759950882       z: 0.695003643483       w: 0.149344841427   
        all_values_arr = strsplit(pose{1}, '      ');
        for k=2:size(all_values_arr,2)
            pose_arr(i,k) = sscanf(all_values_arr{1,k}, '%*s %f');
        end
    end
    
    % generate a new filename
    split_input_file = strsplit(input_file, '.'); % filename extensions
    output_file = strcat(split_input_file{1}, '_converted', '.', split_input_file{2});
    
    % save to file in the correct format for ATE/RPE
    fileID = fopen(output_file, 'w');
    fprintf(fileID,'%f %f %f %f %f %f %f %f\n', pose_arr');
    fclose(fileID);

end

