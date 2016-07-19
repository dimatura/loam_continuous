function [ pose_arr ] = parse_gps_fix_topic( input_file )
%PARSE_GPS_TOPIC Given a file of the navsat/fix topic, parses the
%position in (lat,lng, alt). Generates another file where the parsed
%position is stored: <input_file>_converted.<input_file_extension>

     % increase the decimal point
    format long
    
    % read the text
    text = fileread(input_file);
    
    % split into sequences
    seq = strsplit(text,'---');
    
    % define the timestamp, position (lat, lng, alt)
    pose_arr = zeros(size(seq,2)-1, 4);
    
    % disregard the last sequence
    for i=1:size(seq,2)-1
        % firstly parse the status (up to 'service' string)
        [rem, status_str] = strsplit(seq{i}, 'service');
        
        status_str = strsplit(rem{1,1}, 'status: ');
        status = sscanf(status_str{1,3}, '%d%*s');
        % available gps statuses
%         STATUS_NO_FIX =  -1        % unable to fix position
%         STATUS_FIX =      0        % unaugmented fix
%         STATUS_SBAS_FIX = 1        % with satellite-based augmentation
%         STATUS_GBAS_FIX = 2        % with ground-based augmentation

        % status is valid if >= STATUS_FIX
        if status >= 0
            % check that the service is valid
            
            % rem{1,2} until latitude
            service_str = strsplit(rem{1,2}, 'latitude: ');
            service = sscanf(service_str{1,1}, '%*s%d%*s');
            % available services:
            SERVICE_GPS=1;
            SERVICE_GLONASS=2;
            SERVICE_COMPASS=4;
            SERVICE_GALILEO=8;
            if service == SERVICE_GPS
                % parse the timestamps
                secs_str = strsplit(rem{1,1}, 'secs');
                pose_arr(i,1) = sscanf(secs_str{2}, '%*s%d%*s');
                
                % parse the lat lon alt
                position_str = strsplit(rem{1,2}, 'position_covariance');
                
                position = strsplit(position_str{1,1},':');
                % lat
                pose_arr(i,2) = sscanf(position{3}, '%f%*s');
                % lng
                pose_arr(i,3) = sscanf(position{4}, '%f%*s');
                % alt
                pose_arr(i,4) = sscanf(position{5}, '%f%*s');
            end
        end        
    end
    
    
    geoshow(pose_arr(:,2), pose_arr(:,3));
    title('Ground Truth (GPS)');
    xlabel('Longitude');
    ylabel('Latitude');
    
    % generate a new filename from the original
    split_input_file = strsplit(input_file, '.'); % filename extensions
    output_file = strcat(split_input_file{1}, '_converted', '.', split_input_file{2});
    
    % save to file in the correct format for ATE/RPE
    fileID = fopen(output_file, 'w');
    fprintf(fileID,'%10d %f %f %f\n', pose_arr');
    fclose(fileID);

end

