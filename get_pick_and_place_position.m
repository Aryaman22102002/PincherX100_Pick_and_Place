function [pick_pose_base, drop_pose_base, obstacle_pose_base] = get_pick_and_place_position()

    % Create a webcam object
    cam = webcam("Logi C270 HD WebCam");

    % Camera Intrinsics
    focalLength = [1551.7368, 1552.1575];  
    principalPoint = [622.0061, 373.7144];
    radialDistortion = [0.1952, -0.5791];
    imageSize = [480, 640];

    % Create camera parameters object
    cameraParams = cameraIntrinsics(focalLength, principalPoint, imageSize, ...
        'RadialDistortion', radialDistortion);

    % Define tag size
    tagSize = 0.034;  

    % Marker IDs
    baseMarkerID = 0;   % Base marker ID
    pickMarkerID = 0;   % Pick position marker ID
    dropMarkerID = 1;   % Drop position marker ID
    obstacleMarkerID = 2; % Obstacle marker ID

    % Step 1: Detect Base Marker and Store Transformation
    disp('Place the base marker (ID 0) in view and press Enter...');
    pause;

    T_base_cam_stored = [-0.0333,	-0.9972,	-0.0671,	-0.2039;
                         0.9994,	-0.0328,	-0.0083,	-0.1283;
                         0.0061,	-0.0673,	0.9977,	     0.9950;
                         0,       0,       0,       1.0000];

    % while isempty(T_base_cam_stored)
    %     I = snapshot(cam);
    %     [ids, locs, poses] = readAprilTag(I, "tag36h11", cameraParams, tagSize);
    % 
    %     if any(ids == baseMarkerID)
    %         baseIdx = find(ids == baseMarkerID);
    %         basePose = poses(baseIdx);
    %         T_base_cam_stored = [basePose.R, basePose.Translation'; 0, 0, 0, 1];
    %         disp('Base marker detected and transformation stored.');
    %         disp(T_base_cam_stored);
    %     else
    %         disp('Base marker not detected. Adjust and press Enter.');
    %         pause;
    %     end
    % end
    % Step 2: Detect Pick, Drop, and Obstacle Markers
    disp('Ensure pick (ID 1) and drop (ID 2) markers are in view. Press Enter...');
    pause;

    pick_pose_base = [];
    drop_pose_base = [];
    obstacle_pose_base = []; % Initialize as empty for cases without an obstacle

    while isempty(pick_pose_base) || isempty(drop_pose_base)
        I = snapshot(cam);
        [ids, locs, poses] = readAprilTag(I, "tag36h11", cameraParams, tagSize);

        % Check for pick marker (ID 1)
        if any(ids == pickMarkerID)
            pickIdx = find(ids == pickMarkerID);
            pickPose = poses(pickIdx);
            T_pick_cam = [pickPose.R, pickPose.Translation'; 0, 0, 0, 1];
            pick_pose_base = inv(T_base_cam_stored) * T_pick_cam;
            disp('Pick marker detected.');
        end

        % Check for drop marker (ID 2)
        if any(ids == dropMarkerID)
            dropIdx = find(ids == dropMarkerID);
            dropPose = poses(dropIdx);
            T_drop_cam = [dropPose.R, dropPose.Translation'; 0, 0, 0, 1];
            drop_pose_base = inv(T_base_cam_stored) * T_drop_cam;
            disp('Drop marker detected.');
        end

        % Check for obstacle marker (ID 3) (optional)
        if any(ids == obstacleMarkerID)
            obstacleIdx = find(ids == obstacleMarkerID);
            obstaclePose = poses(obstacleIdx);
            T_obstacle_cam = [obstaclePose.R, obstaclePose.Translation'; 0, 0, 0, 1];
            obstacle_pose_base = inv(T_base_cam_stored) * T_obstacle_cam;
            disp('Obstacle marker detected.');
        end

        % Annotate detected markers on the image
        for i = 1:length(ids)
            loc = locs(:, :, i);
            I = insertShape(I, "polygon", loc, 'LineWidth', 4, 'Color', 'yellow');
            center = mean(loc, 1);
            I = insertText(I, center, sprintf('ID: %d', ids(i)), 'FontSize', 20, ...
                'BoxColor', 'yellow', 'BoxOpacity', 0.8);
        end
        
        % Display annotated image
        imshow(I);

        % Break if pick and drop markers are detected
        if ~isempty(pick_pose_base) && ~isempty(drop_pose_base)
            disp('Pick and drop markers detected.');
            break;
        end

        pause(0.1); % Pause for visualization
    end

    % Clear the webcam object
    clear cam;
end
