% Global Robot Structure
clear all;
global Px100;
Px100 = struct();
global joint_positions pick_pose_base drop_pose_base intermediate_pose_base_1 intermediate_pose_base_2 intermediate_pose_base_3;

% Define pick and drop poses
pick_pose_base = trvec2tform([0.15, 0.10, 0.05]);  % Default pick pose ( You can change it)
drop_pose_base = trvec2tform([0.15, -0.10, 0.05]);  % Default drop pose ( You can change it)

% Define the intermediate positions. 
intermediate_pose_base_1 = trvec2tform([pick_pose_base(1,4), pick_pose_base(2,4), pick_pose_base(3,4) + 0.1]); 
intermediate_pose_base_2 = trvec2tform([drop_pose_base(1,4), drop_pose_base(2,4), drop_pose_base(3,4) + 0.1]); 
intermediate_pose_base_3 = trvec2tform([drop_pose_base(1,4), drop_pose_base(2,4), drop_pose_base(3,4) + 0.1]); 


robot = importrobot('px100.urdf');% path to the urdf file;

% Initialize simulation environment
init_robot_sim(pick_pose_base, drop_pose_base,robot);

%% Main Loop
try
    % Call main task
    main_task();
catch ME
    disp('Error encountered:');
    disp(ME.message);
end

function main_task()
    global joint_positions Px100 pick_pose_base drop_pose_base intermediate_pose_base_1 intermediate_pose_base_2 intermediate_pose_base_3;
    min_z_thresh = 0.02;
    %% Reaching the Payload
    while true
        
        % Get current joint positions (first 4 only)
        joint_positions = get_current_joint_positions();

        % Write Inverse Kinematics for PincherX100. The input should take the pick_pose and the output of it should be the pick_joint_config( joint configuration wrt pick_pose)
        intermediate_joint_config = calculate_ik_sim(intermediate_pose_base_1,Px100,0);
        pick_joint_config = calculate_ik_sim(pick_pose_base,Px100,0);

        % Write the code for trajectory planning for reaching the payload (pick_pose). The input should be the current pose, pick_pose. The output should be "joint_space_waypoints" n x 4 matrix.
        % Number of waypoints for smooth trajectory
        num_waypoints = 100;
        
        % Plan the trajectory
    %    disp(pick_joint_config);
        joint_space_waypoints = plan_end_effector_trajectory(joint_positions, intermediate_joint_config, pick_joint_config(1:4), num_waypoints,  min_z_thresh, 1);
        
        % Execute trajectory (first 4 joints only)
        execute_trajectory(joint_space_waypoints);

        % Attach cylinder at pick pose
        attachCylinder();

        pause(0.5);
        break;  % Exit loop after first run for simplicity
    end

    %% Transporting Payload
    while true
        % Get current joint positions (first 4 only)
        joint_positions = get_current_joint_positions();

        %Write Inverse Kinematics for PincherX100 . The input should take the drop_pose and the output of it should be the drop_joint_config( joint configuration wrt drop_pose)
        intermediate_joint_config = calculate_ik_sim(intermediate_pose_base_2,Px100,0);
        drop_joint_config = calculate_ik_sim(drop_pose_base,Px100,1);

                
        % Write the code for trajectory planning from pick_pose to drop_pose. The input should be the pick_pose, drop_pose. The output should be "joint_space_waypoints" n x 4 matrix.
       % Number of waypoints for smooth trajectory
        num_waypoints = 200;
        
        % Plan the trajectory
        disp(drop_joint_config);
        joint_space_waypoints = plan_end_effector_trajectory(joint_positions, intermediate_joint_config, drop_joint_config(1:4), num_waypoints,  min_z_thresh, 0);
        % Execute trajectory
        execute_trajectory(joint_space_waypoints);

        % Detach cylinder at drop pose
        detachCylinder();

        pause(0.5);
        break;
    end

    %% Returning to Home Configuration
    while true
        % Get current joint positions (first 4 only)
        joint_positions = get_current_joint_positions();
        intermediate_joint_config = calculate_ik_sim(intermediate_pose_base_3,Px100,0);
        home_config = [0, 0, 0, 0];
        % Write the code for trajectory planning from drop_pose to home_pose. The input should be the drop_pose, home.Config . The output should be "joint_space_waypoints" n x 4 matrix.
        joint_space_waypoints = plan_end_effector_trajectory(joint_positions, intermediate_joint_config, home_config, num_waypoints,  min_z_thresh, 0);

        % Execute trajectory
        execute_trajectory(joint_space_waypoints);
        break;
    end
end
% Setting up simulation environment
function init_robot_sim(pick_pose, drop_pose,robot)
    global Px100;
  %  robot = importrobot('C:\Users\aryas\OneDrive\Documents\MATLAB\Pincherx100\px100.urdf');% path to the urdf file;
    Px100.robot = robot;
    Px100.robot.DataFormat = 'row';
    Px100.homeConfig = homeConfiguration(Px100.robot);
    Px100.currentConfig = Px100.homeConfig;
    Px100.robot.Gravity = [0, 0, -9.81];  % Apply gravity in the negative Z-direction
    % Display the robot in its home configuration
    figure;
    show(Px100.robot, Px100.homeConfig, 'PreservePlot', false, 'Collisions', 'on');
    hold on;

    % Initialize environment with platform and cylinder
    Px100.env = {};

    % Add platform
    platform = collisionBox(0.6, 1, 0.02);
    platform.Pose = trvec2tform([0, 0, -0.01]);
    Px100.env{1} = platform;

    % Add cylinder at pick pose
    cylinderRadius = 0.03;
    cylinderHeight = 0.05;
    cylinder = collisionCylinder(cylinderRadius, cylinderHeight);
    cylinder.Pose = pick_pose * trvec2tform([0, 0, -cylinderHeight / 2]); % Centered at pick position
    Px100.env{2} = cylinder;

    % Add pick and drop markers
    add_markers(pick_pose, drop_pose);

    % Display environment objects
    for i = 1:length(Px100.env)
        show(Px100.env{i});
    end
end
% Placing markers at Pick and drop position
function add_markers(pickPose, dropPose)
    global Px100;

    % Add pick marker
    pickMarker = collisionSphere(0.01);
    pickMarker.Pose = pickPose;
    Px100.env{3} = pickMarker;

    % Add drop marker
    dropMarker = collisionSphere(0.01);
    dropMarker.Pose = dropPose;
    Px100.env{4} = dropMarker;

    % Display markers
    show(pickMarker);
    show(dropMarker);
end

%Imposing joint limit constraints and minimum Z threshold (Updated)
function execute_trajectory(waypoints)
    global Px100;

    % Define joint limits
    joint_limits = [
        -1.6, 1.53;  % Joint 1
        -1.68, 1.55; % Joint 2
        -1.68, 1.55; % Joint 3
        -1.86, 2.07  % Joint 4
    ];

    % Minimum allowed z position
    min_z_thresh = 0.02; % 0.02 m

    % Maximum allowed Cartesian distance between consecutive waypoints
    movement_thresh = 0.1;  % 0.1 m

    % Loop through waypoints
    for i = 1:size(waypoints, 1)
        % Apply joint limits for each joint
        for j = 1:4
            waypoints(i, j) = max(joint_limits(j, 1), min(joint_limits(j, 2), waypoints(i, j)));
        end

        % Update current configuration for the robot
        Px100.currentConfig(1:4) = waypoints(i, :);

        % Compute the Cartesian position of the end-effector
        eePose = getTransform(Px100.robot, Px100.currentConfig, 'px100/ee_gripper_link');
        eePosition = tform2trvec(eePose); % Extract Cartesian position (x, y, z)

        % Check the z position constraint
        if eePosition(3) < min_z_thresh
            error('Z position constraint violated at waypoint %d. Execution stopped.', i);
        end

        % Check Cartesian distance between consecutive waypoints(Updated)
        if i > 1
            prevEePose = getTransform(Px100.robot, [waypoints(i-1, :), Px100.currentConfig(5:end)], 'px100/ee_gripper_link');
            prevEePosition = tform2trvec(prevEePose);

            dist = norm(eePosition - prevEePosition);
            if dist > movement_thresh
                error('Cartesian distance between consecutive waypoints exceeds threshold at waypoint %d. Execution stopped.', i);
            end
        end

        % Display the robot's configuration
        show(Px100.robot, Px100.currentConfig, 'PreservePlot', false, 'Collisions', 'on');
        drawnow;
    end
end

function attachCylinder()
    global Px100;
    if isempty(Px100.env{2})
        disp('Cylinder is already attached or does not exist.');
        return;
    end

    cylinder = Px100.env{2};
    cylinderBody = rigidBody('cylinderBody');
    cylinderJoint = rigidBodyJoint('cylinderJoint', 'fixed');

    % Get end-effector pose at the current config
    eePose = getTransform(Px100.robot, Px100.currentConfig, 'px100/ee_gripper_link');
    setFixedTransform(cylinderJoint, eePose \ cylinder.Pose);
    addCollision(cylinderBody, cylinder, inv(cylinder.Pose));
    cylinderBody.Joint = cylinderJoint;
    addBody(Px100.robot, cylinderBody, 'px100/ee_gripper_link');

    % Remove the cylinder from the environment to indicate it is picked up
    Px100.env{2} = [];
end

function detachCylinder()
    global Px100;

    % Remove cylinder from robot
    removeBody(Px100.robot, 'cylinderBody');

    % Get the end-effector pose at the current configuration
    dropPose = getTransform(Px100.robot, Px100.currentConfig, 'px100/ee_gripper_link');
    % Define cylinder dimensions
    cylinderRadius = 0.03;
    cylinderHeight = 0.05;

    % Create a new cylinder collision object
    cylinder = collisionCylinder(cylinderRadius, cylinderHeight);

    % Adjust cylinder pose relative to the drop pose
    cylinder.Pose = dropPose * trvec2tform([0, 0, -cylinderHeight / 2]);

    % Re-add the cylinder to the environment at the new pose
    Px100.env{2} = cylinder;

    % Display the updated environment for verification
    show(Px100.env{2});
end

function joint_positions = get_current_joint_positions()
    global Px100;
    % Read only the first 4 joint positions
    joint_positions = Px100.currentConfig(1:4);
end