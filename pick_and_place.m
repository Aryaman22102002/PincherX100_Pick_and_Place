global Px100;

% Set your port number
Px100.DEVICENAME = 'COM4';

% Initializing the robot
init_robot

% Getting the minimum Z threshold
min_z_thresh = Px100.MIN_Z_THRESH;

% Getting max movement distance
movement_thresh = Px100.MOVEMENT_THRESH;

pick_pose_base = trvec2tform([0.15, 0.10, 0.05]);  % Default pick pose ( You can change it)
drop_pose_base = trvec2tform([0.15, -0.10, 0.05]);  % Default drop pose ( You can change it)

%% Reaching the Payload
while(true)
 joint_limits = [
    -1.6, 1.53;  % Joint 1: Min, Max
    -1.68, 1.55; % Joint 2: Min, Max
    -1.68, 1.55; % Joint 3: Min, Max
    -1.86, 2.07  % Joint 4: Min, Max
];
    % Getting joint positions
    joint_positions = get_joint_pos();

    % Get the postions of pick, place and obstacle
  %  [pick_pose_base, drop_pose_base, obstacle_pose_base] = get_pick_and_place_position();
  %  pick_pose_base(3,4) = 0.05;
  %  drop_pose_base(3,4) = 0.05;

    % Define the intermediate positions. 
    intermediate_pose_base_1 = trvec2tform([pick_pose_base(1,4), pick_pose_base(2,4), pick_pose_base(3,4) + 0.1]); 
    intermediate_pose_base_2 = trvec2tform([drop_pose_base(1,4), drop_pose_base(2,4), drop_pose_base(3,4)+ 0.1]); 
    intermediate_pose_base_3 = trvec2tform([drop_pose_base(1,4), drop_pose_base(2,4), drop_pose_base(3,4) + 0.1]); 

    % Get the initial end-effector position using forward kinematics.
    a0 = 0; a1 = 0; a2 = 0.1; a3 = 0.1;
    alpha0 = 0; alpha1 = -pi/2; alpha2 = 0; alpha3 = 0;
    d1 = 0.0931; d2 = 0; d3 = 0.035; d4 = 0;
    theta1 = joint_positions(1,1); theta2 = joint_positions(1,2); theta3 = joint_positions(1,3); theta4 = joint_positions(1,4);

    T01 = fk_px100(a0,alpha0,d1,theta1); 
    T12 = fk_px100(a1,alpha1,d2,theta2);
    T23 = fk_px100(a2,alpha2,d3,theta3);
    T34 = fk_px100(a3,alpha3,d4,theta4);
    
    T4e = [1, 0, 0, 0.1136;
        0, 0, -1, 0;
        0, 1, 0, 0;
        0, 0, 0, 1];
    
    T04 = T01 * T12 * T23 * T34;
    T0e = T04 * T4e;

    % Use the inverse kinematics function to get the joint configurations.
    intermediate_joint_config = ik_px100(intermediate_pose_base_1,"up");
    pick_joint_config = ik_px100(pick_pose_base,"up");

    % Use the plan_end_effector_trajectory to get the waypoints in joint-space between the initial position, first intermediate waypoint and the pick up position. 
    num_waypoints = 50;
    joint_space_waypoints = plan_end_effector_trajectory(joint_positions, intermediate_joint_config, pick_joint_config(1:4), num_waypoints,  min_z_thresh, 1);

    N = length(joint_space_waypoints);

    for i=1:N
        % Give position command to robot
        set_joint_pos(joint_space_waypoints(i, :));

        while(true)
            % Break out of the loop when end-effector reaches the point
            target_joint_pos = joint_space_waypoints(i, :);  
            % Get the current joint positions
            current_joint_pos = get_joint_pos();  
            disp(['Waypoint ', num2str(i), ' reached!']);
            break;  % Exit the while loop and proceed to the next waypoint
        end
        pause(0.1)
    end
    break;
end

% Close the gripper
closeGripper(1);
pause(0.5) % Wait for the gripper to close

%% Transporting Payload
while(true)
    % Getting joint positions
    joint_positions = get_joint_pos();

    % Get the initial end-effector position using forward kinematics.
    theta1 = joint_positions(1,1); theta2 = joint_positions(1,2); theta3 = joint_positions(1,3); theta4 = joint_positions(1,4);
    T01 = fk_px100(a0,alpha0,d1,theta1); 
    T12 = fk_px100(a1,alpha1,d2,theta2);
    T23 = fk_px100(a2,alpha2,d3,theta3);
    T34 = fk_px100(a3,alpha3,d4,theta4);
    
    T04 = T01 * T12 * T23 * T34;
    T0e = T04 * T4e;

    % Use the inverse kinematics function to get the joint configurations.
    intermediate_joint_config = ik_px100(intermediate_pose_base_2,"up");
    drop_joint_config = ik_px100(drop_pose_base,"up");

    % Use the plan_end_effector_trajectory to get the waypoints in joint-space between the pick-up position, second intermediate waypoint and the pick up position. 
    num_waypoints = 50;
    joint_space_waypoints = plan_end_effector_trajectory(joint_positions, intermediate_joint_config, drop_joint_config(1:4), num_waypoints,  min_z_thresh, 1);

   N = length(joint_space_waypoints);

    for i=1:N
        % Give position command to robot
        set_joint_pos(joint_space_waypoints(i, :));

        while(true)
            % Break out of the loop when end-effector reaches the point
            target_joint_pos = joint_space_waypoints(i, :);  
            % Get the current joint positions
            current_joint_pos = get_joint_pos();  
            disp(['Waypoint ', num2str(i), ' reached!']);
            break;  % Exit the while loop and proceed to the next waypoint
        end
       pause(0.1)
    end
 % Open the gripper
closeGripper(0);
break;
end

pause(0.5);

%% Returning to Home config

while(true)
    % Getting joint positions
    joint_positions = get_joint_pos();

    % Get the initial end-effector position using forward kinematics
    theta1 = joint_positions(1,1); theta2 = joint_positions(1,2); theta3 = joint_positions(1,3); theta4 = joint_positions(1,4);
    T01 = fk_px100(a0,alpha0,d1,theta1); 
    T12 = fk_px100(a1,alpha1,d2,theta2);
    T23 = fk_px100(a2,alpha2,d3,theta3);
    T34 = fk_px100(a3,alpha3,d4,theta4);
    
    T04 = T01 * T12 * T23 * T34;
    T0e = T04 * T4e;

      % Define home configuration.
      home_config = [0, 0, 0, 0];
      num_waypoints = 50;

     % Use the inverse kinematics function to get the joint configurations.
     intermediate_joint_config = ik_px100(intermediate_pose_base_3,"up");
   
     
     % Use the plan_end_effector_trajectory to get the waypoints in joint-space between the drop position, third intermediate waypoint and the home configuration. 
     joint_space_waypoints = plan_end_effector_trajectory(joint_positions, intermediate_joint_config, home_config, num_waypoints,  min_z_thresh, 1);
  
    N = length(joint_space_waypoints);

    for i=1:N
        % Give position command to robot
        set_joint_pos(joint_space_waypoints(i, :));

        while(true)
            % Break out of the loop when end-effector reaches the point
            target_joint_pos = joint_space_waypoints(i, :); 
            % Get the current joint positions
            current_joint_pos = get_joint_pos();  
            disp(['Waypoint ', num2str(i), ' reached!']);
            break;  % Exit the while loop and proceed to the next waypoint
        end
        pause(0.1)
    end
    break;
end