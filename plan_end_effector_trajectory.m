function joint_space_waypoints = plan_end_effector_trajectory(current_config, intermediate_config, target_config, num_waypoints, min_z_thresh, is_picking)
    % Initialize the joint space waypoints
    disp('Target configuration:');
    disp(target_config);

    % Split the number of waypoints between two phases
    num_waypoints_phase1 = floor(num_waypoints / 2);
    num_waypoints_phase2 = num_waypoints - num_waypoints_phase1;

    % Time vectors for each phase
    t1 = linspace(0, 1, num_waypoints_phase1); % Phase 1: to intermediate
    t2 = linspace(0, 1, num_waypoints_phase2); % Phase 2: to target

    % Initialize the joint space waypoints
    disp(current_config);
    joint_space_waypoints = zeros(num_waypoints, length(current_config));

    if is_picking
        % Picking: Phase 1 (current -> intermediate hover position)
        for joint_idx = 1:length(current_config)
            joint_space_waypoints(1:num_waypoints_phase1, joint_idx) = ...
                linear_interpolation(current_config(joint_idx), intermediate_config(joint_idx), t1);
        end

        % Picking: Phase 2 (intermediate hover -> target position)
        for joint_idx = 1:length(current_config)
            joint_space_waypoints(num_waypoints_phase1+1:end, joint_idx) = ...
                linear_interpolation(intermediate_config(joint_idx), target_config(joint_idx), t2);
        end

        % Clamp Z motion to ensure it stays above min_z_thresh
        joint_space_waypoints(:, 3) = max(joint_space_waypoints(:, 3), min_z_thresh);

    else
        % Placing: Phase 1 (current -> intermediate hover position)
        for joint_idx = 1:length(current_config)
            joint_space_waypoints(1:num_waypoints_phase1, joint_idx) = ...
                linear_interpolation(current_config(joint_idx), intermediate_config(joint_idx), t1);
        end

        % Placing: Phase 2 (intermediate hover -> target position)
        for joint_idx = 1:length(current_config)
            joint_space_waypoints(num_waypoints_phase1+1:end, joint_idx) = ...
                linear_interpolation(intermediate_config(joint_idx), target_config(joint_idx), t2);
        end

        % Clamp Z motion to ensure it stays above min_z_thresh
        joint_space_waypoints(:, 3) = max(joint_space_waypoints(:, 3), min_z_thresh);
    end

    % Force the last waypoint to exactly match the target configuration
    joint_space_waypoints(end, :) = target_config;

    % Debugging: Print waypoints before and after the adjustment
end

% Linear interpolation function
function linear_values = linear_interpolation(start_value, end_value, t)
    % Linear interpolation between start_value and end_value
    linear_values = start_value + (end_value - start_value) * t;
end