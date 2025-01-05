function joint_angles = calculate_ik_analytical_sim(T, elbow_config)
    % Link lengths
    L1 = 0.0931;
    L2 = 0.10;
    L3 = 0.10;
    L4 = 0.1136;

    % Extract target position and orientation
    px = T(1, 4);
    py = T(2, 4);
    pz = T(3, 4);
    ze = T(1:3, 3); % End-effector z-axis
   % disp(ze);

    % Wrist center (Joint 4) position
    wx = px - L4 * ze(1);
    wy = py - L4 * ze(2);
    wz = pz - L4 * ze(3);

    % Step 2: Calculate Theta1
    % Base rotation angle
    theta1 = atan2(wy, wx);

    % Step 3: Horizontal (n) and Vertical (m) Distances
    % m = vertical distance from shoulder joint to wrist center
    m = wz - L1;                                                                                                                        
    disp(m);
    % n = horizontal distance in the XY plane from base to wrist center
    n = sqrt(wx^2 + wy^2)-L4;

    % Step 4: Solve for Theta3
    % Elbow angle
    cos_theta3 = ((n^2 + m^2 - L2^2 - L3^2) / (2 * L2 * L3));
   % cos_theta3 = clamp(cos_theta3, -1, 1); % Ensure numerical stability
    disp(m); disp(n);
    if strcmp(elbow_config, "up")
        theta3 = acos(cos_theta3); % Elbow-up configuration
    else
        theta3 = acos(-cos_theta3); % Elbow-down configuration
    end

    % Step 5: Solve for Theta2
    % Shoulder angle
    beta = pi/2+atan2(m, n); % Angle to wrist center
    si = acos((n^2 + m^2 + L2^2 - L3^2) / (2 * L2 * sqrt(n^2 + m^2))); % Angle in triangle
    theta2 =-(beta - si); % Adjust for geometry

    % Step 6: Solve for Theta4
    % Wrist alignment
    theta4 = - theta2 - theta3;

    % Return joint angles
    joint_angles = [theta1, theta2, theta3, theta4];

    % Debugging Information
    disp("Wrist center (wx, wy, wz): "), disp([wx, wy, wz]);
    disp("Computed joint angles (theta1, theta2, theta3, theta4): "), disp(joint_angles);
end

% Helper function to clamp values
function clamped_value = clamp(value, min_val, max_val)
    clamped_value = max(min_val, min(value, max_val));
end

