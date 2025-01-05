function pick_joint_config = calculate_ik_sim(pick_pose, Px100,count)
    % Create an inverse kinematics solver for the robot
    ik = inverseKinematics('RigidBodyTree', Px100.robot, 'SolverAlgorithm', 'BFGSGradientProjection');
    
    % Increase number of iterations and random restarts
    ik.SolverParameters.MaxIterations = 1000;  % More iterations
    ik.SolverParameters.NumRandomRestarts = 100;  % More restarts
    
    % Decrease the tolerance values to make the solver less tolerant
    ik.SolverParameters.TolFun = 1e-10;  % Function tolerance (lower = stricter tolerance)
    ik.SolverParameters.TolX = 1e-10;    % Solution change tolerance (lower = stricter tolerance)
    
    % Define the weights for position and orientation accuracy
    weights = [1, 0, 0, 1, 1, 1];  % Prioritize position accuracy over orientation1
    
    % Use the current configuration as the initial guess (get home configuration)
    initial_guess = Px100.homeConfig;  % Get home configuration of the robot
    
    % Solve for the joint configuration that achieves the pick_pose
    [config_sol, sol_info] = ik('px100/ee_gripper_link', pick_pose, weights, initial_guess);
    
    % Check if the solver has succeeded
    if sol_info.Status > 0  % Success code is 1
        pick_joint_config = config_sol;
        if count == 1
            disp(pick_joint_config(1:4));
        end
        eePose = getTransform(Px100.robot, pick_joint_config, 'px100/ee_gripper_link');
    %    disp("End effector Transform");
     %   disp(eePose);
    else
        error('Inverse kinematics failed for pick_pose');
    end
end


