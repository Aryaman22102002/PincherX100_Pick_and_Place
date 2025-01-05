function pick_joint_config = ik_px100(pick_pose, Px100, count)
    % Create an inverse kinematics solver for the robot   
    robot = Px100.robot_model;
    ik = inverseKinematics('RigidBodyTree', robot, 'SolverAlgorithm', 'BFGSGradientProjection');
    
    % Increase number of iterations and random restarts
    ik.SolverParameters.MaxIterations = 5000;  % More iterations
    ik.SolverParameters.NumRandomRestarts = 100;  % More restarts
    
    % Decrease the tolerance values to make the solver less tolerant
    ik.SolverParameters.TolFun = 1e-10;  % Function tolerance (lower = stricter tolerance)
    ik.SolverParameters.TolX = 1e-10;    % Solution change tolerance (lower = stricter tolerance)
    
    % Define the weights for position and orientation accuracy
    weights = [1, 0, 0, 1, 1, 1];  % Prioritize position accuracy over orientation1
    
    % Use the current configuration as the initial guess (get home configuration)
    initial_guess = homeConfiguration(robot);  % Get home configuration of the robot
    
    % Solve for the joint configuration that achieves the pick_pose
    [config_sol, sol_info] = ik('px100/ee_gripper_link', pick_pose, weights, initial_guess);

    if config_sol(1).JointPosition < -1.6 || config_sol(1).JointPosition > 1.53
        disp("Error1");
    end
    if config_sol(2).JointPosition < -1.68 || config_sol(2).JointPosition > 1.55
    disp("Error2");
    end
    if config_sol(3).JointPosition < -1.68 || config_sol(3).JointPosition > 1.55
    disp("Error3");
    end
    if config_sol(4).JointPosition < -1.86 || config_sol(4).JointPosition > 2.07
    disp("Error4");
    end
    % Check if the solver has succeeded
    if sol_info.Status > 0  % Success code is 1
        pick_joint_config = [config_sol(1:4).JointPosition];
    else
        error('Inverse kinematics failed for pick_pose');
    end
end