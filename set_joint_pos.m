function valid = set_joint_pos(joint_values)
    global Px100;
    robot = Px100;

    % Setting minimum threshold
    MIN_Z_THRESH = robot.MIN_Z_THRESH;
    MOVEMENT_THRESH = robot.MOVEMENT_THRESH;

    valid = true;
    for i=1:4
        
        % Converting decimal values to degrees
        val_read = joint_values(i);

        % Checking for joint limit violations
        if(val_read < robot.JOINT_LIMITS(i, 1) || val_read > robot.JOINT_LIMITS(i, 2))
            valid = false;
            fprintf("\n\n==========x==========\n\nWARNING!\nJoint limit exceeded!\n")
            fprintf("%d",i);
        end

        % Converting encoder values to joint angles
        robot.config(i).JointPosition = joint_values(i);
    end

    % Computing the position along Z-axis
    TF = getTransform(robot.robot_model, robot.config, "px100/ee_gripper_link");
    if(TF(3,4) < MIN_Z_THRESH)
        valid = false;
        fprintf("\n\n==========x==========\n\nWARNING!\nEnd-Effector out of bounds!\n")
    end

    % Getting the distance
    dist = sqrt(sum((TF(1:3,4) - Px100.prevTF(1:3,4)).^2));

    if(dist > MOVEMENT_THRESH)
        valid = false;
        fprintf("\n\n==========x==========\n\nWARNING!\nLarge movement detected!\n")
    end

    if valid
        for i=1:4
            % Converting the angle to encoder value
            disp(joint_values);
            joint_value = Px100.encoder_min + ((Px100.encoder_max - Px100.encoder_min)*(joint_values(i) - Px100.angle_min)/(Px100.angle_max - Px100.angle_min));
            param_goal_pos = typecast(int32(joint_value), 'uint32');
            groupBulkWriteAddParam(robot.groupwrite_num, robot.DXL_ID(i), robot.ADDR_GOAL_POS, robot.LEN_GOAL_POS, param_goal_pos, robot.LEN_GOAL_POS);
        end

        % Writing the bulk packet
        groupBulkWriteTxPacket(robot.groupwrite_num);
        groupBulkWriteClearParam(robot.groupwrite_num);

        % Updating previous TF
        Px100.prevTF = TF;
    end
end