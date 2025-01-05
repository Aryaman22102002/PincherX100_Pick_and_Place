function val_read = get_joint_pos()
    global Px100;
    robot = Px100;
    
    % Reading the packet
    groupBulkReadTxRxPacket(robot.groupread_num);
    
    % Extracting and storing joint values
    val_read = zeros(1,4);
    for i=1:4
        temp = groupBulkReadGetData(robot.groupread_num, robot.DXL_ID(i), robot.ADDR_PRESENT_POSITION, robot.LEN_PRESENT_POSITION);
        % Updated part ( Converting encoder values to joint angles)
        temp = Px100.angle_min + ((temp - Px100.encoder_min) * (Px100.angle_max - Px100.angle_min)/(Px100.encoder_max - Px100.encoder_min));
        val_read(i) = temp;
    end
end