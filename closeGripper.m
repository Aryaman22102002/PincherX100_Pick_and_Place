function closeGripper(isClose)
    global Px100;
    robot = Px100;
     CLOSE = 2048; 
  %  CLOSE = 1900;
    OPEN = 2573;
    if(isClose == true)
        param_home_position = typecast(int32(CLOSE), 'uint32');
    else
        param_home_position = typecast(int32(OPEN), 'uint32');
    end
    write4ByteTxRx(Px100.port_num, Px100.PROTOCOL_VERSION, Px100.DXL_ID(5), Px100.ADDR_GOAL_POS, param_home_position);
    disp("Close Gripper Exited");
end