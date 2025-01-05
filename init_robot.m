%% Importing all the necessary libraries 

% Load the Dynamixel library
lib_name = '';
if strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', ...
        'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h', ...
        'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
end

%% Setting constants
global Px100;

% Defining the robot
robotURDF = "px100.urdf";
Px100.robot_model = importrobot(robotURDF);
Px100.config = homeConfiguration(Px100.robot_model);

% Define motor and communication settings
Px100.DXL_ID = [1, 2, 3, 4, 5];
Px100.BAUDRATE = 1000000;  
%Px100.DEVICENAME = 'COM7';

% Storing byte addresses
Px100.ADDR_OPERATING_MODE = 11;
Px100.ADDR_TORQUE_ENABLE = 64;
Px100.ADDR_GOAL_VELOCITY = 104;  
Px100.ADDR_GOAL_POS = 116;
Px100.ADDR_PRESENT_VELOCITY = 128;
Px100.ADDR_PRESENT_POSITION = 132;

% Values of the bytes
Px100.LEN_GOAL_VELOCITY = 4;
Px100.LEN_GOAL_POS = 4;
Px100.LEN_PRESENT_VELOCITY = 4;
Px100.LEN_PRESENT_POSITION = 4;   
Px100.VELOCITY_MODE = 1;          
Px100.POSITION_MODE = 3;
Px100.PROTOCOL_VERSION = 2.0;
Px100.TORQUE_ENABLE = 1;
Px100.TORQUE_DISABLE = 0;

Px100.COMM_SUCCESS = 0;            % Communication Success result value
Px100.COMM_TX_FAIL = -1001;        % Communication Tx Failed

% Setting joint Limits
Px100.JOINT_LIMITS = [-1.6, 1.53;
                       -1.68, 1.55;
                       -1.68, 1.55;
                       -1.86,2.07;
                      -0.3,1.0];

% Setting Minimum z threshold
Px100.MIN_Z_THRESH = 0.02;

% Setting movement threshold
Px100.MOVEMENT_THRESH = 0.1;

%% Initializing Comms with robot

% Initialize PortHandler and PacketHandler
Px100.port_num = portHandler(Px100.DEVICENAME);
packetHandler();

% Opening port
if openPort(Px100.port_num)
    fprintf('Succeeded to open the port!\n');
else
    fprintf('Failed to open the port!\n');
    unloadlibrary(lib_name);
    return;
end

% Setting baud-rate
if setBaudRate(Px100.port_num, Px100.BAUDRATE)
    fprintf('Succeeded to change the baudrate!\n');
else
    fprintf('Failed to change the baudrate!\n');
    unloadlibrary(lib_name);
    return;
end

%% Initializing params and getting robot to home configuration using position control mode

% Initialize groupBulkWrite Struct
Px100.groupwrite_num = groupBulkWrite(Px100.port_num, Px100.PROTOCOL_VERSION);

% Initialize Groupbulkread Structs
Px100.groupread_num = groupBulkRead(Px100.port_num, Px100.PROTOCOL_VERSION);

% Joint positions for Home config
home_config = [2050, 1250, 2500, 2350, 2500];

% Setting servo params for scaling
    Px100.angle_min = -pi;
    Px100.angle_max =  pi;
    Px100.encoder_min = 0;
    Px100.encoder_max = 4095;

% Getting the home TF
config = homeConfiguration(Px100.robot_model);
for i=1:4
    joint_angle = round(Px100.angle_min + ((home_config(i) - Px100.encoder_min) * (Px100.angle_max - Px100.angle_min)/(Px100.encoder_max - Px100.encoder_min)));
    config(i).JointPosition = joint_angle;
end
Px100.prevTF = getTransform(Px100.robot_model, config, "px100/ee_gripper_link");

% Moving robot to Home configuration
for i = 1:length(Px100.DXL_ID)

    % Setting the operation mode to position control and enabling torque
    write1ByteTxRx(Px100.port_num, Px100.PROTOCOL_VERSION, Px100.DXL_ID(i), Px100.ADDR_OPERATING_MODE, Px100.POSITION_MODE);
    write1ByteTxRx(Px100.port_num, Px100.PROTOCOL_VERSION, Px100.DXL_ID(i), Px100.ADDR_TORQUE_ENABLE, Px100.TORQUE_ENABLE);
    
    % Sending the joint position commands
    param_home_position = typecast(int32(home_config(i)), 'uint32');
    write4ByteTxRx(Px100.port_num, Px100.PROTOCOL_VERSION, Px100.DXL_ID(i), Px100.ADDR_GOAL_POS, param_home_position);

    % Creating the Bulkread template
    groupBulkReadAddParam(Px100.groupread_num, Px100.DXL_ID(i), Px100.ADDR_PRESENT_POSITION, Px100.LEN_PRESENT_POSITION);
end

pause(0.5) % Wait for the robot to reach Home config

closeGripper(true);
pause(0.7);
closeGripper(false);
pause(0.5);