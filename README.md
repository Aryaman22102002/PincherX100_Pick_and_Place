# PincherX100_Pick_and_Place
This project was done as a part of the ME 5250 (Robot Mechanics and Control) course at Northeastern University. 


### Table of Content
* [Objectives](#objectives)
* [The PincherX100](#the-pincherx100)
  * [Key Features](#key-features)
  * [Image Of The Robot](#image-of-the-robot)
  * [Frame Assignment For The Robot Along With Its DH Parameters](#frame-assignment-for-the-robot-along-with-its-dh-parameters)
  * [DH Table](#dh-table)
* [Constraints Imposed On The Robot](#constraints-imposed-on-the-robot)
* [Our Approach](#our-approach)
  * [The Main Pick and Place Code](#the-main-pick-and-place-code)
  * [The Numerical Inverse Kinematics Code](#the-numerical-inverse-kinematics-code)
  * [The Trajectory Planning Code](#the-trajectory-planning-code)
  * [The Obstacle Avoidance Code](#the-obstacle-avoidance-code)
* [Some-Information-Regarding-What-Each-Code-File-Does](#some-information-regarding-what-each-code-file-does)
* [Results](#Results)
  * [Pick and Place In MATLAB Simulation](#pick-and-place-in-matlab-simulation)
  * [Pick and Place Using The Real PincherX100](#pick-and-place-using-the-real-pincherx100)
  * [Pick and Place Along With Obstacle Avoidance](#pick-and-place-along-with-obstacle-avoidance)
* [Acknowledgements](#acknowledgements)
* [Contributors](#contributors)

### Objectives
In this project, we worked on the PincherX100 arm, a 4 DOF manipulator to perform a pick and place task using MATLAB. The object (payload) to be picked was a cylinder with a radius of 2 cm and height of 5 cm. The main tasks of the project included:
- Reaching the payload's location and gripping it.
- Moving the payload to the target location.
- Releasing the payload.
- Reaching the home configuration.

We performed the above task in MATLAB simulation as well as on the hardware PincherX100 arm itself. The above task considered the environment to be an obstacle-free environment. <br>
Apart from this, we also performed the same pick-and-place task, but in the presence of an obstalce in the evironment. The obstacle was a bigger cylinder with a height of 10 cm and diameter of 5 cm. We planned an planning to be able to avoid collisions with such an obstacle and complete the pick and place task if a feasible solution exists.


### The PincherX100
The PincherX 100 is a compact, lightweight robotic arm designed by Trossen Robotics as part of their Interbotix series. It is commonly used in research, education, etc. 

#### Key Features
1) 4 Degrees of Freedom
2) 50g Payload Capacity
3) 300 mm Arm Reach
4) 600 mm Arm Span
5) Revolute Wrist Joint

#### Image Of The Robot
![image](https://github.com/user-attachments/assets/086ca552-c3c7-4b98-a070-751e69235844)  

#### Frame Assignment For The Robot Along With Its DH Parameters
![image](https://github.com/user-attachments/assets/097ebba9-e49a-4b73-9288-f3fd75f1c1f8)

#### DH Table
![image](https://github.com/user-attachments/assets/cbfd1790-15e3-4cfb-b429-25cab5e52cd5)


### Constraints Imposed On The Robot
The following constraints will be imposed on the robot for safe operaIon. If any of them are (about to be) violated, the program will give an error and stop the robot. <br>

1. The following joint limits are imposed on the joint angles to avoid self-collisions:

| Joint  | Minimum Angle (radians) | Maximum Angle (radians)| 
| ------------- | ------------- | ------------- |
| 1  | -1.60  | 1.53  |
| 2 | -1.68  | 1.55  |
| 3  | -1.68  | 1.55  |
| 4  | -1.86  | 2.07  |
| 5  | -0.30 | 1.00  |

2. To ensure that the end-eﬀector does not hit the ground, it is constrained to stay above the speciﬁed the minimum z coordinate threshold: ```min_z_thresh (=0.02m)```.

3. The distance between any two consecuIve waypoints in the Cartesian space must be less than the speciﬁed threshold: ```movement_thresh(=0.1m)```.


### Our Approach

#### The Main Pick and Place Code
The main pick and place code starts by initializing the robot and setting thresholds for movement and height. The task is divided into three main parts: reaching the payload, transporting it, and returning to the home position. For each
phase, the code uses inverse kinematics (IK) to calculate the required joint configurations for the robot to move through intermediate, pick, drop, and home positions. Joint-space trajectories are generated with waypoints, and the robot moves step by step, checking its current position against the target at each waypoint to ensure accuracy. Forward kinematics (FK) is used to calculate the end-effector's position and validate the trajectory. The gripper is controlled to pick and release the payload at the appropriate points, while intermediate poses ensure smooth and collision-free motion. Error checks and small pauses between movements help maintain stability and precision throughout the task.

#### The Numerical Inverse Kinematics Code
This code calculates the inverse kinematics (IK) for a PX100 robotic arm, determining the joint configurations needed to achieve a desired end-effector pose (pick_pose). It uses MATLAB's inverseKinematics solver with the BFGS Gradient Projection algorithm and configures it for higher accuracy by increasing the maximum iterations and random restarts, while tightening tolerance values for stricter convergence. The weights prioritize position accuracy over orientation, and the robot's home configuration is used as the initial guess. The function checks if the computed joint positions fall within predefined limits for each joint and validates the solver's success before returning the joint configuration or reporting an error.

#### The Trajectory Planning Code
The trajectory planning is divided into two phases: (1) moving from the current configuration to an intermediate hovering position, and (2) moving from the intermediate position to the target configuration. The number of waypoints is split proportionally between these phases, and linear interpolation is applied for smooth transitions in joint values. Depending on whether the robot is picking or placing an object (is_picking), the trajectory maintains the Z-coordinate above a specified min_z_thresh to avoid collisions or maintain safe operation. The final waypoint is explicitly set to match the target configuration to ensure precision. This approach ensures a smooth and safe motion trajectory for the manipulator.

#### The Obstacle Avoidance Code
The obstacle was placed in the center of the configuration between the pick and drop positions. Its height was also double that of the object. To avoid the obstacle, we made the arm to go over the obstacle.<br>
To make the arm go over the obstacle, we took in the obstacle’s position and we already had its radius and height. As we were already using extra intermediate waypoints to make the arm hover before picking and dropping the object, we decided to use the intermediate waypoints approach for obstacle avoidance too. We already had three intermediate waypoints: 1) To hover over the object before picking it up, 2) To pick up the object, reach the x and y position of the drop pose and hover over it, 3) After dropping the object, hover again at the same position it hovered before placing followed by going back to home configuration. So, we added a fourth intermediate point where after picking up the object, instead of directly going to the hover position over the drop pose, it would first go about in height (Z axis) at the pick position itself, and then go to the drop point’s hover position. We also used the cylinder's height and radius to set the intermediate point’s x, y and z coordinates to ensure that it won’t collide into the obstacle.


### Some Information Regarding What Each Code File Does
- **Visual_env_with_numerical_ik.m** - This is the main logic for the manipulator's pick and place using the numerical inverse kinematics function in MATLAB simulation. 
- **calculate_ik_sim.m** - This is the numerical inverse kinematics function in MATLAB simulation.
- **closeGripper.m** - This function is used to open and close the arm's gripper for the real PincherX100 arm.
- **fk_px100.m** - This function is used to calculate the forward kinematics for the arm.
- **get_joint_pos.m** - It gives the current joint positions of the arm as output.
- **get_pick_and_place_position.m** - It gives the payload's pick position and target position as well as the obstacle's position using a monocular camera.
- **ik_px100.m** - This is the numerical inverse kinematics function for the real PincherX100 arm.
- **init_robot.m** - This function initializes all the necessary parameters for the robot.
- **pick_and_place.m** - This is the main logic for the manipulator's pick and place for the real PincherX100 arm.
- **pick_and_place_with_obstacle_avoidance.m** - This is the main logic for the manipulator's pick and place with obstacle avoidance for the real PincherX100 arm.
- **plan_end_effector_trajectory.m** - This function designs the trajectory for the manipulator's end effector.
- **set_joint_pos.m** - This function is used to actually move the arm's joints to a given set of joint positions.


### Results

#### Pick and Place In MATLAB Simulation

https://github.com/user-attachments/assets/59a23fb7-de72-4df2-b47f-500e2eb73001

#### Pick and Place Using The Real PincherX100

https://github.com/user-attachments/assets/50dfe666-6c17-4f98-b5eb-7335bceaed71

#### Pick and Place Along With Obstacle Avoidance

https://github.com/user-attachments/assets/68d37a28-041e-4257-92e6-511fd6b5b974


### Acknowledgements
We want our course instructor, Prof. Yasin Yazicioglu, as well as all the teaching assistants who helped us immensely not only while doing this project but throughout the course. They were great at teaching and managing the course and were always available and enthusiastic about solving everyone's doubts. <br>

We would also like to thank Northeastern University for providing us access to the PincherX100 robot manipulator. 


### Contributors
- [Aryaman Shardul](https://github.com/Aryaman22102002)
- [Sairam Sridharan](https://github.com/Sairamzz)


### Contact
- Aryaman Shardul - lnu.arya@northeastern.edu
- Sairam Sridharan - sridharan.sai@northeastern.edu
