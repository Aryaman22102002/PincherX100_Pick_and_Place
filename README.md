# PincherX100_Pick_and_Place
This project was done as a part of the ME 5250 (Robot Mechanics and Control) course at Northeastern University. 


### Table of Content


### Objective
In this project, we worked on the PincherX100 arm, a 4 DOF manipulator to perform a pick and place task using MATLAB. The object (payload) to be picked was a cylinder with a radius of 2 cm and height of 5 cm. The main tasks of the project included:
- Reaching the payload's location and gripping it.
- Moving the payload to the target location.
- Releasing the payload.
- Reaching the home configuration.

We performed the above task in MATLAB simulation as well as on the hardware PincherX100 arm itself. The above task considered the environment to be an obstacle-free environment. <br>
Apart from this, we also performed the same pick-and-place task, but in the presence of an obstalce in the evironment. The obstacle was a bigger cylinder with a height of 10 cm and diameter of 5 cm. We planned an planning to be able to avoid collisions with such an obstacle and complete the pick and place task if a feasible solution exists.


### The PincherX100




### Constraints Imposed on the robot
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


### Our approach

##### The Main Pick and Place Code
The main pick and place code starts by initializing the robot and setting thresholds for movement and height. The task is divided into three main parts: reaching the payload, transporting it, and returning to the home position. For each
phase, the code uses inverse kinematics (IK) to calculate the required joint configurations for the robot to move through intermediate, pick, drop, and home positions. Joint-space trajectories are generated with waypoints, and the robot moves step by step, checking its current position against the target at each waypoint to ensure accuracy. Forward kinematics (FK) is used to calculate the end-effector's position and validate the trajectory. The gripper is controlled to pick and release the payload at the appropriate points, while intermediate poses ensure smooth and collision-free motion. Error checks and small pauses between movements help maintain stability and precision throughout the task.

##### The Numerical Inverse Kinematics Code
This code calculates the inverse kinematics (IK) for a PX100 robotic arm, determining the joint configurations needed to achieve a desired end-effector pose (pick_pose). It uses MATLAB's inverseKinematics solver with the BFGS Gradient Projection algorithm and configures it for higher accuracy by increasing the maximum iterations and random restarts, while tightening tolerance values for stricter convergence. The weights prioritize position accuracy over orientation, and the robot's home configuration is used as the initial guess. The function checks if the computed joint positions fall within predefined limits for each joint and validates the solver's success before returning the joint configuration or reporting an error.

##### The Trajectory Planning Code
The trajectory planning is divided into two phases: (1) moving from the current configuration to an intermediate hovering position, and (2) moving from the intermediate position to the target configuration. The number of waypoints is split proportionally between these phases, and linear interpolation is applied for smooth transitions in joint values. Depending on whether the robot is picking or placing an object (is_picking), the trajectory maintains the Z-coordinate above a specified min_z_thresh to avoid collisions or maintain safe operation. The final waypoint is explicitly set to match the target configuration to ensure precision. This approach ensures a smooth and safe motion trajectory for the manipulator.

##### The obstacle Avoidance Code
The obstacle was placed in the center of the configuration between the pick and drop positions. Its height was also double that of the object. To avoid the obstacle, we made the arm to go over the obstacle.<br>
To make the arm go over the obstacle, we took in the obstacle’s position and we already had its radius and height. As we were already using extra intermediate waypoints to make the arm hover before picking and dropping the object, we decided to use the intermediate waypoints approach for obstacle avoidance too. We already had three intermediate waypoints: 1) To hover over the object before picking it up, 2) To pick up the object, reach the x and y position of the drop pose and hover over it, 3) After dropping the object, hover again at the same position it hovered before placing followed by going back to home configuration. So, we added a fourth intermediate point where after picking up the object, instead of directly going to the hover position over the drop pose, it would first go about in height (Z axis) at the pick position itself, and then go to the drop point’s hover position. We also used the cylinder's height and radius to set the intermediate point’s x, y and z coordinates to ensure that it won’t collide into the obstacle.


### Some information regarding what each code file does




### Results



### Acknowledgements
We would like to thank Prof. Yasin Yazicioglu as well as all the course TAs for guiding us and solving all of our queries not only during this project, but throughout the entire course too. We would also like to thank Northeastern University for providing us access to the PincherX100 arm. 


### Contributors
- [Aryaman Shardul](https://github.com/Aryaman22102002)
- [Sairam Sridharan](https://github.com/Sairamzz)

### Contact
- Aryaman Shardul - lnu.arya@northeastern.edu
- Sairam Sridharan - sridharan.sai@northeastern.edu
