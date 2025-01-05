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



### Some information regarding what each code file does




### Results



### Acknowledgements
We would like to thank Prof. Yasin Yazicioglu as well as all the course TAs for guiding us and solving all of our queries not only during this project, but throughout the entire course too. We would also like to thank Northeastern University for providing us access to the PincherX100 arm. 


### Contributors
- Aryaman Shardul
- Sairam Sridharan

### Contact
