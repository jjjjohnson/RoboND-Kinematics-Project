## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.jpg
 

---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of the gazebo after `roslaunch kr210_claw_moveit demo.launch`.

![alt text][image1]

From the kr210.urdf.xacro file, I can get the information about the links(basic line, link1-6, grabber link), i.e. their orientation and position. I can also obtain the joint position and its axis in the global coordinate, which allow me derive the DH table.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

i | alpha<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | theta<sub>i-1</sub>
--- | --- | --- | --- | ---
0 | 0 | 0 | 0.75 | q1
1 | -pi/2 | 0.35 | 0 | q2-pi/2
2 | 0 | 1.25 | 0 | q3
3 | -pi/2 | -0.054 | 1.5 | q4
4 | -pi/2 | 0 | 0 |q5
5 | -pi/2 | 0 | 0 |q6
6 | 0| 0  | 0.303 |0

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
I divide the joint angles into two groups: theta1,2,3(position of weist center(WC)) and theta3,4,5(orientation of grapper). 
First, I need to know the position of WC:

```
            # Calculate waist center
            lx = cos(row) * cos(pitch)
            ly = sin(row) * cos(pitch)
            lz = -sin(pitch)

            wx = px - l_end_effector * lx
            wy = py - l_end_effector * ly
            wz = pz - l_end_effector * lz
```
where lx,ly and lz are the end-effector orientation along X axes of the local coordinate frame. and l_end_effector = link6 + gripper link.
From the image blow, we can see that `theta2 = pi/2 - (a - b)`, `theta3 = pi - c`.

```
a = acos((1.25^2 + l1^2 - 1.5^2)/(2*1.25*l1))
b = atan2(wcz-0.75, l2)
c = acos((1.25^2 + 1.5^2 - l1^2)/(2*1.25*1.5))
```

![alt text][image2]

After getting theta1,2,3, we can them fill into R0_3, calculate the inverse by calculating the transpose then calculate R3_6 by substituting roll, pitch and yall. Since the  roll, pitch and yaw are extrinsic, we can derive `Rrpy = simplify(R_z*R_y*R_x)`, where R_z is rotation by z(yaw), R_y is rotation by y(pitch) and R_x is rotation by x(roll).

```
R0_3 = T0_3[0:3, 0:3]
R0_3_inv = Transpose(R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3}))
R3_6 = R0_3_inv * Rrpy.subs({roll:r, pitch:p, yaw:p})
```
R36 can also be obtained by T36[:3,:3] and they should match with every element calculated above.

```
R3_6 = Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
[sin(q5)*cos(q6), -sin(q5)*sin(q6),   cos(q5)],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
```

Based on the above corresponding equal relation, we can derive theta4,5,6:

```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = acos(R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


I have filled in code based on the analysis above and when I was testing it, I found the robot arm could reach the desired position but the gripper is too loose to pick up the object, which I assume is beyond the IK_server.py. I would like to fix the bug if I know how to adjust the gripper joint.
There are however one concern: I used `acos()` to calculate some joint angle. Since the cosin of same absolute value of positive or negative angle appears the same, the code just pick the positive one. If we really have to figure out which one to use, since we have multiple joints and each joint choice will affect other joints' angle. it makes the choice of angles more complicated. I think the thing we need to concern is the joint constrain. I would like to improve it more since I code just do the job naively not very smartly...




