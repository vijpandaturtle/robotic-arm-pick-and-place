## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./images/zero_config.png
[image2]: ./images/walthrough_diagram.png
[image3]: ./images/matrix.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The forward kinematics code was fairly easy to implementation and involved filling in the values in the DH parameter table. The urdf file consisting of the position of each joint relative to it's preceeding joint so the parameters were very intuitive.
In order to calculate the DH parameters I used the sketch of the arm in it's zero configuration.

![alt text][image1]



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

***DH Parameter Table***
---

| Joint  | ∝(i-1) | a(i-1) | θ(i) | d(i)  |
|---|---|---|---|---|
| 1  | 0 | 0 | θ1  | 0.75  |
| 2 | -pi/2 | 0.35 | θ2 - pi/2 | 0  |
| 3 | 0 | 1.25 |  θ3  | 0 |
| 4 | -pi/2 | - 0.054  | θ4  | 1.50  |
| 5 | pi/2   |  0 | θ5  | 0 |
| 6 | -pi/2  |  0|  θ6 | 0  |
| Gripper Frame (End-effector) | 0 | 0 | 0  | 0.303  |

This is the homogeneous transform matrix that I used to perform operations for forward kinematics i.e to transform from the base frame to the gripper frame. This equation is the matrix as per the dh parameter convention

![alt text][image3]

The following matrix is obtained by peforming alternate rotations and translations about the dh parameters **alpha**, **a**, **theta** and **d** respectively. Now to obtain transforms between consecutive frames, I simply substituted corresponding values from the dh parameter table.

These are the individual transform matrices based on the DH parameter table.

```python
T0_1 = [[cos(q1), -sin(q1), 0, 0],
        [sin(q1), cos(q1), 0, 0],
        [0, 0, 1, 0.750000000000000],
        [0, 0, 0, 1]]

T1_2 = [[sin(q2), cos(q2), 0, 0.350000000000000],
        [0, 0, 1, 0],
        [cos(q2), -sin(q2), 0, 0],
        [0, 0, 0, 1]]

T2_3 = [[cos(q3), -sin(q3), 0, 1.25000000000000],
        [sin(q3), cos(q3), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]

T3_4 = [[cos(q4), -sin(q4), 0, -0.0540000000000000],
        [0, 0, 1, 1.50000000000000],
        [-sin(q4), -cos(q4), 0, 0],
        [0, 0, 0, 1]]

T4_5 = [[cos(q5), -sin(q5), 0, 0],
        [0, 0, -1, 0],
        [sin(q5), cos(q5), 0, 0],
        [0, 0, 0, 1]]

T5_6 = [[cos(q6), -sin(q6), 0, 0],
        [0, 0, 1, 0],
        [-sin(q6), -cos(q6), 0, 0],
        [0, 0, 0, 1]]

T6_G = [[1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.303000000000000],
        [0, 0, 0, 1]]
```

To obtain the total transforms from the base_frame to the gripper_frame, is simply a product of all the consecutive transform matrices, in the reverse order. For this purpose I used sympy library which has an inbuilt class available for matrix operations.

```python
T0_G = simplify(T6_G*T5_6*T4_5*T3_4*T3_4*T2_3*T1_2*T0_1)
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

This was the hardest part of the project for me. I took a lot of time to visualize the thetas especially theta2 and theta3. These thetas are represented as q1,q2,q3.. in the DH parameter table. Here is a diagram that aided me in visualization.

![alt text][image2]

The axes represented in this image correspond to the sqrt(x^2 + y^2) and z axes. The triangle between joints 2, 3 and 5 can be evaluated by projecting it onto this frame. As per these axes, and using the diagram provided in section 1 for reference side_a and side_c can be easily found. As for side_b, the given co-ordinates are srqt(Wx^2 + Wy^2) and Wz. So the formula is something like this :

```python
side_b = sqrt(pow(sqrt(Wx**2 + Wy**2) - 0.35,2) + pow((Wz - 0.75),2))
```
The terms 0.35 and 0.75 are the distance to be subtracted because the co-ordinates of wrist are calculated with respect to the base frame.
Finding the angles of the triangle is easy, with the use of the cosine rule. Acoording to the above parameters, I have calculated values of thetas 1-3. For deriving theta 4-6, I used equations provided in the euler angles of rotation matrix section. Below, are the equations for thetas1-6

```python
# Angles for end-effector position
theta1 = atan2(Wy, Wx)
theta2 = pi/2 - angle_a - atan2((Wz - 0.75), sqrt(Wx**2 + Wy**2) - 0.35)
theta3 = pi/2 - angle_b + 0.036
# Angles for end-effector orientation
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]) - R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```
Note : The thetas are labelled as q1, q2, q3... in the IK_server script.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

The inverse kinematics code was fairly easy once I was able to visualize the sides of the triangle formed by joints 2, 3 and 5. There are the x and y co-ordinates and also correction terms to adjust the position of the frame because the wrist center is calculated with respect to the base frame. A brief explanation of my analysis of the IK problem can be found under rubric section 3.
Although, the angles are calculated accurately, there is room for more improvement, because I have not provided multiple solutions for each angle. Doing this will improve my solution of the IK problem.
