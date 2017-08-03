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
[image2]: ./images/walkthrough_diagram.png
[image3]: ./misc_images/misc3.png

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

*DH Parameter Table*
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

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
![alt text][image2]
And here's another image!


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


0
And just for fun, another example image:
![alt text][image3]
