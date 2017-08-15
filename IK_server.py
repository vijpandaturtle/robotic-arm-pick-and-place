#!/usr/bin/env python

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# Define Modified DH Transformation matrix
def  transform(q , a, d, alpha):
     transform = Matrix([ [cos(q), -sin(q), 0, a], [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d], [sin(q)*sin(alpha), cos(q)*sin(alpha),
              cos(alpha), cos(alpha)*d], [0, 0, 0, 1] ])
     return transform


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        # Define DH param symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6= symbols('alpha0:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

        # Joint angle symbols
        # Modified DH params
        s = { d1 : 0.75, alpha0 : 0,     a0 : 0,
              d2 : 0,    alpha1 : -pi/2, a1 : 0.35, q2 : q2 - pi/2,
              d3 : 0,    alpha2 : 0,     a2 : 1.25,
              d4 : 1.50, alpha3 : -pi/2, a3 : -0.054,
              d5 : 0,    alpha4 : pi/2,  a4 : 0,
              d6 : 0,    alpha5 : -pi/2, a5 : 0,
              d7 : 0.303,alpha6 : 0,     a6 : 0, q7 : 0
             }

        # Create individual transformation matrices
        T0_1 = transform(q1, a0, d1, alpha0)
        T0_1 = T0_1.subs(s)
        T1_2 = transform(q2, a1, d2, alpha1)
        T1_2 = T1_2.subs(s)
        T2_3 = transform(q3, a2, d3, alpha2)
        T2_3 = T2_3.subs(s)
        T3_4 = transform(q4, a3, d4, alpha3)
        T3_4 = T1_2.subs(s)
        T4_5 = transform(q5, a4, d5, alpha4)
        T4_5 = T4_5.subs(s)
        T5_6 = transform(q6, a5, d6, alpha5)
        T5_6 = T5_6.subs(s)
        T6_G = transform(q7, a6, d7, alpha6)
        T6_G = T6_G.subs(s)

        # Correction to fix difference in orientation between gripper frame and dh parameter convention method
        R_z = Matrix([[cos(pi), -sin(pi), 0, 0],[sin(pi), cos(pi), 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
        R_y = Matrix([[cos(-pi/2), 0, sin(-pi/2), 0],[0, 1, 0, 0],[-sin(-pi/2), 0, cos(-pi/2), 0],[0, 0, 0, 1]])
        # Correction term
        R_correction = simplify(R_z*R_y)

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
            # Transformation to find end-effector position
            T0_G = simplify(T6_G*T5_6*T4_5*T3_4*T3_4*T2_3*T1_2*T0_1)
            # Corrected total transform
            T_total = simplify(T0_G * R_correction)
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method
            # Calculating positions of the wrist center
            end_effector_pos = Matrix([px, py, pz])
            R0_6 = Matrix(T0_G)
            R0_6.row_del(3)
            translate = R0_6.col(3)
            #print(translate)
            R0_6.col_del(3)
            #print(R0_6)
            wrist_center = simplify(end_effector_pos - R0_6*translate)
            #print(wrist_center_pos)
            Wx, Wy, Wz = wrist_center[0], wrist_center[1], wrist_center[2]

            side_a = 1.50
            side_b = sqrt(pow(sqrt(Wx**2 + Wy**2) - 0.35,2) + pow((Wz - 0.75),2))
            side_c = 1.25
            angle_a = acos((side_b**2 + side_c**2 - side_a**2)/(2*side_b*side_c))
            angle_b = acos((side_a**2 + side_c**2 - side_b**2)/(2*side_a*side_c))
            angle_c = acos((side_a**2 + side_b**2 + side_c**2)/(2*side_a*side_b))
            # Finding the first three joint angles using trigonometry
            q1 = atan2(Wy, Wx)
            q2 = pi/2 - angle_a - atan2((Wz - 0.75), sqrt(Wx**2 + Wy**2) - 0.35)
            q3 = pi/2 - angle_b + 0.036

            # Finding the last three joint angles
            R0_3 = T0_1[0:3]*T1_2[0:3]*T2_3[0:3]
            R0_3 = R0_3.subs(s)
            # Using the matrix containing last three transforms to calculate last three thetas
            R3_6 = R0_3.inv('LU')*R0_6

            q4 = atan2(R3_6[2,2], -R3_6[0,2])
            q5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]) - R3_6[1,2])
            q6 = atan2(-R3_6[1,1], R3_6[1,0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    	    joint_trajectory_point.positions = [q1, q2, q3, q4, q5, q6]
    	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
