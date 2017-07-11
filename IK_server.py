#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:7')
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6= symbols('alpha0:7')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:6')
            
           # Joint angle symbols
           # Modified DH params
           q1, q2, q3, q4, q5, q6, q7 = 
           d1, d2, d3, d4, d5, d6, d7 = 
           alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = 0, -pi/2, 0, -pi/2, pi/2, -pi/2, 0
           a0, a1, a2, a3, a4, a5, a6 = 
            
             
            # Define Modified DH Transformation matrix
            function transform(q , d, a, alpha):
                 transform = Matrix([ [cos(q), -sin(q), 0, a], [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(q)*d], [sin(q)*sin(alpha), cos(q)*sin(alpha), 
                          cos(alpha), cos(alpha)*d], [0, 0, 0, 1] ])
                 return transform 


            # Create individual transformation matrices
            T0_1 = transform(q1, d1, a0, alpha0)
            T1_2 = transform(q2, d2, a1, alpha1)
            T2_3 = transform(q3, d3, a2, alpha2)
            T3_4 = transform(q4, d4, a3, alpha3)
            T4_5 = transform(q5, d5, a4, alpha4)
            T5_6 = transfrom(q6, d6, a5, alpha5)
            T6_G = transform(q7, d7, a6, alpha6)
            
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
            T0_G = simplify(T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G)
		


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
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
