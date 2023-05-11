# import numpy as np
from math import *
import roboticstoolbox as rtb
from spatialmath import *
from numpy import *

joint1_limit_shoulder_left = [pi/2,pi]
joint1_limit_shoulder_right = [-pi/2, 0]
joint2_limits = [-pi, 0]#(elbow not below horizontal)
joint3_limits = [0, 0]#calculated on the go
joint4_limits = [0, 0]#calculated on the go
# joint5_limits_shoulder_left = [0, pi]
# joint5_limits_shoulder_right = [-pi, 0]
joint5_limit = [-pi/2,pi/2]
joint6_limits = [-6.1, 6.1]

##tmp limits
limits=[[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]


#[[max min](joint1), [max min](joint2), [max min](joint3), [max min](joint4), [max min](joint5), [max min](joint6)]
# shoulder_left = [[pi/2, pi], elbow_up, [0, 2.6], [-pi, -pi/2], [-pi, pi], [-6.1, 6,1]]
# shoulder_right = [[-pi/2, 0], elbow_up, [-2.6, 0], [-pi, -pi/2], [-pi, pi], [-6.1, 6,1]]
#!limits for joint 4 should be calculated
# shoulder_left = [joint1_limit_shoulder_left, joint2_limits, joint3_limits, joint4_limits, [0, pi], [-6.1, 6.1]]
# shoulder_right = [joint1_limit_shoulder_left, joint2_limits, joint3_limits, joint4_limits, [-pi, 0], [-6.1, 6.1]]

# def get_elbow_up(poses):
#     #Get elbow up solution
#     solutions = []
#     for pose in poses:
#         #if pose is in between elbow up limits
#         if pose[1] >= elbow_up[0] and pose[1] <= elbow_up[1]:
#             print("Elbow up solution found: ", pose)
#             solutions.append(pose)
#     print ("Elbow up solutions: ", solutions)
#     return solutions

def center_joint_around_0(value):
    # print("values: ", values)
    tmp_val = value
    # print("tmp_val: ", tmp_val)
    tmp_val = tmp_val%(2*pi)
    if tmp_val > pi:
        tmp_val -= 2*pi
    if tmp_val < -pi:
        tmp_val += 2*pi
    # print("tmp_val: ", tmp_val)
    return tmp_val

# def shoulder_left_or_right(pose):
#     #Check if shoulder is left or right
#     # print("pose: ", pose)
#     tmp = 
#     print("tmp: ", tmp)
#     if tmp >= 0:
#         return 'right'
#     else:
        # return 'left'

def calculate_joint3(pose):
    #Calculate joint3 limits (elbow bend up?)
    #check if shoulder is left or right
    #should be calculated based on the position of joint 1
    limits = [0, 0]
    shoulder_pose=shoulder_left_or_right(pose)
    print("shoulder_pose: ", shoulder_pose)
    if shoulder_pose=='right':
        limits[0]=-2.8; #streached
        limits[1]=0 #folded
    else:
        limits[0]=-2.8; #streached
        limits[1]=0 #streached
    limits=center_joints_around_0(limits)
    # print("limits joint3: ", limits[2])
    return limits

#dynamic check to se if j4 is within limit(horizontal and vertical)
def calculate_joint4(pose):
    # Calculate joint4 limits
    # check if shoulder is left or right
    # should be calculated based on the position of joint 2 and 3
    limits=[0,0]
    if (shoulder_left_or_right(pose)=='right'):
        vertical=-pose[1]-pose[2]
        vertical=center_joint_around_0(vertical)
        horizontal=vertical-(pi/2*1.1)#1.1 is a factor to allow the joint to be just below horizontal(joint below next joint)
        horizontal=center_joint_around_0(horizontal)
        # print("\nshoulder is left")
    else:
        #res = -90 deg horizontal when the arm is fully streached out
        vertical=-2*pi-pose[1]-pose[2]
        vertical=center_joint_around_0(vertical)
        horizontal=vertical+(pi/2*1.1) #1.1 is a factor to allow the joint to be just below horizontal(joint below next joint)
        horizontal=center_joint_around_0(horizontal)
        # print("\nshoulder is right")
    # print("vertical: ", vertical)
    limits=[min(horizontal, vertical), max(horizontal, vertical)]
    # limits=center_joints_around_0(limits)

    # print("limits joint4: ", limits)
    # print("limits joint4 after center: ", limits[3])
    return limits

def value_between(val, limits):
    return(val >= min(limits) and val <= max(limits))

def calculate_limits_and_check(poses, limits):
    #Check if all poses are within limits
    solutions = []
    # print("poses: ", poses)l
    limits[1]=joint2_limits
    cheking_order = [1,5]
    for pose in poses:
        joints_okay = True
        for i in cheking_order:
            if not value_between(pose[i], limits[i]):
                joints_okay = False
                break
        if joints_okay:
            solutions.append(pose)
    print("solutions: ", solutions)
    return solutions

def center_joints_around_0(sol):
    tmp_sol = sol  
    for i in range(len(tmp_sol)):
        # tmp_sol[j] = tmp_sol[j]%(2*pi)
        # if tmp_sol[j] > pi:
        #     tmp_sol[j] -= 2*pi
        # if tmp_sol[j] < -pi:
        #     tmp_sol[j] += 2*pi
        # print("j: ", tmp_sol[i])
        tmp_sol[i]=center_joint_around_0(tmp_sol[i])
        # print("j after: ", tmp_sol[i])
        # tmp_sols[i] = tmp_sol
    # print("tmp_sol: ", tmp_sol)
    return tmp_sol

def get_best_sols(poses, object_pose):
    # determine if shoulder is left or right should be used 
    # if tcp is on the left of the object compared to the base frame, use shoulder_left
    # if tcp is on the right of the object compared to the base frame, use shoulder_right
    for pose in poses:
        pose = center_joints_around_0(pose)
    # poses = center_joints_around_0(poses)
    # print("hellaposes: ", poses)

    obj_pose = object_pose[0:3]
    T = rtb.models.DH.UR5().fkine(poses[0]) # it doesn't matter what solution is used
    translation = T.t
    angle_to_obj = atan2(obj_pose[1], obj_pose[0]) ##!check the order of this one
    angle_to_tcp = atan2(translation[1], translation[0])

    if angle_to_obj > angle_to_tcp: #!check if this is correct might need to switch
        limits[0] = joint1_limit_shoulder_left
    else:
        limits[0] = joint1_limit_shoulder_left

    # limits[1]=joint2_limits

    # limits = shoulder_left #! should be changed to dynamic check based on the pose of the object, it might need to be checked in the end
    solutions_within_limits = calculate_limits_and_check(poses, limits)
    print("solutions_within_limits:\n")
    for sol in solutions_within_limits:
        print("sols within limits: ", sol)
    #all poses should be within limits but we get bot shoulder left and right solutions
    # print("solutions_within_limits:\n ", solutions_within_limits)
    if len(solutions_within_limits) == 0:
        return None
    if len(solutions_within_limits) == 1:
        return [solutions_within_limits[0]] #return the only solution in case only one is found we do not want to discard it
    return solutions_within_limits
    
    # tmp_poses=get_elbow_up(poses)


    


    """
    The elbow up or down should be based on the position joint 1 and 2
    """
