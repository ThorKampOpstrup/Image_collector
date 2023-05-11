#!generate configuration in "simulation" from .csv file of poses

import numpy as np
import math
from re import T
import numpy as np
from numpy import arctan2 as atan2
from numpy import sin as sin
from numpy import cos as cos
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from scipy.spatial.transform import Rotation
from scipy.linalg import norm
import swift

from spatialmath.base import r2q
from spatialmath.base import eul2r

from transforms3d.axangles import axangle2mat
from transforms3d.quaternions import mat2quat
from transforms3d.euler import euler2mat
from transforms3d.affines import compose


import sys
# caution: path[0] is reserved for script path (or '' in REPL)
sys.path.insert(1, '../')

from configuration_selection import *
# import Analytic_IK as aik
from analyticalIK import *
from spatialmath import *
import pose_calculator as pc
import configuration_selection as cs
import spatialgeometry as sg
from spatialmath.base import q2r


import csv

# tcp_offset = [0.032, 0, 0.045, 0, -math.pi/2, 0] # x,y,z,rx,ry,rz
# object_pose = [0.35, 0.35, 0.0]
object_pose = [0.0, 0.35, 0.1]

def q_to_hom(q):
    trans_vector = np.concatenate(([q[0:3]],[[1]]),axis=1).T
    # print("trans_vector: ", trans_vector)
    rot_matrix = np.concatenate((q2r(q[3:7]),[[0,0,0]]),axis=0)
    homogenous_mat = np.concatenate((rot_matrix,trans_vector),axis=1)
    return homogenous_mat

def plot_markers(env, poses):
    if __SHOW_ENV__:
        """Plot a list of poses as markers in the environment"""
        for pose in poses:
            marker = sg.Axes(0.1, pose=q_to_hom(pose))
            marker.pose = q_to_hom(pose)
            env.add(marker)



# def dh_to_matrix(theta, d, a, alpha, joint_type):
#     if joint_type == 0:
#         # Revolute joint
#         R = axangle2mat([0, 0, 1], theta)
#         D = [a, -sin(alpha)*d, cos(alpha)*d]
#     elif joint_type == 1:
#         # Prismatic joint
#         R = euler2mat(alpha, 0, 0)
#         D = [cos(theta)*a, sin(theta)*a, d]
#     else:
#         raise ValueError("Invalid joint type")
#     H = compose(R, D)
#     return H

#!MIGHT BE WRONGEE
UR5_default =rtb.DHRobot([
rtb.RevoluteDH(d=0.089159),
rtb.RevoluteDH(a=-0.425, alpha=np.pi / 2),
rtb.RevoluteDH(a=-0.39225),
rtb.RevoluteDH(d=0.10915),
rtb.RevoluteDH(d=0.09465, alpha=np.pi / 2),
rtb.RevoluteDH(d=0.0823, alpha=-np.pi / 2)],name="UR5_default")

__SHOW_ENV__ = True
UR5 = rtb.models.UR5()
env = swift.Swift()

def move_robot(env, pos):
    if __SHOW_ENV__:
        env.step(0.0001)
        UR5.q = pos
        env.step(0.0001)


# initial_config = [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]
initial_config = [0, 0, 0, 0, 0, 0]


if __SHOW_ENV__:
    # init environtment
    env.launch(realtime=True, browser="Google-chrome")
    # add robot and marker to the environement
    env.add(UR5)
    # plot_markers(env, cartesian_pose)
    move_robot(env, initial_config)

# UR5e =rtb.DHRobot([
#     rtb.RevoluteDH(d=0.1625, alpha=np.pi / 2),
#     rtb.RevoluteDH(a=-0.425),
#     rtb.RevoluteDH(a=-0.3922),
#     rtb.RevoluteDH(d=0.1333, alpha=np.pi / 2),
#     rtb.RevoluteDH(d=0.0997, alpha=-np.pi / 2),
#     rtb.RevoluteDH(d=0.0996)
# ],name="UR5e")


def main():
    aik = analyticalIK()

    file = open("../test/end_poses.csv", "r")
    end_poses = []
    for line in file:
        line = line.strip()
        line = line.split(",")
        end_poses.append([float(i) for i in line])
    file.close()

    obj_marker = [-object_pose[0], -object_pose[1], object_pose[2], 0, 0, 0, 0]
    plot_markers(env, [obj_marker])

    # print("center test: ", cs.center_joint_around_0(math.pi+0.001))

    # for end_pose in end_poses:
    #     rotation_vector = end_pose[3:]
    #     print("rotation_vector: ", rotation_vector)
    #     print("norm: ", norm(rotation_vector))
    #     print("rotation_vector/norm: ", np.divide(rotation_vector,norm(rotation_vector)))
    #     rot = axangle2mat(np.divide(rotation_vector,norm(rotation_vector)), norm(rotation_vector))
    #     Q_array = mat2quat(rot)
    #     marker = np.array([-end_pose[0], -end_pose[1], end_pose[2], Q_array[0], Q_array[1], Q_array[2], Q_array[3]])
    #     plot_markers(env, [marker])

    
    # input("Press Enter to continue...")
    for end_pose in end_poses:
         # print("end_pose: ", end_pose)
        # Create a rotation object from Euler angles specifying axes of rotation
        # rot = Rotation.from_euler('xyz', end_pose[3:], degrees=True)
        # rotation_vec = [end_pose[4:6]] 
        # # print ("rotation_vec: ", rotation_vec)
        # rot = axangle2mat(end_pose[3:], 1)
        # # Convert to quaternions and print
        # Q_array = mat2quat(rot)
        rotation_vector = end_pose[3:]
        rot = axangle2mat(np.divide(rotation_vector,norm(rotation_vector)), norm(rotation_vector))
        Q_array = mat2quat(rot)
        pose_2_solve = np.array([end_pose[0], end_pose[1], end_pose[2], Q_array[0], Q_array[1], Q_array[2], Q_array[3]])
        pose2_plot = np.array([-end_pose[0], -end_pose[1], end_pose[2], Q_array[0], Q_array[1], Q_array[2], Q_array[3]])
        plot_markers(env, [pose2_plot])
        # HT = SE3(pose_2_solve[:3]) * UnitQuaternion(pose_2_solve[3:]).SE3(), 
        # print("Q: ", Q)
        Q = UnitQuaternion(Q_array)
        # print("Q: ", Q)
        HT = SE3(end_pose[:3]) * Q.SE3()
        # robot = rtb.models.DH.Puma560()
        # robot = rtb.models.DH.UR5()
        
        # cheat_solution = robot.ikine_a(HT, 'run')
        # config_validate


        # print("Q: ", Q.matrix)
        # marker = np.array([end_pose[0], end_pose[1], end_pose[2], Q_array[0], Q_array[1], Q_array[2], Q_array[3]])
        # print("marker: ", marker)
        # print("____________pose: ", marker[0:3])
        # plot_markers(env, [marker])
        # print("HT: ", HT)
        # q_sols = aik.IK(HT, robot=UR5_default)
        # pose_2_solve = [end_pose[:3], Q_array]
        print("pose_2_solve: ", pose_2_solve)
        q_sols = aik.solve(desired_tcp_pose=pose_2_solve)
        np.set_printoptions(precision = 5)
        print("Test the cart pose:", end_pose, "\n", "Result is:")
        print("    theta1", "  theta2", " theta3", "   theta4", "  theta5", " theta6\n")
        if q_sols ==[]:
            print("No solution found")
            continue
        for q_sol in q_sols:
            print([round(num, 2) for num in q_sol])
        
        # print("q_sols: ", q_sols)
        best_sol = cs.get_best_sols(poses=q_sols, object_pose=object_pose, robot=UR5)
        # print("best_sol: ", best_sol)
        # input("Press Enter to continue...")
        
        # if best_sol == None:
        #     print("No solution found")
        #     continue
        for link in UR5.links:
            print("\nlink: ", link)
        # best_sol = cs.center_joints_around_0(best_sol)
        # move_robot(env, best_sol)
        # pose = UR5.fkine(UR5.q, start='base', end='wrist_3_link')
        # print("best_sol: ", best_sol[0])
        # UR5_default.q = best_sol[0]
        # print("ur5 dh: ", UR5_default.q)
        # print("good sol: ",  UR5_default.config_validate(UR5_default.configs, ("r", "u", 'n')))
        if best_sol == []:
            continue
        print("best_sol: ", best_sol)
        
        for q_set in best_sol:
        # input("Press Enter to continue...")

            # q_set[0]=q_set[0]+np.pi
            # print("q_set before round: ", q_set)
            q_set = center_joints_around_0(q_set)
            print("q_set after round: ", q_set)
            move_robot(env, q_set)
            # print("ur5 dh: ", UR5_default)
            # print("q_set: ", UR5.q)
            # print("cartesian coordinate:\n", aik.fk(UR5.q, joint_from=0, joint_to=6, robot=UR5_default))
            #save pose from link 0 to link 3
            pose = UR5.fkine(UR5.q, start='base', end='wrist_3_link')
            print("\n\n")
            status = cs.calculate_joint4(UR5.q)
            # print("T: ", T)
            # pose = UR5.fkine(UR5.q, start='base', end='wrist_3_link')
            # print("rtb:\n", pose)
            # print ("self implemented:\n", aik.fk(q_set, joint_from=0, joint_to=6, robot=UR5_default))
            input("Press Enter to continue...")
            # break
            

if __name__ == "__main__":
    main()
