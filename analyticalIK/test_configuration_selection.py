import numpy as np

from cmath import asin
from distutils.command.sdist import sdist
from email.headerregistry import HeaderRegistry
import math
from re import T
import numpy as np
import spatialmath as sm
from numpy import arctan2 as atan2
from numpy import sin as sin
from numpy import cos as cos
from numpy import arcsin as asin
from numpy import arccos as acos
from numpy import linalg
from spatialmath.base import q2r

from shutil import move
from webbrowser import Chrome
import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from spatialmath import SE3
from spatialmath import SO3
import swift
import spatialgeometry as sg
from spatialmath.base import q2r
from spatialmath.base import r2q
from spatialmath.base import eul2r
import pprint as pp
from spatialmath.base import trnorm
# from scipy.spatial.transform import Rotation  
from transforms3d.axangles import axangle2mat
from transforms3d.euler import euler2mat
from transforms3d.affines import compose

from configuration_selection import *
import Analytic_IK as aik
from spatialmath import *

import csv

# cartesian_pose = [
#     [-0.464, 0.436, 0.102, 0.0000, 0.0008, -1.0000, -0.0008],
#     [0.2184, 0.556, 0.10, 0.85355339, 0.35355339, 0.35355339, 0.14644661],
#     [0.156, 0.331, 0.222, 0.70105738, 0.43045933, 0.09229596, 0.56098553],
#     [0.284, -0.456, 0.52, 0.8365163 ,  0.22414387, -0.48296291, -0.12940952],
#     [-0.3, -0.526, 0.202, 0.49240388,  0.85286853, -0.15038373, -0.08682409]]

tcp_offset = [0.032, 0, 0.045, 0, -math.pi/2, 0] # x,y,z,rx,ry,rz

# def dh_to_matrix(theta, d, a, alpha, joint_type):
#     if joint_type == 0:
#         # Revolute joint
#         R = np.concatenate((axangle2mat([0, 0, 1], theta),[[0,0,0]]), axis=0)
#         print("R: ",R)
#         D = np.array([a, -sin(alpha)*d, cos(alpha)*d, 1]).T
#     elif joint_type == 1:
#         # Prismatic joint
#         R = euler2mat(alpha, 0, 0)
#         D = np.array([cos(theta)*a, sin(theta)*a, d, 1])
#     else:
#         raise ValueError("Invalid joint type")
#     # print("R: ",R)
#     print("D: ",D)
#     H = compose(R, D)
#     return H

def dh_to_matrix(theta, d, a, alpha, joint_type):
    if joint_type == 0:
        # Revolute joint
        R = axangle2mat([0, 0, 1], theta)
        D = [a, -sin(alpha)*d, cos(alpha)*d]
    elif joint_type == 1:
        # Prismatic joint
        R = euler2mat(alpha, 0, 0)
        D = [cos(theta)*a, sin(theta)*a, d]
    else:
        raise ValueError("Invalid joint type")
    H = compose(R, D)
    return H

# tcp_trans_vector = np.concatenate(([tcp_offset[0:3]],[[1]]),axis=1).T
# tcp_rot_matrix = np.concatenate((eul2r(tcp_offset[3:6]),[[0,0,0]]),axis=0)
# print ("rot: ",tcp_rot_matrix)
# tcp_homogenous_mat = np.concatenate((tcp_rot_matrix,tcp_trans_vector),axis=1)
# last_link_homogenous_mat = dh_to_matrix(tmp_last_link.theta,tmp_last_link.d,tmp_last_link.a,tmp_last_link.alpha,tmp_last_link.sigma)
# print("last link: ",last_link_homogenous_mat)


UR5_default =rtb.DHRobot([
rtb.RevoluteDH(d=0.089159, alpha=np.pi / 2),
rtb.RevoluteDH(a=-0.425),
rtb.RevoluteDH(a=-0.39225),
rtb.RevoluteDH(d=0.10915, alpha=np.pi / 2),
rtb.RevoluteDH(d=0.09465, alpha=-np.pi / 2),
rtb.RevoluteDH(d=0.0823)],name="UR5_default")#!UPDATE LAST TO TCP


__SHOW_ENV__ = True
UR5 = rtb.models.UR5()
env = swift.Swift()


# print(UR5.links)

# def plot_markers(env, poses):
#     if __SHOW_ENV__:
#         """Plot a list of poses as markers in the environment"""
#         for pose in poses:
#             marker = sg.Axes(0.1, pose=cs.q_to_hom(pose))
#             marker.pose = cs.q_to_hom(pose)
#             env.add(marker)

def move_robot(env, pos):
    if __SHOW_ENV__:
        env.step()
        UR5.q = pos
        env.step()


initial_config = [-np.pi / 2, -np.pi /
                  2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]

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
    # xyz abcd (Quaternion)

    # TCP = [-0.464, -0.436, 0.202, 0.0000, 0.0008, -1.0000, -0.0008]
    # print(cartesian_pose[0][:])
    # print(IK_solve(cartesian_pose[1][:]))
    #TCP = [0.2184, 0.556, 0.10, 0.85355339, 0.35355339, 0.35355339, 0.14644661]

    # solutions = inversK.IK(cartesian_pose[0][:], UR5e)
    # for i in range(len(solutions)):
    #     print(solutions[i])
    # for sol in solutions:
    #     print("solution: ", sol)
    #     input('Press enter to continue')

    #     move_robot(env, sol)

    cartesian_poses = [
        [0.3, 0.0, 0.1438276615812609, 1.4586879513659083, -2.750319113248153, 0.09466671588524875],
        [0.2427050983124842, 0.17633557568774194, 0.1438276615812609,1.4586879513659083, -2.750319113248153, 0.09466671588524875],
        [0.09270509831248423, 0.285316954888546, 0.1438276615812609,1.4586879513659083, -2.750319113248153, 0.09466671588524875],
        [-0.0927050983124842, 0.2853169548885461, 0.1438276615812609,1.4586879513659083, -2.750319113248153, 0.09466671588524875],
        [-0.24270509831248419, 0.17633557568774197, 0.1438276615812609,1.4586879513659083, -2.750319113248153, 0.09466671588524875],
        [-0.3, 0.0, 0.1438276615812609,1.4586879513659083, -2.750319113248153, 0.09466671588524875],
        [-0.24270509831248427, -0.1763355756877419, 0.1438276615812609,1.4586879513659083, -2.750319113248153, 0.09466671588524875],
        [-0.09270509831248426, -0.285316954888546, 0.1438276615812609,1.4586879513659083, -2.750319113248153, 0.09466671588524875],
        [0.09270509831248416, -0.2853169548885461, 0.1438276615812609,1.4586879513659083, -2.750319113248153, 0.09466671588524875],
        [0.24270509831248419, -0.176335575687742, 0.1438276615812609,1.4586879513659083, -2.750319113248153, 0.09466671588524875]
    ]
    #read cartesian poses from file

    # n = 0
    for i in cartesian_poses:
        print("Test the cart pose:", i, "\n")
        print(i[3:])
        r=eul2r(i[3:])
        quat = r2q(r)
        Q = UnitQuaternion(quat)
        HT = SE3(i[:3]) * Q.SE3()
        # HT = SE3()
        #         dh_param = np.array([
        # #ai-1,        alphai-1,           di,        thetai
        # [0,             0,              0.1625,          0],
        # [0,             np.pi/2,           0,            0],
        # [-0.425,        0,                 0,            0],
        # [-0.3922,       0,               0.1333,         0],
        # [0,             np.pi/2,         0.0997,         0],
        # [0,            -np.pi/2,         0.0996,         0]])
        # rmodel = rtb.models.DH.UR5()
        # rmodel.DH

        result = aik.IK(HT, robot=UR5_default)
        np.set_printoptions(precision=5)
        print("Test the cart pose:", i, "\n", "Result is:")
        print("    theta1", "  theta2", " theta3", "   theta4",
              "  theta5", " theta6\n", result, "\n")
        # n = n + 1
        print("The number of solutions is: ", len(result))
        #!add += math.pi/2 to the first joint
        # for i in range(len(result)):
        #     result[i][0] = result[i][0] + np.pi / 2
        # sols = get_best_sols(poses=result, object_pose=[-0.25, 0.1, 0.1438276615812609])
        # sol = check_within_limits(result, [[0, -pi/2], elbow_up, [-2.6, 0], [-pi, -pi/2], [-pi, pi], [-6.1, 6,1]])
        # sol = get_elbow_up(result)
        sols = center_joints_around0(result)
        print("The best solution is: ", sols)
        # sols = get_elbow_up(sols)
        sols = get_best_sols(poses=sols, object_pose=[0.01, 0.01])
        print("The best solution is: ", sols)
        if sols is None:
            print("No solution")
            input('Press enter to continue')
            continue
        
        print("The best solution is: ", sols)
        print("Shoulder is: ", shoulder_left_or_right(sols))
        move_robot(env, sols)
        input('Press enter to continue')
        # if len(sols) == 1:
        #     print("solution: ", sols)
        #     print("Shoulder is: ", shoulder_left_or_right(sols))
        #     move_robot(env, sols)
        #     input('Press enter to continue')
        #     continue
        # for solution in sols:
        #     print("solution: ", solution)
        #     print("Shoulder is: ", shoulder_left_or_right(solution))
        #     move_robot(env, solution)
        #     input('Press enter to continue')
        # for sol in result.transpose():
        #     print("solution: ", sol)
        #     move_robot(env, sol)
        #     input('Press enter to continue')

    # solutions[0][0]=solutions[0][0]+np.pi/2


if __name__ == "__main__":
    main()
