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
import pprint as pp
from spatialmath.base import trnorm

# import configuration_selection as cs
import Analytic_IK as aik
from spatialmath import *
from configuration_selection import *

# cartesian_pose = [
#     [-0.464, 0.436, 0.102, 0.0000, 0.0008, -1.0000, -0.0008],
#     [0.2184, 0.556, 0.10, 0.85355339, 0.35355339, 0.35355339, 0.14644661],
#     [0.156, 0.331, 0.222, 0.70105738, 0.43045933, 0.09229596, 0.56098553],
#     [0.284, -0.456, 0.52, 0.8365163 ,  0.22414387, -0.48296291, -0.12940952],
#     [-0.3, -0.526, 0.202, 0.49240388,  0.85286853, -0.15038373, -0.08682409]]


__SHOW_ENV__ = True
UR5 = rtb.models.UR5()

print(UR5.links)
env = swift.Swift()

def plot_markers(env, poses):
    if __SHOW_ENV__:
        """Plot a list of poses as markers in the environment"""
        for pose in poses:
            marker = sg.Axes(0.1, pose=cs.q_to_hom(pose))
            marker.pose = cs.q_to_hom(pose)
            env.add(marker)

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
    #print(cartesian_pose[0][:])
    #print(IK_solve(cartesian_pose[1][:]))
    #TCP = [0.2184, 0.556, 0.10, 0.85355339, 0.35355339, 0.35355339, 0.14644661]

    # solutions = inversK.IK(cartesian_pose[0][:], UR5e)
    # for i in range(len(solutions)):
    #     print(solutions[i]) 
    # for sol in solutions:
    #     print("solution: ", sol)
    #     input('Press enter to continue')

    #     move_robot(env, sol)

    cartesian_poses = [
        [-0.464, -0.436, 0.202, 0.0000, 0.0008, -1.0000, -0.0008], #TCP pose provided by Aljaz at the Robotics Elite Summer course
        [-0.464, 0.436, 0.102, 0.0000, 0.0008, -1.0000, -0.0008],
        [0.2184, 0.556, 0.10, 0.85355339, 0.35355339, 0.35355339, 0.14644661],
        [0.156, 0.331, 0.222, 0.70105738, 0.43045933, 0.09229596, 0.56098553],
        [0.284, -0.456, 0.52, 0.8365163, 0.22414387, -0.48296291, -0.12940952],
        [-0.3, -0.526, 0.202, 0.49240388, 0.85286853, -0.15038373, -0.08682409],
    ]
    # n = 0
    for i in cartesian_poses:
        Q = UnitQuaternion(i[3:])
        print("Q: ", Q)
        HT = SE3(i[:3]) * Q.SE3()
        print("teeeeeest: ",HT)
        # HT = SE3()
        result = aik.IK(HT, robot=rtb.models.DH.UR5())
        np.set_printoptions(precision = 5)
        print("Test the cart pose:", i, "\n", "Result is:")
        print("    theta1", "  theta2", " theta3", "   theta4", "  theta5", " theta6\n", result, "\n")
        # n = n + 1
        print("The number of solutions is: ", len(result))
        for sol in result:
            for i in range(len(sol)):
                if sol[i] > np.pi:
                    sol[i] = sol[i] - 2 * np.pi
                elif sol[i] < -np.pi:
                    sol[i] = sol[i] + 2 * np.pi
            sol = center_joints_around0(sol)
            print("solution: ", sol)
            move_robot(env, sol)
            input('Press enter to continue')


    # solutions[0][0]=solutions[0][0]+np.pi/2

if __name__ == "__main__":
    main()