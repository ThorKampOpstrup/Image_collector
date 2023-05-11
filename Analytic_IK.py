import math

import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from spatialmath import SE3
from spatialmath import SO3
from spatialmath import *
from spatialmath import twist
import swift
import spatialgeometry as sg

#####LINK NAMES#####
# 0: base_link
# 1: shoulder_link
# 2: upper_arm_link
# 3: forearm_link
# 4: wrist_1_link
# 5: wrist_2_link
# 6: wrist_3_link
# 7: tool0
####################

# # Universal Robot UR5e kiematics parameters

# UR5e = rtb.DHRobot([
#     rtb.RevoluteDH(d=0.1625, alpha=np.pi / 2),
#     rtb.RevoluteDH(a=-0.425),
#     rtb.RevoluteDH(a=-0.3922),
#     rtb.RevoluteDH(d=0.1333, alpha=np.pi / 2),
#     rtb.RevoluteDH(d=0.0997, alpha=-np.pi / 2),
#     rtb.RevoluteDH(d=0.0996)
# ], name="UR5e")

UR5 = rtb.models.DH.UR5()

def forward_kin(robot, start=0, end=6):
    T = SE3(0, 0, 0)
    theta = robot.theta
    for i in range(start, end):
        T_new = SE3(robot[i].a, 0, robot[i].d) * SE3.Rz(theta[i]) * SE3.Rx(robot[i].alpha)
        T = T * T_new
    print("T: ", T)
    return T

def fk(theta, joint_from = 0, joint_to = 6, robot=rtb.models.DH.UR5()):
    T = SE3(0, 0, 0)
    # print("name: ", robot.name)
    for i in range(joint_from, joint_to):
        # print("i: ", i, "theta[i]: ", theta[i], "robot[i].a: ", robot[i].a, "robot[i].d: ", robot[i].d, "alpha[i]: ", robot[i].alpha)
        T_new = SE3(robot[i].a, 0, robot[i].d) * SE3.Rz(theta[i]) * SE3.Rx(robot[i].alpha)
        T = T * T_new
    return T

def IK(T, robot=rtb.models.DH.UR5(), fkine_robot=rtb.models.UR5()):
    theta = np.zeros(shape=(6, 8))
    v = SE3.Rx(0) * [0, 0, -robot[5].d]
    P05 = T * v
    # print(theta)

    # Theta1
    theta1p = np.arctan2(P05[1], P05[0]) + np.arccos(
        robot[3].d / (np.sqrt(pow(P05[0], 2) + pow(P05[1], 2)))) + np.pi / 2
    theta1n = np.arctan2(P05[1], P05[0]) - np.arccos(
        robot[3].d / (np.sqrt(pow(P05[0], 2) + pow(P05[1], 2)))) + np.pi / 2
    theta[0][0:4] = theta1p
    theta[0][4:8] = theta1n

    

    # Theta5
    P06 = T * [0, 0, 0]
    d4, d6 = robot.d[3], robot.d[5]
    theta[4][0:2] = np.arccos((P06[0] * np.sin(theta1p) - P06[1] * np.cos(theta1p) - d4) / d6)
    theta[4][2:4] = -np.arccos((P06[0] * np.sin(theta1p) - P06[1] * np.cos(theta1p) - d4) / d6)
    theta[4][4:6] = np.arccos((P06[0] * np.sin(theta1n) - P06[1] * np.cos(theta1n) - d4) / d6)
    theta[4][6:8] = -np.arccos((P06[0] * np.sin(theta1n) - P06[1] * np.cos(theta1n) - d4) / d6)

    # Theta6
    T60 = T.inv().A
    X60x, X60y = T60[0, 0], T60[1, 0]
    Y60x, Y60y = T60[1, 0], T60[1, 1]

    for i in range(8):
        Theta1 = theta[0][i]
        Theta5 = theta[4][i]
        if np.sin(Theta5) != 0:
            theta[5][i] = np.arctan2((-X60y * np.sin(Theta1) + Y60y * np.cos(Theta1)) / np.sin(Theta5),
                                    (X60x * np.sin(Theta1) - Y60x * np.cos(Theta1)) / np.sin(Theta5))
        else:
            theta[5][i] = 0

    # Theta3
    for i in range(0, 8, 2):
        # fkine_robot.q = theta[:, i]
        T06 = T
        T01 = fk(theta[:, i], 0, 1, robot = robot)
        # T01 = fkine_robot.fkine(theta[0], start='base_link', end='shoulder_link')
        print("T01: ", T01)
        # print("T01: ", T01)
        T16 = T01.inv()*T06
        T45 = fk(theta[:, i], 4, 5, robot=robot)
        # T45 = fkine_robot.fkine(theta[4], start='wrist_1_link', end='wrist_2_link')
        T56 = fk(theta[:, i], 5, 6, robot=robot)
        # T56 = fkine_robot.fkine(theta[5], start='wrist_2_link', end='tool0')
        T14 = T16*(T45*T56).inv()
        Px = T14.A[0][3]
        Pz = T14.A[2][3]
        a2 = robot.a[1]
        a3 = robot.a[2]
        P41length = np.sqrt(Px**2 + Pz**2)
        # if a2*a3==0:
        #     theta[2][i] = np.arccos((P41length**2 - a2**2 - a3**2)/(1))
        #     theta[2][i+1] = -np.arccos((P41length**2 - a2**2 - a3**2)/(1))
        # else:
        theta[2][i] = np.arccos((P41length**2 - a2**2 - a3**2)/(2*a2*a3))
        theta[2][i+1] = -np.arccos((P41length**2 - a2**2 - a3**2)/(2*a2*a3))
        #Theta 2
        theta[1][i] = np.arctan2(-Pz, -Px) - np.arcsin((-a3*np.sin(theta[2][i]))/P41length)
        theta[1][i+1] = np.arctan2(-Pz, -Px) - np.arcsin((-a3*np.sin(theta[2][i + 1]))/P41length)

    #Theta4
    for i in range(8):
        # fkine_robot.q = theta[:, i]
        T06 = T
        T01 = fk(theta[:, i], 0, 1,robot=robot)
        # T01 = fkine_robot.fkine(theta[0], start='base_link', end='shoulder_link')
        T16 = T01.inv()*T06
        T45 = fk(theta[:, i], 4, 5,robot=robot)
        # T45 = fkine_robot.fkine(theta[4], start='wrist_1_link', end='wrist_2_link')
        T56 = fk(theta[:, i], 5, 6, robot = robot)
        # T56 = fkine_robot.fkine(theta[5], start='wrist_2_link', end='tool0')
        T14 = T16*(T45*T56).inv()
        T12 = fk(theta[:, i], 1, 2,robot=robot)
        # T12 = fkine_robot.fkine(theta[1], start='shoulder_link', end='upper_arm_link')
        T23 = fk(theta[:, i], 2, 3,robot=robot)
        # T23 = fkine_robot.fkine(theta[2], start='upper_arm_link', end='forearm_link')
        T34 = T23.inv()*T12.inv()*T14
        theta[3][i] = np.arctan2(T34.A[1][0], T34.A[0][0])

    tmp_sols = []
    for sol in theta.transpose():
        sol_valid = True
        tmp_sol=[]
        for j in sol:
            if math.isnan(j):
                sol_valid = False
                break
            tmp_sol.append(j)
        if sol_valid:
            tmp_sols.append(tmp_sol)
        # print("WWWWWWWWWWWWWWsol",tmp_sol)
    
    return tmp_sols


