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

initial_config = [-np.pi / 2, -np.pi /
                   2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]


__SHOW_ENV__ = True
UR5 = rtb.models.UR5()

print(UR5.links)
env = swift.Swift()

cartesian_pose = [
    [-0.464, 0.436, 0.102, 0.0000, 0.0008, -1.0000, -0.0008],
    [0.2184, 0.556, 0.10, 0.85355339, 0.35355339, 0.35355339, 0.14644661],
    [0.156, 0.331, 0.222, 0.70105738, 0.43045933, 0.09229596, 0.56098553],
    [0.284, -0.456, 0.52, 0.8365163 ,  0.22414387, -0.48296291, -0.12940952],
    [-0.3, -0.526, 0.202, 0.49240388,  0.85286853, -0.15038373, -0.08682409]]

def q_to_hom(q):
    trans_vector = np.concatenate(([q[0:3]],[[1]]),axis=1).T
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

def move_robot(env, pos):
    if __SHOW_ENV__:
        env.step()
        UR5.q = pos
        env.step()


if __SHOW_ENV__:
    # init environtment
    env.launch(realtime=True, browser="Google-chrome")
    # add robot and marker to the environement
    env.add(UR5)
    plot_markers(env, cartesian_pose)
    move_robot(env, initial_config)

# DHRobot([
#     rtb.RevoluteDH(d=0.1625, alpha=np.pi / 2),
#     rtb.RevoluteDH(a=-0.425),
#     rtb.RevoluteDH(a=-0.3922),
#     rtb.RevoluteDH(d=0.1333, alpha=np.pi / 2),
#     rtb.RevoluteDH(d=0.0997, alpha=-np.pi / 2),
#     rtb.RevoluteDH(d=0.0996)
# ],name="UR5e")

dh_param = np.array([
    #ai-1,        alphai-1,           di,        thetai
    [0,             0,              0.1625,          0],
    [0,             np.pi/2,           0,            0],
    [-0.425,        0,                 0,            0],
    [-0.3922,       0,               0.1333,         0],
    [0,             np.pi/2,         0.0997,         0],
    [0,            -np.pi/2,         0.0996,         0]])



def TransFunc(i,angles):
    #T^i-1_i
    a = dh_param[:,0]
    alpha = dh_param[:,1]
    d = dh_param[:,2]
    theta = angles
    return [
    [       cos(theta[i]),              -sin(theta[i]),             0,                  a[i]        ],
    [sin(theta[i])*cos(alpha[i]), cos(theta[i])*cos(alpha[i]), -sin(alpha[i]), -sin(alpha[i])*d[i]  ],
    [sin(theta[i])*sin(alpha[i]), cos(theta[i])*sin(alpha[i]), cos(alpha[i]),   cos(alpha[i])*d[i]  ],
    [            0,                          0,                       0,                  1         ]]

#P06, Origin of frame 6 seen from frame 0
def theta1(angles,P05x, P05y): #two solutions
    d4 = dh_param[3][2]
    sol1 = angles.copy()
    sol2 = angles.copy()
    sol1[0] = atan2(P05y, P05x) + acos((d4)/math.sqrt(P05x**2 + P05y**2))+math.pi/2
    sol2[0] = atan2(P05y, P05x) - acos((d4)/math.sqrt(P05x**2 + P05y**2))+math.pi/2
    return [sol1, sol2]

def theta2(angles,P14x, P14z):
    theta3 = angles[2]
    a3 = dh_param[3][0]
    angles[1] = atan2(-P14z, -P14x) - asin((-a3*sin(theta3))/((np.linalg.norm(np.array([P14x, P14z]))))) 
    return angles
    

def theta3(angles,P14x, P14z):
    a2 = dh_param[2][0]
    a3 = dh_param[3][0]
    sol1 = angles.copy()
    sol2 = angles.copy()
    #print('acos', ((np.linalg.norm(np.array([P14x, P14z])))**2 - (a2)**2 - (a3)**2) / (2* a2 * a3))
    sol1[2] = acos(((np.linalg.norm(np.array([P14x, P14z])))**2 - (a2)**2 - (a3)**2) / (2* a2 * a3))
    sol2[2] = -acos(((np.linalg.norm(np.array([P14x, P14z])))**2 - (a2)**2 - (a3)**2) / (2* a2 * a3))

    #add error handling
    return [sol1, sol2]

def theta4(angles,X34y, X34x):
    #X34y is a unit vector giving the direction of the x-axis of frame 4 seen from 3
    angles[3] = atan2(X34y, X34x)
    return angles

def theta5(angles, P06x, P06y): #2 solution
    #P06x, P06y are the x and y coordinates of the origin of frame 6, seen from frame 0
    d4 = dh_param[3][2]
    d6 = dh_param[5][2]
    theta1 = angles[0]
    sol1 = angles.copy()
    sol2 = angles.copy()
    sol1[4] = acos((P06x*sin(theta1)-P06y*cos(theta1)-d4)/d6)
    sol2[4] = - acos((P06x*sin(theta1)-P06y*cos(theta1)-d4)/d6)
    return [sol1, sol2]

# Y06 is a unit vector giving the direction of the y axis of frame 6, seen from frame 0
def theta6(angles, X60y, Y60y, X60x, Y60x):
    #X60y unit vector giving the direction of the x axis of frame 0, seen from frame 6
    #Y60y unit vector giving the direction of the y axis of frame 0, seen from frame 6
    theta1 = angles[0]
    theta5 = angles[4]
    
    var1 = (-X60y*sin(theta1) + Y60y*cos(theta1)) / (sin(theta5))
    var2 = (X60x*sin(theta1) - Y60x*cos(theta1)) / (sin(theta5))
    angles[5] = atan2(var1,var2)
    return angles



def P05(T06):
    d6 = dh_param[5][2]
    out_p05 = np.array([0,0,-d6,1])
    c = np.matmul(T06, np.transpose(out_p05))
    return c

def P06(q):
    return [q[0], q[1], q[2]]




def IK_solve(q):
    
    initial_config = [0,0,0,0,0,0]

    # Precalcuations for angle1
    t06 = q_to_hom(q)   # Homogenous matrix of frame 6 seen from frame 0.
    
    #P05xy
    [P05x, P05y, _,_] = P05(t06)


    # Calculate angle1
    new_configurations= theta1(initial_config, P05x, P05y)
    #print('new_configurations theta 1', new_configurations)
    
    # Precalulations for angle5
    [P06x, P06y, _] = P06(q)

    # Calculate angle5

    old_configurations = new_configurations.copy()

    new_configurations = []

    for config in old_configurations:
        sol = theta5(config, P06x, P06y)
        new_configurations.append(sol[0])
        new_configurations.append(sol[1])
        
    #print('new_configurations with theta5', new_configurations)

    
    # Precalculations for angle6
    t60 = np.linalg.inv(t06) # Homogenous matrix of frame 0 seen from frame 6.

    X60y = t60[0][1]
    Y60y = t60[1][1]
    X60x = t60[0][0]
    Y60x = t60[1][0]

    # Calculate angle6

    old_configurations = new_configurations.copy()
    
    new_configurations = []
    for config in old_configurations:
        sol = theta6(config, X60y, Y60y, X60x, Y60x)
        new_configurations.append(sol)

    #print('new_configurations theta 6', new_configurations)

    
    # angle 3

    old_configurations = new_configurations.copy()
    new_configurations = []

    for config in old_configurations:
        # Precalculations for angle3

        ## P14x & z
        T01 = TransFunc(0,config)
        T45 = TransFunc(4,config)
        T56 = TransFunc(5,config)
        T14 = np.matmul(np.linalg.inv(T01), np.matmul(t06, np.linalg.inv(np.matmul(T45, T56))))
        P14x = T14[0][3]
        P14z = T14[2][3]


        #Calculate angle3
        sol = theta3(config, P14x, P14z)
        new_configurations.append(sol[0])
        new_configurations.append(sol[1])


    
    #print('new_configurations theta 3', new_configurations)


    
    # Precalculations for angle2

    # Calculate angle2
    #angle2 = theta2(P14z, P14x, theta3(P14x, P14z))
    old_configurations = new_configurations.copy()
    new_configurations = []

    for config in old_configurations:
        ## P14x & z
        T01 = TransFunc(0,config)
        T45 = TransFunc(4,config)
        T56 = TransFunc(5,config)
        T14 = np.matmul(np.linalg.inv(T01), np.matmul(t06, np.linalg.inv(np.matmul(T45, T56))))
        P14x = T14[0][3]
        P14z = T14[2][3]
        sol = theta2(config, P14x, P14z)
        new_configurations.append(sol)
        #print(sol)

    #print('new_configurations theta 2', new_configurations)
   
    


    # Angle4
    old_configurations = new_configurations.copy()
    new_configurations = []

    for config in old_configurations:
        # Precalculaitons for angle4
        #X34y & x
        T01 = TransFunc(0,config)
        T12 = TransFunc(1,config)
        T23 = TransFunc(2,config)
        T45 = TransFunc(4,config)
        T56 = TransFunc(5,config)
        T34 = np.matmul(np.linalg.inv(np.matmul(T01,np.matmul(T12, T23))), np.matmul(t06, np.linalg.inv(np.matmul(T45,T56))))
        X34y = T34[1,0]
        X34x = T34[0,0]

        # Calculate angle4
        sol = theta4(config, X34y, X34x)
        new_configurations.append(sol)
        #print(sol)
    
    return new_configurations
    #print('new_configurations theta 4', new_configurations)
   



def main():
    # xyz abcd (Quaternion)
   
    TCP = [-0.464, -0.436, 0.202, 0.0000, 0.0008, -1.0000, -0.0008]
    #print(cartesian_pose[0][:])
    #print(IK_solve(cartesian_pose[1][:]))
    #TCP = [0.2184, 0.556, 0.10, 0.85355339, 0.35355339, 0.35355339, 0.14644661]
   

    solutions = IK_solve(cartesian_pose[2][:])
    for i in range(len(solutions)):
        print(solutions[i]) 
    for sol in solutions:
      input('Press enter to continue')

      move_robot(env, sol)
    solutions[0][0]=solutions[0][0]+np.pi/2
    move_robot(env, solutions[0])
if __name__ == "__main__":
    main()