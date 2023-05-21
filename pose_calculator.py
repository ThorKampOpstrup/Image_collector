import math
import os
import robot_class
import operator
from scipy.spatial.transform import Rotation as R
import numpy as np
import roboticstoolbox as rtb
import Analytic_IK as IK

UR5WithTCP =rtb.DHRobot([
            rtb.RevoluteDH(d=0.089159, alpha=np.pi / 2),
            rtb.RevoluteDH(a=-0.425),
            rtb.RevoluteDH(a=-0.39225),
            rtb.RevoluteDH(d=0.10915, alpha=np.pi / 2),
            rtb.RevoluteDH(d=0.09465, alpha=-np.pi / 2),
            rtb.RevoluteDH(d=0.0823)],name="UR5WithTCP")


def generate_positions_xyz(object_pose, angle, distance, n):
    # calculate uniformly distributed positions around object with angle and distance
    #! (!!!!!NOT THE TCP POSITIONS NO ROTATIONS!!!!!!)
    positions = []
    # loop through 0 to 2pi in n steps
    step_size =  2 * math.pi/n
    for i in range(n):
        tmp = i *step_size
        # print(tmp)
        x = object_pose[0] + (distance * math.cos(tmp))
        # print(x)
        y = object_pose[1] + (distance * math.sin(tmp))
        z = object_pose[2] + (distance * math.sin(angle))
        positions.append([x, y, z])
    return positions

def fibonacci_band_positions(object_pose, min_angle, max_angle, distance, n):

    points = []
    phi = math.pi * (math.sqrt(5) - 1)
    # phi = (1+math.sqrt(5))/2

    min_z = math.sin(min_angle)*distance
    max_z = math.sin(max_angle)*distance

    step_size_z = (max_z-min_z)/float(n)

    for i in range(n):
        z = min_z+(i*step_size_z)

        radius = math.sqrt(pow(float(distance),2)-pow(float(z),2))
        theta = phi*i

        x = math.cos(theta) * radius
        y = math.sin(theta) * radius

        points.append([x+object_pose[0],y+object_pose[1],z+object_pose[2]])

    return points


def hom2axan(hom):
    #calculate the angle axis from the homogenous matrix
    rotm = hom[0:3, 0:3]
    axan = R.from_matrix(rotm).as_rotvec()
    # print("axan: ",axan)
    return axan

def calculate_angle_axis(pos_obj, pos_tcp, rotation): #rot is in deg
    #take the first 3 elements of pos_obj and pos_tcp
    pos_obj= pos_obj[:3]
    # print("pos_obj",pos_obj)
    pos_tcp= pos_tcp[:3]
    # print("pos_tcp",pos_tcp)
    dir_vec = []
    for i in range(3):
        tmp = pos_tcp[i]-pos_obj[i]
        if tmp == 0:
            dir_vec.append(0.0000001)
        else:
            dir_vec.append(tmp)
    # print("dir_vec",dir_vec)


    tmp_dist = math.sqrt(dir_vec[0]**2 + dir_vec[1]**2+ dir_vec[2]**2)
    # # rot_v = math.atan(dir_vec[2]/tmp_dist)*-1 # The angle of the light to the object
    rot_v = math.acos(dir_vec[2]/tmp_dist)
    # print("rot_v: ",rot_v)
    rot_v = math.radians(90)+((math.pi/2)-rot_v)

    
    # print("dir_vec: ",dir_vec)
    rot_h = math.atan2(dir_vec[1], dir_vec[0])
    # print("rot_h: ",rot_h)
    # print("dir_vec:", dir_vec)

    rx = 0
    #ry = math.pi/4
    # ry = 0
    ry = -rot_v
    rz = rot_h
    # rz = -math.pi

    # print ("rz: ",rz)
    rot_to_allign = [rz, ry, rx]
    rotm_to_allign = R.from_euler('ZYX', rot_to_allign, degrees=False)

    # rotation = 45 #! rotation should not be set to 0
    rot_about_z = [math.radians(rotation), 0, 0]
    rotm_about_z = R.from_euler('ZYX', rot_about_z, degrees=False)

    rotm_combined = rotm_to_allign * rotm_about_z
    # # print(rotm_about_z.as_matrix())
    # print(rotm_to_allign.as_matrix())
    angle_axis = hom2axan(rotm_combined.as_matrix())
    # print("angle axis: ",angle_axis)
    # angle_axis = hom2axan(rotm_to_allign.as_matrix())
    # print ("angle axis:HHHHHHHHHH: ",angle_axis)
    return angle_axis

def eul2hom(pos, eul):
    tv = np.concatenate(([pos[0:3]],[[1]]),axis=1).T
    # print("tv: ",tv)
    rotm = R.from_euler('ZYX', eul)
    # print("rotm: ",rotm.as_matrix())
    rotm = np.concatenate((rotm.as_matrix(), [[0, 0, 0]]), axis=0)
    # print("rotm: ",rotm)
    hom = np.concatenate((rotm, tv), axis=1)
    # print("hom: ",hom)
    return hom

def axan2hom(pos, axan):
    tv = np.concatenate(([pos[0:3]],[[1]]),axis=1).T
    # print("tv: ",tv)
    rotm = np.concatenate((R.from_rotvec(axan).as_matrix(), [[0, 0, 0]]), axis=0)
    # print("rotm: ",rotm)
    hom = np.concatenate((rotm, tv), axis=1)
    # print("hom: ",hom)
    return hom

def b2robotEnd_hom(light_hom, light_pos, light_axan):
    #calculate the homogenous matrix from the base to the light
    b2light_H = axan2hom(light_pos[0:3], light_axan)
    b2tcp_H = np.matmul(b2light_H, light_hom)
    return b2tcp_H


def hom2trans(hom):
    return hom[0:3, 3]

def angle_dif(obj_pose, light_pose):
    angle_light = math.atan2(light_pose[1],light_pose[0])
    angle_obj = math.atan2(obj_pose[1],obj_pose[0])
    angle_dif = angle_light - angle_obj
    return angle_dif


def generate_obtainable_end_position(object_pose, light_position, tcp_offset=[0, 0, 0, 0, 0, 0], robot_obj=None):
    # calculate obtainable tcp positions from light source positions
    #if robot is not set all positions are set

    obtainable_position_robot = []

    if angle_dif(object_pose, light_position) >= 0:
        # print("right, dif: ",angle_dif(object_pose, light_position))
        rot = 270
    else:
        # print("left, dif: ",angle_dif(object_pose, light_position))
        rot = 90

    H_tcp = eul2hom(tcp_offset[0:3], tcp_offset[3:6])
    # print("H_tcp: ",H_tcp)
    H_tcp = np.linalg.inv(H_tcp)
    # print("H_tcp: ",H_tcp)

    # print("position: ",positions[i]," rotation: ",j)
    angle_axis = calculate_angle_axis(object_pose, light_position, rot)
    b2robotEnd_H = b2robotEnd_hom(H_tcp, light_position, angle_axis)
    angle_axis = hom2axan(b2robotEnd_H)
    # b2tcp_H = axan2hom(light_position[0:3], angle_axis)
    robot_pose = hom2trans(b2robotEnd_H).transpose().tolist()
    robot_pose.extend(angle_axis)
    # print("robot_pose: ",robot_pose)

    # print(robot_pose)

    if robot_obj is None:
        obtainable_position_robot = robot_pose
        # print("robot_obj is None pos is: ", robot_pose)
    elif robot_obj.position_is_obtainable(robot_pose):
        obtainable_position_robot =robot_pose
        # print("position is obtainable: ", robot_pose)
    else:
        print(rot, "position is not obtainable: ",rot , robot_pose)
        return None
        
    return obtainable_position_robot
        

def save_positions_to_file(positions, path_to_save_positions):
    # save positions to file
    # if os.path.isfile(path_to_save_positions):
    # # os.toutch(path_to_save_positions)
    # os.path.isfile(path_to_save_positions)
    file = open(path_to_save_positions, "w")
    # clear file content
    file.truncate(0)

    for i in range(len(positions)):
        file.write(str(positions[i][0]) + ", " + str(positions[i][1]) + ", " + str(positions[i][2]) + ", " + str(positions[i][3]) + ", " + str(positions[i][4]) + ", " + str(positions[i][5]) + "\n")
    file.close()

# [-0.4174005432019973, 0.5427838611526721, 0.021020499521421487, -2.994002414920472, -7.485006041160759e-07, -0.951605479667744]