import swift
from numpy import *
from configuration_selection import *
import roboticstoolbox as rtb
import spatialgeometry as sg
from spatialmath.base import q2r



from transforms3d.axangles import axangle2mat
from transforms3d.quaternions import axangle2quat
from transforms3d.quaternions import mat2quat
from scipy.linalg import norm
from scipy.spatial.transform import Rotation as R

# initial_config = [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]
initial_config = [0, 0, 0, 0, 0, 0]

__SHOW_ENV__ = True
UR5 = rtb.models.UR5()
env = swift.Swift()

def q_to_hom(q):
    trans_vector = concatenate(([q[0:3]],[[1]]),axis=1).T
    # print("trans_vector: ", trans_vector)
    # print ("q[3:7]: ", q[3:7])
    rot_matrix = concatenate((q2r(q[3:7]),[[0,0,0]]),axis=0)
    homogenous_mat = concatenate((rot_matrix,trans_vector),axis=1)
    print("homogenous_mat: ", homogenous_mat)
    return homogenous_mat

def plot_markers(poses):
    if __SHOW_ENV__:
        """Plot a list of poses as markers in the environment"""
        for pose in poses:
            marker = sg.Axes(0.1, pose=q_to_hom(pose))
            # print("pose: ", pose)
            marker.pose = q_to_hom(pose)
            env.add(marker)

def plot_markers_from_rotvec(poses):
    if __SHOW_ENV__:
        env.step(0.0001)
        """Plot a list of poses as markers in the environment"""
        for pose in poses:
            env.step(0.0001)
            # tmppose = pose[0:3]
            # print("pose: ", pose)
            rotvec = pose[3:]
            # print("rotvec: ", rotvec)
            rotm = R.from_rotvec(rotvec)
            quat = rotm.as_quat()
            tmp = concatenate((pose[0:3], quat))
            marker = sg.Axes(0.1, pose=q_to_hom(tmp))
            print("tmp: ", tmp)
            print("marker: ", marker)
            # marker.pose = q_to_hom(tmp)
            print("marker: ", marker)
            env.step(0.1)
            env.add(marker)


def move_robot(pos):
    if __SHOW_ENV__:
        env.step(0.0001)
        UR5.q = pos
        # marker = UR5.fkine(pos)
        # t = marker.t
        # q = R.from_matrix(marker.R).as_quat()
        # print("t: ", t)
        # print("q: ", q)
        # env.remove.objects()
        # if obj_pose != []:
            # plot_markers_from_rotvec([obj_pose])

        # plot_markers([concatenate((t, q))])
        env.step(0.0001)

if __SHOW_ENV__:
    # init environtment
    env.launch(realtime=True, browser="Google-chrome")
    # add robot and marker to the environement
    env.add(UR5)
    # plot_markers(env, cartesian_pose)
    move_robot(initial_config)

def show_robot_pose_freedrive(pose, robot):
    ##take pose as translation and rotation vector(as saved in csv file)
    robot.teachMode()

    # rotation_vector = pose[3:]
    # rot = axangle2mat(divide(rotation_vector,norm(rotation_vector)), norm(rotation_vector))
    # Q_array = mat2quat(rot)
    # pose2_plot = array([-pose[0], -pose[1], pose[2], Q_array[0], Q_array[1], Q_array[2], Q_array[3]])
    # print("pose_copy: ", pose_copy)
    print("org_copy: ", pose)
    pose_copy = pose.copy()
    pose_copy[0] = -pose_copy[0]
    pose_copy[1] = -pose_copy[1]
    print("pose_copy: ", pose_copy)
    print("org_copy: ", pose)

    # plot_markers_from_rotvec([pose_copy])


    # plot_markers(env, [pose])
    try:
        while True:
            joints = robot.get_joint_values()
            joints = center_joints_around_0(joints)
            move_robot(joints)
    except KeyboardInterrupt:
        robot.endTeachMode()


    
    





