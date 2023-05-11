from Analytic_IK import *
import roboticstoolbox as rtb
import swift
from spatialmath.base import q2r
from transforms3d.axangles import axangle2mat
from transforms3d.quaternions import mat2quat
import configuration_selection as cs

initial_config = [-np.pi / 2, -np.pi /
                  2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]

pos_to_match = [0.3, 0.3, 0.3]
rot_to_match = [0.5, 0.5, 0.5]

UR5_default =rtb.DHRobot([
rtb.RevoluteDH(d=0.089159, alpha=np.pi / 2),
rtb.RevoluteDH(a=-0.425),
rtb.RevoluteDH(a=-0.39225),
rtb.RevoluteDH(d=0.10915, alpha=np.pi / 2),
rtb.RevoluteDH(d=0.09465, alpha=-np.pi / 2),
rtb.RevoluteDH(d=0.0823)],name="UR5_default")#!UPDATE LAST TO TCP


rot = axangle2mat(rot_to_match, 1)
# Convert to quaternions and print
Q_array = mat2quat(rot)
# print("Q: ", Q)
Q = UnitQuaternion(Q_array)
# print("Q: ", Q)
HT = SE3(pos_to_match[:3]) * Q.SE3()

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

__SHOW_ENV__ = True
UR5 = rtb.models.UR5()
env = swift.Swift()

def move_robot(env, pos):
    if __SHOW_ENV__:
        env.step(0.0001)
        UR5.q = pos
        env.step(0.0001)

if __SHOW_ENV__:
    # init environtment
    env.launch(realtime=True, browser="Google-chrome")
    # add robot and marker to the environement
    env.add(UR5)
    # plot_markers(env, cartesian_pose)
    move_robot(env, initial_config)

def main():
    q_sols = IK(HT, UR5_default)
    plot_markers(env, [0.3, 0.3, 0.3, 0.5, 0.5, 0.5])
    
    for q_sol in q_sols:
        q_sol = cs.center_joint_around_0(q_sol)
        print("q_sol: ", q_sol)
        move_robot(env, q_sol)
        input("Press Enter to continue...")



if __name__ == "__main__":
    main()
