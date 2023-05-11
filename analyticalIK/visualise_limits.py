from webbrowser import Chrome
import roboticstoolbox as rtb
import swift
import numpy as np
from math import *
import configuration_selection as cs

UR5WithTCP = rtb.DHRobot([
    rtb.RevoluteDH(d=0.089159, alpha=np.pi / 2),
    rtb.RevoluteDH(a=-0.425),
    rtb.RevoluteDH(a=-0.39225),
    rtb.RevoluteDH(d=0.10915, alpha=np.pi / 2),
    rtb.RevoluteDH(d=0.09465, alpha=-np.pi / 2),
    rtb.RevoluteDH(d=0.0823)], name="UR5WithTCP")  # !UPDATE LAST TO TCP


__SHOW_ENV__ = True
UR5 = rtb.models.UR5()
env = swift.Swift()

initial_config = [-np.pi / 2, -np.pi /
                  2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]


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
    # plot_markers(env, cartesian_pose)
    move_robot(env, initial_config)


def main():
    q_to_plot = [
        [0.4359, 0.4359, 0.4359, 0.4359, -2.3476, -2.3476, -2.3476, -2.3476],
        [-2.9829, -1.9167, -2.9759, -1.9515, -2.9347, -1.8685, -2.9277, -1.9033],
        [1.1162, -1.1162, 1.0720, -1.0720, 1.1163, -1.1163,  1.0719, -1.0719],
        [-0.0213, -0.0009, -0.1399, -0.1204, 0.0266, 0.0470, -0.0923, -0.0727],
        [1.8575, 1.8575, -1.8575, -1.8575, 1.8666, 1.8666, -1.8666, -1.8666],
        [0.7418, 0.7418, -2.3998, -2.3998, 0.7440, 0.7440, -2.3976, -2.3976]
    ]
    # for pos
    q_to_plot = np.array(q_to_plot).T

    for q in q_to_plot:
        q = cs.center_joints_around_0(q)
        print("q: ", q)
        move_robot(env, q)
        print("cartesian pose:\n", UR5.fkine(q))
        # print("shoulder: ", cs.shoulder_left_or_right(q))
        input('Press enter to continue')


if __name__ == "__main__":
    main()
