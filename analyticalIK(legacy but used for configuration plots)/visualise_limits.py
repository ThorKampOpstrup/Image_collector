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
        [5.318, -0.518, 0.969, 1.118, 1.572-pi/2, 0.604],
        [5.318, 0.409, -0.969, 2.13, 1.572-pi/2, 0.604],
        [5.318, -0.902, 1.751, -2.421, -1.572+pi/2, -2.537],
        [5.138, 0.753, -1.751, -0.574, -1.572+pi/2, -2.537],
        [2.598, -3.894, 1.751, -2.568, 1.569-pi/2, -2.116],
        [2.598, -2.24, -1.751, -0.721, 1.569-pi/2, -2.116],
        [2.598, -3.551, 0.969, 1.012, -1.569+pi/2, 1.026],
        [2.598, -2.624, -0.969, 2.023, -1.569+pi/2, 1.026]
    ]
    # for pos
    # q_to_plot = np.array(q_to_plot).T

    for q in q_to_plot:
        q = cs.center_joints_around_0(q)
        print("q: ", q)
        move_robot(env, q)
        print("cartesian pose:\n", UR5.fkine(q))
        # print("shoulder: ", cs.shoulder_left_or_right(q))
        input('Press enter to continue')


if __name__ == "__main__":
    main()
