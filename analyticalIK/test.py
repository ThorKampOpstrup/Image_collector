from analyticalIK import *
from configuration_selection import *
from webbrowser import Chrome
import roboticstoolbox as rtb
import swift

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



# desired_pos = [-0.464, 0.436, 0.102, 0.0000, 0.0008, -1.0000, -0.0008]
# desired_pos = [-0.264, 0.236, 0.102, 1.0000, 0.0008, 0.0000, -0.0008]
# desired_pos = [-0.264, 0.236, 0.102, 0.37777, 0.20193, 0.23637, 0.87215]
# desired_pos = [-0.264, 0.236, 0.102, 0.4365, -0.3129, -0.0034, 0.8436]
desired_pos =[0.284, -0.456, 0.52, 0.87215, 0.37777, 0.20193, 0.23637]

IK = analyticalIK()
solutions = IK.solve(desired_tcp_pose=desired_pos)

for sol in solutions:
    sol = center_joints_around_0(sol)
    move_robot(env, sol)
    print(sol)
    print(UR5.fkine(sol))
    input("Press Enter to continue...")
