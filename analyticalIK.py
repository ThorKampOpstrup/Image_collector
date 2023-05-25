##Developed in ROVI course 2023 - Jannich Nielsen, Peter Nørris Nielsen, Phillip Marschner, Jacob Brændstrup, Leia Popper & Thor Opstrup
import math
import numpy as np
from numpy import arctan2 as atan2
from numpy import arcsin as asin
from numpy import arccos as acos
from numpy import sin as sin
from numpy import cos as cos
from numpy import linalg
from spatialmath.base import q2r
from spatialmath.base import eul2r


class analyticalIK:
    dh_param = np.array([
        #ai-1,        alphai-1,           di,        thetai
        [0,             0,              0.089159,          0],
        [0,             np.pi/2,           0,            0],
        [-0.425,        0,                 0,            0],
        [-0.39225,       0,               0.10915,         0],
        [0,             np.pi/2,         0.09465,         0],
        [0,            -np.pi/2,         0.0823,         0]])


    def q_to_hom(self, q):
        trans_vector = np.concatenate(([q[0:3]], [[1]]), axis=1).T
        # print("testssssts: ", q[3:7])
        rot_matrix = np.concatenate((q2r(q[3:7]), [[0, 0, 0]]), axis=0)
        # print("rot:\n", rot_matrix)
        homogenous_mat = np.concatenate((rot_matrix, trans_vector), axis=1)
        return homogenous_mat

    def TransFunc(self, i, angles):
        # T^i-1_i
        a = self.dh_param[:, 0]
        alpha = self.dh_param[:, 1]
        d = self.dh_param[:, 2]
        theta = angles
        return [
            [cos(theta[i]),                 -sin(theta[i]),                 0,                  a[i]],
            [sin(theta[i])*cos(alpha[i]),   cos(theta[i])*cos(alpha[i]),    -sin(alpha[i]),     -sin(alpha[i])*d[i]],
            [sin(theta[i])*sin(alpha[i]),   cos(theta[i])*sin(alpha[i]),    cos(alpha[i]),      cos(alpha[i])*d[i]],
            [0,                             0,                              0,                   1]]

    def theta1(self, angles, P05x, P05y):  # two solutions
        d4 = self.dh_param[3][2]
        sol1 = angles.copy()
        sol2 = angles.copy()
        sol1[0] = atan2(P05y, P05x) + acos((d4) /
                                           math.sqrt(P05x**2 + P05y**2))+math.pi/2
        sol2[0] = atan2(P05y, P05x) - acos((d4) /
                                           math.sqrt(P05x**2 + P05y**2))+math.pi/2
        return [sol1, sol2]

    def theta2(self, angles, P14x, P14z):
        theta3 = angles[2]
        a3 = self.dh_param[3][0]
        angles[1] = atan2(-P14z, -P14x) - asin((-a3*sin(theta3)) /
                                               ((np.linalg.norm(np.array([P14x, P14z])))))
        return angles

    def theta3(self, angles, P14x, P14z):
        a2 = self.dh_param[2][0]
        a3 = self.dh_param[3][0]
        sol1 = angles.copy()
        sol2 = angles.copy()
        #print('acos', ((np.linalg.norm(np.array([P14x, P14z])))**2 - (a2)**2 - (a3)**2) / (2* a2 * a3))
        sol1[2] = acos(((np.linalg.norm(np.array([P14x, P14z])))
                       ** 2 - (a2)**2 - (a3)**2) / (2 * a2 * a3))
        sol2[2] = -acos(((np.linalg.norm(np.array([P14x, P14z])))
                        ** 2 - (a2)**2 - (a3)**2) / (2 * a2 * a3))

        # add error handling
        return [sol1, sol2]

    def theta4(self, angles, X34y, X34x):
        # X34y is a unit vector giving the direction of the x-axis of frame 4 seen from 3
        angles[3] = atan2(X34y, X34x)
        return angles

    def theta5(self, angles, P06x, P06y):  # 2 solution
        # P06x, P06y are the x and y coordinates of the origin of frame 6, seen from frame 0
        d4 = self.dh_param[3][2]
        d6 = self.dh_param[5][2]
        theta1 = angles[0]
        sol1 = angles.copy()
        sol2 = angles.copy()
        sol1[4] = acos((P06x*sin(theta1)-P06y*cos(theta1)-d4)/d6)
        sol2[4] = - acos((P06x*sin(theta1)-P06y*cos(theta1)-d4)/d6)
        return [sol1, sol2]

    # Y06 is a unit vector giving the direction of the y axis of frame 6, seen from frame 0
    def theta6(self, angles, X60y, Y60y, X60x, Y60x):
        # X60y unit vector giving the direction of the x axis of frame 0, seen from frame 6
        # Y60y unit vector giving the direction of the y axis of frame 0, seen from frame 6
        theta1 = angles[0]
        theta5 = angles[4]

        var1 = (-X60y*sin(theta1) + Y60y*cos(theta1)) / (sin(theta5))
        var2 = (X60x*sin(theta1) - Y60x*cos(theta1)) / (sin(theta5))
        angles[5] = atan2(var1, var2)
        return angles

    def P05(self, T06):
        d6 = self.dh_param[5][2]
        out_p05 = np.array([0, 0, -d6, 1])
        c = np.matmul(T06, np.transpose(out_p05))
        return c

    def P06(self, q):
        return [q[0], q[1], q[2]]

    def solve(self, desired_tcp_pose):
        initial_config = [0, 0, 0, 0, 0, 0]
        t06 = self.q_to_hom(desired_tcp_pose)
        [P05x, P05y, _, _] = self.P05(t06)

        # Calculate angle1
        new_configurations = self.theta1(initial_config, P05x, P05y)

        [P06x, P06y, _] = self.P06(desired_tcp_pose)

        # Calculate angle5
        old_configurations = new_configurations.copy()
        new_configurations = []

        for config in old_configurations:
            sol = self.theta5(config, P06x, P06y)
            new_configurations.append(sol[0])
            new_configurations.append(sol[1])

        # Precalculations for angle6
        # Homogenous matrix of frame 0 seen from frame 6.
        t60 = np.linalg.inv(t06)

        X60y = t60[0][1]
        Y60y = t60[1][1]
        X60x = t60[0][0]
        Y60x = t60[1][0]

        # Calculate angle6
        old_configurations = new_configurations.copy()

        new_configurations = []
        for config in old_configurations:
            sol = self.theta6(config, X60y, Y60y, X60x, Y60x)
            new_configurations.append(sol)

         # angle 3
        old_configurations = new_configurations.copy()
        new_configurations = []

        for config in old_configurations:
            # Precalculations for angle3

            ## P14x & z
            T01 = self.TransFunc(0, config)
            T45 = self.TransFunc(4, config)
            T56 = self.TransFunc(5, config)
            T14 = np.matmul(np.linalg.inv(T01), np.matmul(
                t06, np.linalg.inv(np.matmul(T45, T56))))
            P14x = T14[0][3]
            P14z = T14[2][3]

            # Calculate angle3
            sol = self.theta3(config, P14x, P14z)
            new_configurations.append(sol[0])
            new_configurations.append(sol[1])

        # Calculate angle2
        old_configurations = new_configurations.copy()
        new_configurations = []

        for config in old_configurations:
            ## P14x & z
            T01 = self.TransFunc(0, config)
            T45 = self.TransFunc(4, config)
            T56 = self.TransFunc(5, config)
            T14 = np.matmul(np.linalg.inv(T01), np.matmul(
                t06, np.linalg.inv(np.matmul(T45, T56))))
            P14x = T14[0][3]
            P14z = T14[2][3]
            sol = self.theta2(config, P14x, P14z)
            new_configurations.append(sol)

        # Angle4
        old_configurations = new_configurations.copy()
        new_configurations = []

        for config in old_configurations:
            # Precalculaitons for angle4
            #X34y & x
            T01 = self.TransFunc(0, config)
            T12 = self.TransFunc(1, config)
            T23 = self.TransFunc(2, config)
            T45 = self.TransFunc(4, config)
            T56 = self.TransFunc(5, config)
            T34 = np.matmul(np.linalg.inv(np.matmul(T01, np.matmul(T12, T23))), np.matmul(
                t06, np.linalg.inv(np.matmul(T45, T56))))
            X34y = T34[1, 0]
            X34x = T34[0, 0]

            # Calculate angle4
            sol = self.theta4(config, X34y, X34x)
            new_configurations.append(sol)
            # print(sol)

        valid_configurations = []
        for config in new_configurations:
            # loop though all angles if all angles are not nan, append to valid_configurations
            if not np.isnan(config).any():
                valid_configurations.append(config)

        return valid_configurations

        # Calculate angle2
        # return theta1, theta2, theta3, theta4, theta5, theta6
