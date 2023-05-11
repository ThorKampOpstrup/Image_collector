import rtde_control
import math
import numpy as np

configuration = [-2.21489, -2.14082, -1.68737, -0.88547,  1.56983, -0.64569]
configuration[0] += math.pi/2
print(np.rad2deg(configuration))

controller = rtde_control.RTDEControlInterface("10.42.0.63")
controller.moveJ(configuration, 0.1, 0.1)

controller.disconnect()