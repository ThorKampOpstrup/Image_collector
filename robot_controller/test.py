# from rtde_control import RTDEControlInterface as RTDEControl
# from rtde_receive import RTDEReceiveInterface as RTDEReceive
# rtde_frequency = 500.0
# rtde_c = RTDEControl("10.42.0.63", rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP)

# # get current pose
# pose = rtde_c.getActualQ()
# print(pose)

# # move to new pose
# pose = pose+0.001
# rtde_c.moveL(pose, 0.5, 0.3)

# # get current pose
# pose = rtde_c.getActualQ()
# print(pose)

# #close connection
# rtde_c.disconnect()



# import rtde_control
# import rtde_receive
# rtde_c = rtde_control.RTDEControlInterface("10.42.0.63")

# # get current pose
# pose = rtde_c.getActualQ()
# print(pose)

# # move to new pose
# pose = pose+0.1
# rtde_c.moveL(pose, 0.1, 0.1)

# # get current pose
# pose = rtde_c.getActualTCPPose()
# print(pose)

# #close connection
# rtde_c.disconnect()


import rtde_receive
import rtde_control
rtde_r = rtde_receive.RTDEReceiveInterface("10.42.0.63")
actual_q = rtde_r.getActualQ()

print(actual_q)

rtde_c = rtde_control.RTDEControlInterface("10.42.0.63")
new_q = [0.28142963381543906, -0.6022172706129398, -0.33360159019641533, 1.0285342403751057, 1.3271312501107964, -0.3546890080337985]
# new_q = [0, 0, 0, 0, 0, 0]
# new_q = [-0.143, -0.435, 0.20, -0.001, 3.12, 0.04]
rtde_c.moveL(new_q, 0.1, 0.3)

print("New pose sent")

rtde_c.disconnect()
rtde_r.disconnect()

print("Connection closed")



