import rtde_receive

rtde_r = rtde_receive.RTDEReceiveInterface("10.42.0.63")
actual_q = rtde_r.getActualTCPPose()

print(actual_q)



rtde_r.disconnect()

print("Connection closed")