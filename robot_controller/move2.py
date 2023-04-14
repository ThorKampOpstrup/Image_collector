import RTDE


# Connect to the UR robot
robot = rtde.RTDE("10.42.0.63")
robot.connect()

# Get the current pose
pose = robot.getActualTCPPose()
print(pose)

# disconnect
robot.disconnect()
