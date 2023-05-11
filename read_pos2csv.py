import rtde_receive
import rtde_control
import math
import time

# !comment this line if it does not work!
tcp_offset = [0.032, 0, 0.045, 0, -math.pi/2, 0]

receiver = rtde_receive.RTDEReceiveInterface("10.42.0.63")
controller = rtde_control.RTDEControlInterface("10.42.0.63")


# robot.set_tcp_offset(offset=tcp_offset)
# robot.set_freedrive()

f = open("tmp.csv", "w")

controller.teachMode()

while 1:
    Pose = receiver.getActualTCPPose()
    #wait 0.1s
    time.sleep(0.5)
    print(Pose)


while 1:
    # wit for key press
    controller.teachMode()
    key = input("Press any key to take a picture, press q to quit ")
    if key == "q":
        controller.endTeachMode()
        controller.disconnect()
        receiver.disconnect()
        f.close()
        break
    Pose = receiver.getActualTCPPose()
    print("adding :", Pose)
    # append pose to file
    f.write(str(Pose[0]) + ", " + str(Pose[1]) + ", " + str(Pose[2]) +
            ", " + str(Pose[3]) + ", " + str(Pose[4]) + ", " + str(Pose[5]) + "\n")
