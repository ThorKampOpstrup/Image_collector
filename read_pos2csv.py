import robot_class
import cv2 as cv


robot = robot_class.robot("10.42.0.63")

f = open("poses_test.csv", "w")



while 1:
    # wit for key press
    key = input("Press any key to take a picture, press q to quit ")
    if key == "q":
        robot.disconnect()
        f.close()
        break
    Pose = robot.get_pose()
    print("adding :", Pose)
    # append pose to file
    f.write(str(Pose[0]) + "," + str(Pose[1]) + "," + str(Pose[2]) + "," + str(Pose[3]) + "," + str(Pose[4]) + "," + str(Pose[5]) + "\n")
