import robot_class
import camera_class
import csv
import os
import cv2 as cv

class setup:
    # robot = robot_class.robot()
    # camera = camera_class.camera()

    def __init__(self, ip="10.42.0.63", gain=0, exposure=100000, light='Daylight6500K'):
        self.robot = robot_class.robot(ip)
        self.camera = camera_class.camera(gain, exposure, light)

    def  take_images_from_list(self, list_of_poses_csv, path_to_save_images, object_pose, speed=0.4, acc=0.5):
        # check if the list of poses exists and file extension is csv
        if not list_of_poses_csv.endswith(".csv"):
            print("The list of poses must be a csv file")
            return
        
        # read list of poses
        file = open(list_of_poses_csv, "r")
        poses = list(csv.reader(file, delimiter=","))
        file.close()

        # convert to float
        for i in range(len(poses)):
            poses[i] = [float(x) for x in poses[i]]

        touch = os.path.isfile(path_to_save_images+"light_positions.txt")
        file = open(path_to_save_images+"light_positions.txt", "a")

        # take images
        for i in range(len(poses)):
            pose = self.robot.move_to(poses[i], speed, acc)
            # subtract all values from object pose 
            pose = [x - y for x, y in zip(pose, object_pose)]
            pose = pose[:3]
            print("Pose: " + str(pose))
            self.camera.take_image(path_to_save_images + "pose_" + str(i) + ".png")
            # pose = self.robot.get_actual_tcp_pose()
            file.write(str(pose[0]) + ", " + str(pose[1]) + ", " + str(pose[2]) +"\n")
            print("Image taken from pose: " + str(i))
        
        file.close()
        print("Images taken from list of poses")

    def get_object_pose(self):
        self.robot.teachMode()
        key = input("Press any key accept object pose")
        self.robot.endTeachMode()
        input("Press any key confirm robot is clear")
        pose = self.robot.get_pose()
        print("Object pose: " + str(pose))
        # add [0.0, 0, 0.2, 0, 0, 0, 0] to pose
        pose = [x + y for x, y in zip(pose, [0, 0, 0.02, 0, 0, 0, 0])]#!update this value
        print("pose: " + str(pose))
        self.robot.move_to(pose, speed=0.5, acc=0.5)
        print("Robot moved")
        return pose

    def generate_mask(self, path_to_save_mask, position_of_to_set_light_path):
        file = open(position_of_to_set_light_path, "r")
        poses = list(csv.reader(file, delimiter=","))
        file.close()

        pose = [float(x) for x in poses[0]]
 
        # print("pose: " + str(pose))
        pose = [float(x) for x in pose]
        self.robot.move_to(pose, 0.4, 0.5)
        input("Confirm that object is NOT present and light is turned on...")
        image_no_object= self.camera.take_image(path_to_save_mask + "mask_tmp.png")
        input("Confirm that object is present and light is turned on...")
        image_object = self.camera.take_image(path_to_save_mask + "mask_tmp2.png")

        # os.rm(path_to_save_mask + "mask_tmp.png")
        # os.rm(path_to_save_mask + "mask_tmp2.png")

        # generate mask
        mask = image_object - image_no_object
        mask[mask < 10 ] = 0
        mask[mask >= 10] = 255
        cv.imwrite(path_to_save_mask + "mask.png", mask)

    

    def disconnect(self):
        self.robot.disconnect()