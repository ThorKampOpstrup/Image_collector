import robot_class
import camera_class
from twin import *
import csv
import os
import cv2 as cv
import glob


class setup:

    # robot = robot_class.robot()
    # camera = camera_class.camera()

    def __init__(self, ip="10.42.0.63", gain=0, exposure=100000, light='Daylight6500K'):
        self.robot = robot_class.robot(ip)
        self.camera = camera_class.camera(gain, exposure, light)

    def read_poses_from_csv(self, path_to_csv):
        # check if the list of poses exists and file extension is csv
        if not path_to_csv.endswith(".csv"):
            print("The list of poses must be a csv file")
            return
        
        # read list of poses
        file = open(path_to_csv, "r")
        poses = list(csv.reader(file, delimiter=","))
        file.close()

        # convert to float
        for i in range(len(poses)):
            poses[i] = [float(x) for x in poses[i]]

        return poses

    def  take_images_from_poses(self, poses, path_to_save_images, object_pose, speed=0.4, acc=0.5):
        # check if the list of poses exists and file extension is csv
        # os.remove(path_to_save_images+"light_positions.txt")
        # os.path.isfile(path_to_save_images+"light_positions.txt")
        file = open(path_to_save_images+"light_positions.txt", "a")
        file.truncate(0)
        
        # take images
        for i in range(len(poses)):
            if not self.robot.position_is_obtainable(poses[i]):
                continue
            pose = self.robot.move_to(poses[i], speed, acc)
            # subtract all values from object pose 
            pose = [x - y for x, y in zip(pose, object_pose)]
            pose = pose[:3]
            print("Pose: " + str(pose))
            self.camera.take_image(path_to_save_images + "pose_" + str(i) + ".png")
            # pose = self.robot.get_actual_tcp_pose()
            file.write(str(pose[0]) + " " + str(pose[1]) + " " + str(pose[2]) +"\n")
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
    
    def generate_util_files_pictures(self, path_to_save):
        # delete file with name "light_positions.txt"
        file = open(path_to_save+"light_positions.txt", "a")
        file.truncate(0)
        file.close()

        file = open(path_to_save+"filenames.txt", "a")
        file.truncate(0)
        file.close()

        # Getting All Files List
        fileList = glob.glob(path_to_save+"pose_*.png", recursive=False)
            
        # Remove all files one by one
        for file in fileList:
            try:
                os.remove(file)
            except OSError:
                print("Error while deleting file")
        
        print("Removed all matched files!")


        # delete all images in path_to_save with name "pose_*.png"


    
    def show_pose_and_take_pictures(self, path_to_save_images, start_from, object_pose,  poses, speed=0.4, acc=0.5):
        # file = open(path_to_save_images+"names.txt", "a")
        # get last line
        #         
        # show digital twin and pose

        print("showing digital twin... show the the robot the pose... Press ctrl+c to continue")
        show_robot_pose_freedrive(poses[0], self.robot)
        
        positions_file = open(path_to_save_images+"light_positions.txt", "a")
        filenames = open(path_to_save_images+"filenames.txt", "a")
        object_pose = object_pose[0][:3]

        # take images
        for i in range(len(poses)):
            print("pose ", i, ": ", poses[i])
            if not self.robot.position_is_obtainable(poses[i]):
                continue
            pose = self.robot.move_to(poses[i], speed, acc)
            q_config = self.robot.get_joint_values()
            move_robot(q_config)
            # subtract all values from object pose 
            # print("object_pose: ", object_pose)
            # print("pose: ", pose)
            # pose = [x - y for x, y in zip(pose, object_pose)]
            # pose = pose[:3]
            # print("Pose: ", pose)
            pose = self.robot.receiver.getActualTCPPose()
            # print("adding :", Pose)
            # # append pose to file
            # f.write(str(Pose[0]) + ", " + str(Pose[1]) + ", " + str(Pose[2]) +
            print("object_pose: ", object_pose)
            pose = pose[:3]
            print("Pose: ", pose)
            pose = [x - y for x, y in zip(pose, object_pose)]
            print("Pose: " + str(pose))
            image_number = start_from + i
            self.camera.take_image(path_to_save_images + "pose_" + str(image_number) + ".png")
            # pose = self.robot.get_actual_tcp_pose()
            positions_file.write(str(pose[0]) + " " + str(pose[1]) + " " + str(pose[2]) +"\n")
            filenames.write("pose_" + str(image_number) + ".png\n")
            # print("Image taken from pose: " + str(last_image_number))
        
        positions_file.close()
        filenames.close()
        
    def show_livefeat(camera):
        # show livefeat
        print("showing livefeat... Press ctrl+c to continue")
        try:
            while True:
                img = camera.get_image()
                cv.imshow("Livefeat", img)
        except KeyboardInterrupt:
            cv.destroyAllWindows()
            print("KeyboardInterrupt has been caught.")
    
        


        

    def generate_mask(self, path_to_save_mask, position_of_to_set_light_path):
        MASK_THREASHOLD = 10

        file = open(position_of_to_set_light_path, "r")
        poses = list(csv.reader(file, delimiter=","))
        file.close()

        pose = [float(x) for x in poses[0]]
 
        print("pose: ", pose)
        pose = [float(x) for x in pose]
        self.robot.controller.moveJ(pose, speed=0.4, acceleration=0.5) ##!this is updated
        input("Confirm that object is NOT present and light is turned on...")
        self.camera.take_image(path_to_save_mask + "mask_without.png")
        show_livefeat()
        input("Confirm that object is present and light is turned on...")
        self.camera.take_image(path_to_save_mask + "mask_with.png")

        image_object = cv.imread(path_to_save_mask + "mask_with.png", cv.IMREAD_COLOR)
        image_no_object = cv.imread(path_to_save_mask + "mask_without.png", cv.IMREAD_COLOR)

        #delete images
        # os.remove(path_to_save_mask + "mask_with.png")
        # os.remove(path_to_save_mask + "mask_without.png")

        # avg_black = cv.imread("black/avg_black.png", cv.IMREAD_COLOR)
        # cv.subtract(image_no_object, avg_black, image_no_object)
        # cv.subtract(image_object, avg_black, image_object)

        # blur images
        image_no_object = cv.fastNlMeansDenoising(
            image_no_object, None, 3, 11, 21)
        image_object = cv.fastNlMeansDenoising(
            image_object, None, 3, 11, 21)

        cv.imwrite("tmp_no.png", image_no_object)
        cv.imwrite("tmp_obj.png", image_object)

        # convert images to grayscale
        # image_no_object = cv.cvtColor(image_no_object, cv.COLOR_BGR2GRAY)
        # image_object = cv.cvtColor(image_object, cv.COLOR_BGR2GRAY)

        # generate mask
        mask = cv.absdiff(image_object, image_no_object)
        mask[mask < MASK_THREASHOLD] = 0
        mask[mask >= MASK_THREASHOLD] = 255

        # perform opening
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (10, 10))
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

        #perform closing
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (21, 21))
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)


        cv.imwrite(path_to_save_mask + "mask.png", mask)

    

    def disconnect(self):
        self.robot.disconnect()