import robot_class
import camera_class
import csv

class setup:
    # robot = robot_class.robot()
    # camera = camera_class.camera()

    def __init__(self, ip="10.42.0.63", gain=0, exposure=100000, light='Daylight6500K'):
        self.robot = robot_class.robot(ip)
        self.camera = camera_class.camera(gain, exposure, light)

    def  take_images_from_list(self, list_of_poses_csv, path_to_save_images, speed=0.4, acc=0.5):
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

        # take images
        for i in range(len(poses)):
            self.robot.move_to(poses[i], speed, acc)
            self.camera.take_image(path_to_save_images + "pose_" + str(i) + ".png")
            print("Image taken from pose: " + str(poses[i]))
        
        print("Images taken from list of poses")



    def disconnect(self):
        self.robot.disconnect()