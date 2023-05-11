import sys
# caution: path[0] is reserved for script path (or '' in REPL)
sys.path.insert(1, '../')

import setup_class
from math import*
from twin import *
from scipy.spatial.transform import Rotation
import time
import pose_calculator as pc



tcp_offset = [0.032, 0, 0.045, 0, -pi/2, 0]
file_of_poses = "auto_poses.csv"
path_to_save_pictures = "pictures/"
shutter_param = 5000
ip_addr = "10.42.0.63"

def sort_poses(poses):
    # take poses with rotation vector
    # return 2 arrays of poses: left and right side
    left_poses = []
    right_poses = []
    for pose in poses:
        rot_vec = pose[3:]
        eul = Rotation.from_rotvec(rot_vec).as_euler('xyz', degrees=True)
        if eul[0] < 0:
            left_poses.append(pose)
        else:
            right_poses.append(pose)

    return left_poses, right_poses


if __name__ == "__main__":
    setup = setup_class.setup(ip=ip_addr, exposure=shutter_param)

    obj_file = open("obj_pose.csv", "r")
    object_pose = []
    for line in obj_file:
        line = line.strip()
        line = line.split(",")
        object_pose.append([float(i) for i in line])
    obj_file.close()

    print("Object pose: ", object_pose)
    light_poses= pc.generate_positions_xyz(object_pose=object_pose[0], angle=pi/1.5, distance=0.30, n=50)
    print("len light_poses: " + str(len(light_poses)))

    robot_q = []
    for pose in light_poses:
        tmp = pc.generate_obtainable_end_position(object_pose=object_pose[0], light_position=pose, tcp_offset=tcp_offset) #!should add tcp offset feature
        robot_q.append(tmp)
    print(robot_q)
    pc.save_positions_to_file(robot_q, file_of_poses)



    # file = open(file_of_poses, "r")
    # end_poses = []
    # for line in file:
    #     line = line.strip()
    #     line = line.split(",")
    #     end_poses.append([float(i) for i in line])
    # file.close()

    left_poses, right_poses = sort_poses(robot_q)

    print("Left poses: ", left_poses)
    print("\n\nRight poses: ", right_poses)

    # plot_markers_from_rotvec(right_poses)

    # input("Press enter to continue...")
    # plot_markers_from_rotvec(left_poses)
    # input("Press enter to continue...")

    
    setup.generate_util_files_pictures(path_to_save_pictures)
    
    # # input("Press enter to continue...")
    setup.show_pose_and_take_pictures(path_to_save_images=path_to_save_pictures, start_from=0, poses=left_poses) 
    # setup.show_pose_and_take_pictures(path_to_save_images=path_to_save_pictures, start_from=0, poses=left_poses) 
    setup.show_pose_and_take_pictures(path_to_save_images=path_to_save_pictures, start_from=len(left_poses), poses=right_poses)

    
    