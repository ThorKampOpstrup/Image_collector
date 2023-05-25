import sys
# caution: path[0] is reserved for script path (or '' in REPL)
sys.path.insert(1, '../')

import setup_class
from math import*
from twin import *
from scipy.spatial.transform import Rotation
import time
import pose_calculator as pc

#angle to take pictures from
min_angle = pi/4 #! ??Is it to low??
max_angle = pi/2.5 #! ??Is it to high??
distance = 0.25
number_of_images = 100

speed = 1
acc = 1


tcp_offset = [-0.035, 0, 0.045, 0, -pi/2, 0]
file_of_poses = "auto_poses.csv"
path_to_save_pictures = "peter/"
mask_position_path = "take_mask_joint_values.csv"
shutter_param = 100000.0
ip_addr = "10.42.0.63"

def sort_poses(poses, object_pose):
    # take poses with rotation vector
    # return 2 arrays of poses: left and right side
    object_pos = object_pose[:3]
    left_poses = []
    right_poses = []
    for pose in poses:
        angle = pc.angle_dif(object_pose, pose[:3])
        if angle < 0: #! might need to switch to >0
            left_poses.append(pose)
        else:
            right_poses.append(pose)

        # rot_vec = pose[3:]
        # eul = Rotation.from_rotvec(rot_vec).as_euler('xyz', degrees=True)
        # if eul[0] < 0:
        #     left_poses.append(pose)
        # else:
        #     right_poses.append(pose)

    return left_poses, right_poses


if __name__ == "__main__":

    obj_file = open("obj_pose.csv", "r")
    object_pose = []
    for line in obj_file:
        line = line.strip()
        line = line.split(",")
        object_pose.append([float(i) for i in line])
    obj_file.close()

    print("Object pose: ", object_pose)
    
    # light_poses = pc.generate_positions_xyz(object_pose=object_pose[0], angle=1.39626, distance=0.25, n=100) 
    light_poses = pc.fibonacci_band_positions(object_pose=object_pose[0], min_angle=min_angle, max_angle=max_angle, distance=distance, n=number_of_images)
    print("len light_poses: " + str(len(light_poses)))

    robot_poses = []
    for pose in light_poses:
        tmp = pc.generate_obtainable_end_position(object_pose=object_pose[0], light_position=pose) #!should add tcp offset feature
        robot_poses.append(tmp)
    print(robot_poses)
    
    
    pc.save_positions_to_file(robot_poses, file_of_poses)



    # file = open(file_of_poses, "r")
    # end_poses = []
    # for line in file:
    #     line = line.strip()
    #     line = line.split(",")
    #     end_poses.append([float(i) for i in line])
    # file.close()

    left_poses, right_poses = sort_poses(robot_poses, object_pose=object_pose[0])

    left_poses = sorted(left_poses, key=lambda x:(sqrt(x[0]**2 + x[1]**2 + x[2]**2))) #! sort by distance from base
    right_poses = sorted(right_poses, key=lambda x:(sqrt(x[0]**2 + x[1]**2 + x[2]**2))) #! sort by distance from base


    print("Left poses: ", left_poses)
    print("len left_poses: ", len(left_poses))
    print("\n\nRight poses: ", right_poses)
    print("len right_poses: ", len(right_poses))

    # plot_markers_from_rotvec(right_poses)

    # plot_markers_from_rotvec(left_poses)
    # input("Press enter to continue...")

  
    setup = setup_class.setup(ip=ip_addr, exposure=shutter_param)
    setup.robot.set_tcp_offset(offset=tcp_offset)
    setup.generate_util_files_pictures(path_to_save_pictures)
    
    input("Press enter move robot to take mask images...")
    setup.generate_mask(path_to_save_mask=path_to_save_pictures, position_of_to_set_light_path=mask_position_path)
    print("Mask generated")

    # input("Press enter to continue...")
    setup.show_pose_and_take_pictures(path_to_save_images=path_to_save_pictures, object_pose=object_pose, start_from=0, poses=left_poses, speed=speed, acc=acc)
    # setup.show_pose_and_take_pictures(path_to_save_images=path_to_save_pictures, start_from=0, poses=left_poses) 
    setup.show_pose_and_take_pictures(path_to_save_images=path_to_save_pictures, object_pose=object_pose, start_from=len(left_poses), poses=right_poses, speed=speed, acc=acc)

    
    