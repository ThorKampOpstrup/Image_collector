# some_file.py
import sys
# caution: path[0] is reserved for script path (or '' in REPL)
sys.path.insert(1, '../')

import pose_calculator as pc
import numpy as np
import robot_class
import math

# object_pose = [0.2, 0.35, 0.1]
object_pose = [0.1818255165440956, -0.5101149287882081, -0.1227379674121187]
# tcp_offset = [0.032, 0, 0.045, 0, -math.pi/2, 0]

light_poses= pc.generate_positions_xyz(object_pose=object_pose, angle=math.pi/1.5, distance=0.30, n=50)
# poses = pose_calculator.generate_default_tcp_poses(poses=poses, tcp_offset=tcp_offset, rotations=rotations)
# robot_default_tcp = pc.generate_obtainable_positions(object_pose=object_pose, positions=light_poses, tcp_offset=tcp_offset)
# pc.save_positions_to_file(light_poses, "/home/thor/Documents/8.Semester/Project/Image_collector/test/pictures_backup/light_positions.txt")
robot_q = []

print("len light_poses: " + str(len(light_poses)))
for pose in light_poses:
    tmp = pc.generate_obtainable_end_position(object_pose=object_pose, light_position=pose) #!should add tcp offset feature
    robot_q.append(tmp)
print(robot_q)
pc.save_positions_to_file(robot_q, "/home/thor/Documents/8.Semester/Project/Image_collector/test/pictures_backup/light_positions.txt")


