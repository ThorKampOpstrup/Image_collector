import setup_class
import math
import pose_calculator
import numpy as np
from twin import *



# path_to_list = "pose_calibration/images/poses.csv"
# path_to_list = "poses_test.csv"
# path_to_list = "poses_test_above.csv"
path_to_save_list_op_poses = "test.csv"
mask_position_path = "mask_light_pose.csv" #!# make this file store the Joint values
# path_to_list = "tcp_test_pose.csv"
# save_location = "test/"
save_location = "test/images/"
# speed_param = 3
# acc_param = 5
# shutter_param = 30
# speed_param = 3.14
# acc_param = 10
speed_param = 1
acc_param = 1
shutter_param = 5000
tcp_offset = [0.032, 0, 0.045, 0, -math.pi/2, 0]
end_of_shader = [0.032, 0, 0.045, 0, -math.pi/2, 0]
# tcp_offset = [0,0,0,0,0,0]
ip_addr = "10.42.0.63"


# make main function
if __name__ == "__main__":
    # create setup object
    setup = setup_class.setup(ip=ip_addr, exposure=shutter_param)

    # set tcp offset wort object posiion callibration
    setup.robot.set_tcp_offset(offset=end_of_shader)
    object_pose = setup.get_object_pose()
    print("Object pose: " + str(object_pose))

    # generate mask image
    setup.generate_mask(path_to_save_mask=save_location,
                        position_of_to_set_light_path=mask_position_path)

    # move away to place object
    # setup.robot.moveL((object_pose + [0.2, 0, 0, 0, 0, 0, 0]), 0.5, 0.5)

    # wait for user to place object

    # set tcp offset
    setup.robot.set_tcp_offset(offset=[0,0,0,0,0,0]) #the tcp offset is handled by the inverse pose_calculator

    light_poses= pose_calculator.generate_positions_xyz(object_pose=object_pose, angle=math.pi/4, distance=0.30, n=50)
    # poses = pose_calculator.generate_default_tcp_poses(poses=poses, tcp_offset=tcp_offset, rotations=rotations)
    robot_default_tcp = pose_calculator.generate_obtainable_positions(object_pose=object_pose, positions=light_poses, robot_obj=setup.robot)
    print(robot_default_tcp)

    ## place robot near first pose with "digital twin"
    obj_marker = [object_pose[0], object_pose[1], object_pose[2], 0, 0, 0, 0]
    plot_markers(env, [obj_marker])

    

    input("Press Enter to start... STAY CLEAR OF ROBOT!")
    
    # take images from list
    setup.take_images_from_poses(poses=robot_default_tcp,
                                 path_to_save_images=save_location, object_pose=object_pose, speed=speed_param, acc=acc_param)

    # disconnect
    setup.disconnect()
