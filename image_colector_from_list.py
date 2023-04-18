import setup_class
import math

# path_to_list = "pose_calibration/images/poses.csv"
path_to_list = "poses_test.csv"
# path_to_list = "tcp_test_pose.csv"
# save_location = "test/"
save_location = "test/images/"
# speed_param = 3
# acc_param = 5
# shutter_param = 30
speed_param = 3.14
acc_param = 3
shutter_param = 100000
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

    #generate mask image
    setup.generate_mask(path_to_save_mask=save_location, position_of_to_set_light_path=path_to_list)
    

    # move away to place object
    # setup.robot.moveL((object_pose + [0.2, 0, 0, 0, 0, 0, 0]), 0.5, 0.5)



    # wait for user to place object
    input("Press Enter to start... STAY CLEAR OF ROBOT!")

    # set tcp offset
    setup.robot.set_tcp_offset(offset=tcp_offset)

    # take images from list
    setup.take_images_from_list(list_of_poses_csv=path_to_list,
                                path_to_save_images=save_location, object_pose=object_pose, speed=speed_param, acc=acc_param)
    # disconnect
    setup.disconnect()
