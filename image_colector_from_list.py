import setup_class

# path_to_list = "pose_calibration/images/poses.csv"
path_to_list = "poses_test.csv"
# save_location = "test/"
save_location = "test/images/"
# speed_param = 3
# acc_param = 5
# shutter_param = 30
speed_param = 2
acc_param = 1
shutter_param = 100000

ip_addr = "10.42.0.63"



# make main function
if __name__ == "__main__":
    # create setup object
    setup = setup_class.setup(ip=ip_addr, exposure=shutter_param)
    # take images from list
    setup.take_images_from_list(list_of_poses_csv = path_to_list, path_to_save_images = save_location, speed=speed_param, acc=acc_param)
    # disconnect
    setup.disconnect()
