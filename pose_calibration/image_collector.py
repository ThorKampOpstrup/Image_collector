import pypylon.pylon as py
import numpy as np
import cv2

#! this sample has been tested with a Basler acA1920-155uc
# type 'q' or 'ESC' in the window to close it

# the camera is configured to run at high framerate with only two lines hight
# the acquired rows are concatenated as a virtual frame and this frame is displayed
cam = py.InstantCamera()

def setup_camera():
    tlf = py.TlFactory.GetInstance()

    global cam
    cam = py.InstantCamera(tlf.CreateFirstDevice())
    print("Using device:", cam.GetDeviceInfo().GetModelName())
    cam.Open()

    # Set the image acquisition mode to single frame
    cam.AcquisitionMode.SetValue('SingleFrame')

    cam.ColorTransformationSelector.SetValue('RGBtoRGB')
    cam.LightSourceSelector.SetValue('Daylight6500K')
    print("light: " + cam.LightSourceSelector.GetValue())

    # Set the pixel format to RGB8
    # print ("Pixel format: " + cam.PixelFormat.GetValue())
    cam.PixelFormat.SetValue('BayerRG8')
    # cam.pixelFormat.setValue('YUV422Packed')
    # cam.PixelFormat.SetValue('Mono8')
    print ("Pixel format: " + cam.PixelFormat.GetValue())
    # camera.PixelFormat.SetValue(PixelFormat_BayerRG8)

    cam.GainRaw.SetValue(0)
    print("Gain: " + str(cam.GainRaw.GetValue()))

    # Set the exposure time to 10000 microseconds (10 milliseconds)
    # cam.ExposureTimeAbs.SetValue(100000)
    cam.ExposureTimeAbs.SetValue(10)
    # cam.StartGrabbing()


def take_image(path_to_save):
# Start the image acquisition
    global cam
    cam.StartGrabbing()
    # Retrieve the captured image
    # grab_result = cam.RetrieveResult(5000, py.TimeoutHandling_ThrowException)
    grab_result = cam.RetrieveResult(200)
    # grab_result = cam.RetrieveResult(5000, py.TimeoutHandling_ThrowException)


    # Convert the grabbed image to a NumPy array
    image_array = grab_result.Array
    # convert to RGB
    # image_rgb = cv2.cvtColor(image_array, cv2.COLOR_YUVRGB)
    # image_rgb = image_array
    image_rgb = cv2.cvtColor(image_array, cv2.COLOR_BAYER_RG2RGB)


    # Save the image to a file
    print("size: ", image_rgb.shape)
    print("type: ", image_rgb.dtype)
    # print(image_array)

    #blur avg_black
    # avg_black = cv2.blur(avg_black, (3,3))
    # cv2.imwrite("avg_black.png", avg_black)
    # cv2.subtract(image_rgb, avg_black, image_rgb)

    # image_rgb = cv2.blur(image_rgb, (3,3))
    # image_rgb = cv2.fastNlMeansDenoisingColored(image_rgb, None, 10, 10, 7, 21)
    cv2.imwrite(path_to_save, image_rgb)
    cam.StopGrabbing()

    # py.ImagePersistence.Save(py.ImagePersistence.ImageFileFormat_Bmp, 'image.bmp', grab_result)


    # Release the grabbed image and stop the image acquisition
# def disconnect_camera():
#     global cam
#     cam.StopGrabbing()




# define main function
if __name__ == "__main__":
    setup_camera()

    take_image(path_to_save = "images/pose6.png")
    # disconnect_camera()










# # setup center scan line
# cam.Height = SCANLINE_HEIGHT
# cam.Width = cam.Width.Max
# cam.CenterX = True
# cam.CenterY = True

# # setup for
# cam.PixelFormat = "BRG8"
# # cam.Gain = 20
# # cam.ExposureTime = 900
# # print("Resulting framerate:", cam.ResultingFrameRate.Value)

# cam.StartGrabbing()

# img = np.ones((VIRTUAL_FRAME_HEIGHT, cam.Width.Value, 3), dtype=np.uint8)
# missing_line = np.ones(
#     (SCANLINE_HEIGHT, cam.Width.Value, 3), dtype=np.uint8)*255
# image_idx = 0
# while True:
#     for idx in range(VIRTUAL_FRAME_HEIGHT // SCANLINE_HEIGHT):
#         with cam.RetrieveResult(2000) as result:
#             if result.GrabSucceeded():
#                 with result.GetArrayZeroCopy() as out_array:
#                     img[idx * SCANLINE_HEIGHT:idx *
#                         SCANLINE_HEIGHT + SCANLINE_HEIGHT] = out_array
#             else:
#                 img[idx * SCANLINE_HEIGHT:idx * SCANLINE_HEIGHT +
#                     SCANLINE_HEIGHT] = missing_line
#                 print(idx)

#     img_rgb = img

#     # Display the resulting frame
#     cv2.imshow('Linescan View', img_rgb)

#     image_idx += 1
#     if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
#         break

# # When everything done, release the capture
# cam.StopGrabbing()
# cv2.destroyAllWindows()

# cam.Close()