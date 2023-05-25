import pypylon.pylon as py
import numpy as np
import cv2

class camera:
    tlf = py.TlFactory.GetInstance()
    cam = py.InstantCamera()

    # !COLOR def __init__(self, gain, exposure, light):
    #     self.cam = py.InstantCamera(self.tlf.CreateFirstDevice())
    #     print("Using device:", self.cam.GetDeviceInfo().GetModelName())
    #     self.cam.Open()

    #     self.cam.AcquisitionMode.SetValue('SingleFrame')
    #     self.cam.ColorTransformationSelector.SetValue('RGBtoRGB')
    #     self.cam.LightSourceSelector.SetValue(light)
    #     print("light: " + self.cam.LightSourceSelector.GetValue())

    #     self.cam.PixelFormat.SetValue('BayerRG8')
    #     print ("Pixel format: " + self.cam.PixelFormat.GetValue())

    #     self.cam.GainRaw.SetValue(gain)
    #     print("Gain: " + str(self.cam.GainRaw.GetValue()))

    #     self.cam.ExposureTimeAbs.SetValue(exposure)
    #     print("Exposure: " + str(self.cam.ExposureTimeAbs.GetValue()))

    def __init__(self, gain, exposure, light):
        self.cam = py.InstantCamera(self.tlf.CreateFirstDevice())
        print("Using device:", self.cam.GetDeviceInfo().GetModelName())
        self.cam.Open()

        self.cam.AcquisitionMode.SetValue('SingleFrame')
        # self.cam.ColorTransformationSelector.SetValue('RGBtoRGB')
        # self.cam.LightSourceSelector.SetValue(light)
        # print("light: " + self.cam.LightSourceSelector.GetValue())

        # self.cam.PixelFormat.SetValue('BayerRG8')
        print ("Pixel format: " + self.cam.PixelFormat.GetValue())

        self.cam.GainRaw.SetValue(gain)
        print("Gain: " + str(self.cam.GainRaw.GetValue()))

        self.cam.ExposureTimeAbs.SetValue(exposure)
        print("Exposure: " + str(self.cam.ExposureTimeAbs.GetValue()))

    def set_exposure(self, exposure):
        self.cam.ExposureTimeAbs.SetValue(exposure)
        print("Exposure: " + str(self.cam.ExposureTimeAbs.GetValue()))
    
    def set_gain(self, gain):
        self.cam.GainRaw.SetValue(gain)
        print("Gain: " + str(self.cam.GainRaw.GetValue()))
    
    def set_light(self, light):
        self.cam.LightSourceSelector.SetValue(light)
        print("light: " + self.cam.LightSourceSelector.GetValue())
    
    def return_image(self):
        self.cam.StartGrabbing()
        grab_result = self.cam.RetrieveResult(200)
        self.cam.StopGrabbing()

        image_array = grab_result.Array
        image_rgb = cv2.cvtColor(image_array, cv2.COLOR_BAYER_RG2RGB)

        return image_rgb
    
    def take_image(self, path_to_save):
        self.cam.StartGrabbing()
        grab_result = self.cam.RetrieveResult(200)
        self.cam.StopGrabbing()

        image_array = grab_result.Array
        image_rgb = cv2.cvtColor(image_array, cv2.COLOR_BAYER_RG2RGB)

        cv2.imwrite(path_to_save, image_rgb)
        return image_rgb
        # print("Image saved to: " + path_to_save)



