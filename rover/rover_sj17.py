import logging
import time
import picamera
from rover.mobility_system_sj17 import MobilitySystem
#from rover import image_processor


class Rover(MobilitySystem):
    def __init__(self):
        MobilitySystem.__init__(self)
        self._image_folder = "/var/www/html/rover_img/"

    def capture_image(self,
                      resolution=(1280, 720), iso=None, shutter_speed=None):
        image_path = self._image_folder + \
                     time.strftime("%Y%m%d-%H%M-%S") + '.jpg'
        with picamera.PiCamera(resolution=resolution) as camera:
            camera.vflip = True
            camera.hflip = True
            if iso is not None:
                camera.iso = iso
            if shutter_speed is not None:
                camera.shutter_speed = shutter_speed
            time.sleep(2)
            camera.capture(image_path)
        return image_path


    # def find_blue_object(self):
    #     hsv_lower_range = image_processor.BLUE_HSV_LOWER_RANGE
    #     hsv_upper_range = image_processor.BLUE_HSV_UPPER_RANGE
    #     with picamera.array.PiRGBArray(camera, size=(640, 480)) as stream:
    #         self.capture_image(stream, format='bgr')
    #         return image_processor.find_colored_object_bearing(
    #             stream, hsv_lower_range, hsv_upper_range)
