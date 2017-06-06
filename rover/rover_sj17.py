import logging
import time
import picamera
from rover.mobility_system_sj17 import MobilitySystem
#from rover import image_processor


class Rover(object):
    # There should only be one instance
    _instances = []

    # Initialise the object
    def __init__(self):
        if len(self._instances) > 1:
            print("ERROR: You can't have more than one Rover instance.")
            exit(1)
        self._instances.append(self)
        self._image_folder = "/var/www/html/rover_img/"
        self.mobility_system = MobilitySystem()

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
            print("@capture_image: Take photo and save it as " + image_path)

    def front_is_blocked(self):
        blocked = self.mobility_system.front_is_blocked()
        if blocked:
            print("@front_is_blocked: Front is blocked.")
        else:
            print("@front_is_blocked: Front is not blocked.")
        return blocked

    def go_forward(self, target_distance: float = 0.3):
        return self.mobility_system.go_forward(target_distance=target_distance)

    def go_backward(self, target_distance: float = 0.3):
        return self.mobility_system.go_backward(target_distance=target_distance)

    def turn_right(self, target_angle: float = 90.0):
        return self.mobility_system.turn_right(target_angle=target_angle)

    def turn_left(self, target_angle: float = 90.0):
        return self.mobility_system.turn_left(target_angle=target_angle)

    # def find_blue_object(self):
    #     hsv_lower_range = image_processor.BLUE_HSV_LOWER_RANGE
    #     hsv_upper_range = image_processor.BLUE_HSV_UPPER_RANGE
    #     with picamera.array.PiRGBArray(camera, size=(640, 480)) as stream:
    #         self.capture_image(stream, format='bgr')
    #         return image_processor.find_colored_object_bearing(
    #             stream, hsv_lower_range, hsv_upper_range)
