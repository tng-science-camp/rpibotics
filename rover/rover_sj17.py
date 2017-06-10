# import logging
import time
import picamera
from rover.mobility_system_sj17 import MobilitySystem
from rover.dht22 import DHTSensor

# from rover import image_processor

GPIO_PIN_DHT22 = 24
MOBILITY_SYSTEM_CONFIG = {
    'motor_left'           : {'gpio_pin_ena': 12,
                              'gpio_pin_in1': 20,
                              'gpio_pin_in2': 16,
                              'frequency'   : 100.0},
    'motor_right'          : {'gpio_pin_ena': 13,
                              'gpio_pin_in1': 26,
                              'gpio_pin_in2': 19,
                              'frequency'   : 100.0},
    'encoder_left'         : {'gpio_pin'  : 8,
                              'slit_count': 20},
    'encoder_right'        : {'gpio_pin'  : 7,
                              'slit_count': 20},
    'obstacle_sensor_left' : {'gpio_pin': 17},
    'obstacle_sensor_right': {'gpio_pin': 27},
    'pid'                  : {'kp': 300.0,
                              'ki': 200.0,
                              'kd': 0.0},
    'initial_duty_cycle'   : {'duty_cycle_left' : 70.0,
                              'duty_cycle_right': 70.0}
}


class Rover(MobilitySystem, DHTSensor):
    def __init__(self):
        MobilitySystem.__init__(self, MOBILITY_SYSTEM_CONFIG)
        DHTSensor.__init__(self, GPIO_PIN_DHT22)
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
