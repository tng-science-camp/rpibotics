import logging
import time
import picamera
from rover.mobility_system_sj17 import MobilitySystem
from rover.dht22 import DHTSensor
from rover.hmc5983 import Magnetometer
from rover.lance import Lance

# from rover import image_processor

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

DHT_SENSOR_CONFIG = {
    'gpio_pin': 24
}

MAG_CONFIG = {
    'port'       : 1,
    'address'    : 0x1E,
    'max_gauss'  : 1.3,
    'declination': (11, 55)
}

LANCE_CONFIG = {
    'gpio_pin'         : 21,
    'duty_cycle_open'  : 3,
    'duty_cycle_closed': 12
}


class Rover(object):
    def __init__(self,
                 mobility_system_config=MOBILITY_SYSTEM_CONFIG,
                 dht_sensor_config=DHT_SENSOR_CONFIG,
                 mag_config=MAG_CONFIG,
                 lance_config=LANCE_CONFIG):
        logging.info('Initializing a Rover.')
        self.mob = MobilitySystem(config=mobility_system_config)
        self.dht = DHTSensor(gpio_pin=dht_sensor_config['gpio_pin'])
        self.mag = Magnetometer(port=mag_config['port'],
                                address=mag_config['address'],
                                max_gauss=mag_config['max_gauss'],
                                declination=mag_config['declination'])
        self.lance = Lance(config=lance_config)
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
