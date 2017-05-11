import RPi.GPIO as GPIO
from rover.dcmotor import DCMotor
import picamera
import picamera.array
from rover import image_processor


class RoverSJ17(object):
    # There should only be one instance
    _instances = []

    # Initialise the object
    def __init__(self):
        if len(self._instances) > 1:
            print("ERROR: You can't have more than one Rover instance.")
            exit(1)
        self._instances.append(self)
        self.motor1 = DCMotor(13, 26, 19)
        self.motor2 = DCMotor(12, 20, 16)
        self.camera = picamera.PiCamera()
        self.camera.vflip = True
        self.camera.hflip = True

    def __del__(self):
        GPIO.cleanup()

    def capture_image(self, stream, resolution=(640, 480), format='jpeg'):
        camera.resolution = resolution
        camera.capture(stream, format, resize=resolution)

    def find_blue_object(self):
        hsv_lower_range = image_processor.BLUE_HSV_LOWER_RANGE
        hsv_upper_range = image_processor.BLUE_HSV_UPPER_RANGE
        with picamera.array.PiRGBArray(camera, size=(640, 480)) as stream:
            self.capture_image(stream, format='bgr')
            return image_processor.find_colored_object_bearing(
                stream, hsv_lower_range, hsv_upper_range)
