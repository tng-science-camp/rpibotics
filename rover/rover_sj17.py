import RPi.GPIO as GPIO
from rover.dcmotor import DCMotor


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

    def __del__(self):
        GPIO.cleanup()