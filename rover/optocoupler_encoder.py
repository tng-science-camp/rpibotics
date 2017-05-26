"""

"""
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

class OptocouplerEncoder(object):
    def __init__(self, gpio_pin, s=20):
        self._slit_count = s
        self._gpio_pin = gpio_pin
        self._count = 0
        self._rotations = 0
        self._rotation_rate = 0
        self._previous_update_time = None
        GPIO.setup(self._gpio_pin, GPIO.IN)

    def __del__(self):
        GPIO.cleanup((self._gpio_pin))

    def reset(self):
        self._count = 0
        self._rotations = 0
        self._rotation_rate = None
        self._previous_update_time = None

    def update_on_change(self):
        current_time = time.time()
        self._count += self._count
        self._rotations = self._count / self._slit_count
        if self._previous_update_time is not None:
            duration = current_time - self._previous_update_time
            self._rotation_rate = 1 / self._slit_count / duration
        self._previous_update_time = current_time

    def run(self):
        GPIO.add_event_detect(self._gpio_pin,GPIO.RISING, callback=self.update_on_change)

    def get_rotations(self):
        return self._rotations

    def get_rotation_rate(self):
        return self._rotation_rate


