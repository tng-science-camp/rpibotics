"""

"""
import RPi.GPIO as GPIO
import time
import math

GPIO.setmode(GPIO.BCM)


class OptocouplerEncoder(object):
    def __init__(self, gpio_pin, s=20):
        self._slit_count = s
        self._gpio_pin = gpio_pin
        self._count = 0
        self._rotations = 0
        self._rotation_rate = float('nan')
        self._previous_update_time = float('nan')
        GPIO.setup(self._gpio_pin, GPIO.IN)
        GPIO.add_event_detect(self._gpio_pin, GPIO.BOTH,
                              callback=self.increment_and_update)

    def __del__(self):
        GPIO.cleanup((self._gpio_pin))

    def reset(self):
        self._count = 0
        self._rotations = 0
        self._rotation_rate = float('nan')
        self._previous_update_time = float('nan')

    def increment_and_update(self, gpio_pin):
        assert self._gpio_pin == gpio_pin
        current_time = time.time()
        self._count += 1
        self._rotations = self._count / self._slit_count / 2
        if not math.isnan(self._previous_update_time):
            duration = current_time - self._previous_update_time
            self._rotation_rate = 1 / self._slit_count / 2 / duration
        self._previous_update_time = current_time

    def get_rotations(self):
        return self._rotations

    def get_rotation_rate(self):
        return self._rotation_rate


