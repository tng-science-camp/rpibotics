import logging
import RPi.GPIO as GPIO

# Set up the GPIO pins, referring to the constants


class IRObstacleSensor(object):

    def __init__(self, gpio_pin):
        logging.info('Initializing a IRObstacleSensor.')
        #GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self._gpio_pin = gpio_pin
        GPIO.setup(self._gpio_pin, GPIO.IN)
        GPIO.add_event_detect(self._gpio_pin, GPIO.FALLING,
                              callback=self._execute_when_obstacle_detected)
        self._detect_callbacks = set()

    def __del__(self):
        GPIO.cleanup(self._gpio_pin)

    def add_detect_callback(self, callback):
        self._detect_callbacks.add(callback)

    def clear_detect_callbacks(self):
        self._detect_callbacks.clear()

    def obstacle_is_in_front(self):
        return not GPIO.input(self._gpio_pin)

    def _execute_when_obstacle_detected(self, pin):
        assert self._gpio_pin == pin
        for callback in self._detect_callbacks:
            callback()
