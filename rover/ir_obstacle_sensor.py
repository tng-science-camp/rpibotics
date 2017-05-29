import RPi.GPIO as GPIO

# Set up the GPIO pins, referring to the constants


class IRObstacleSensor(object):

    def __init__(self, pin):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self._gpio_pin = pin
        GPIO.setup(self._gpio_pin, GPIO.IN)
        GPIO.add_event_detect(self._gpio_pin, GPIO.FALLING,
                              callback=self._execute_on_obstacle_detection)
        self._on_detect_callbacks = list()

    def __del__(self):
        GPIO.cleanup(self._gpio_pin)

    def register_on_detect_callback(self, callback):
        self._on_detect_callbacks.append(callback)

    def obstacle_in_front(self):
        return not GPIO.input(self._gpio_pin)

    def _execute_on_obstacle_detection(self, pin):
        assert self._gpio_pin == pin
        for callback in self._on_detect_callbacks:
            callback()
