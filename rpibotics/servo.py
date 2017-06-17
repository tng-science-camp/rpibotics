"""

"""
import logging
import time
import RPi.GPIO as GPIO


class Servo(object):

    def __init__(self,
                 gpio_pin: int, duty_cycle_0: float, duty_cycle_180: float):
        """

        :rtype: None
        """
        logging.info('Initializing a Servo.')
        self._pin = gpio_pin
        self._duty_cycle_0 = duty_cycle_0
        self._duty_cycle_180 = duty_cycle_180
        #GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin, GPIO.OUT)
        self._pwm = GPIO.PWM(self._pin, 50)

    def __del__(self):
        GPIO.cleanup(self._pin)

    def calculate_duty_cycle(self, angle: float):
        return (self._duty_cycle_180 - self._duty_cycle_0) / 180.0 * angle + \
               self._duty_cycle_0

    def go_to(self, angle: float):
        if angle < 0.0:
            angle = 0.0
        elif angle > 180.0:
            angle = 180.0
        self._pwm.start(self.calculate_duty_cycle(angle))
        time.sleep(1.5)

    def go_to_0(self):
        self.go_to(0.0)

    def go_to_180(self):
        self.go_to(180.0)

    def go_to_90(self):
        self.go_to(90.0)
