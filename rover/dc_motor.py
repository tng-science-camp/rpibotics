"""

"""
import RPi.GPIO as GPIO

# Set up the GPIO pins, referring to the constants
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


class DCMotor(object):

    def __init__(self, ena, in1, in2, f=20):
        self._ena = ena
        self._in1 = in1
        self._in2 = in2
        GPIO.setup(self._ena, GPIO.OUT)
        GPIO.setup(self._in1, GPIO.OUT)
        GPIO.setup(self._in2, GPIO.OUT)
        # Initialize PWM on pwmPin 20Hz frequency
        self._pwm = GPIO.PWM(self._ena, f)

    def __del__(self):
        GPIO.cleanup((self._ena, self._in1, self._in2))

    def turn_clockwise(self, duty_cycle=50):
        """
        Turns the motor clockwise.
        :param duty_cycle: The percent duty cycle of the PWM. Defaults to 50%
        :return: 
        """
        GPIO.output(self._in1, True)
        GPIO.output(self._in2, False)
        self._pwm.start(duty_cycle)

    def turn_counter_clockwise(self, duty_cycle=50):
        """
        Turns the motor counter clockwise.
        :param duty_cycle: The percent duty cucle of the PWM. Defaults to 50%
        :return: 
        """
        GPIO.output(self._in1, False)
        GPIO.output(self._in2, True)
        self._pwm.start(duty_cycle)

    def stop(self):
        """
        Stops the motor.
        :return: 
        """
        GPIO.output(self._in1, False)
        GPIO.output(self._in2, False)
        self._pwm.stop()

    def change_pwm_frequency(self, frequency=20):
        """ 
        Changes the PWM frequency
        :param frequency: The frequency of PWM in Hz. Defaults to 20Hz
        :return: 
        """
        # It's not clear, but some report that 40kHz is max for L298
        if frequency >= 40000:
            frequency = 40000
        self._pwm.ChangeFrequency(frequency)
