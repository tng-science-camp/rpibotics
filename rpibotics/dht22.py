"""

"""
import logging
import Adafruit_DHT as DHT


class DHTSensor(object):
    def __init__(self, gpio_pin):
        logging.info('Initializing a DHTSensor.')
        self._sensor = DHT.AM2302
        self._pin = gpio_pin

    def measure_humidity_and_temperature(self, retries=15, delay_seconds=2):
        """
        Measures the humidity and the temperature and retries the measurement
        with delay_seconds in between the retries. Note that DHT sensor cannot
        be read faster than once every 2 seconds.
        :param retries: Number of measurement retries
        :param delay_seconds: Number of seconds in between retries in seconds
        :return: (humidity, temperature) in percentage and degree Celsius. 
                 If measurement fails, return (None, None).
        """
        if retries == 0:
            return DHT.read(self._sensor, self._pin)
        else:
            return DHT.read_retry(self._sensor, self._pin,
                                  retries, delay_seconds)

    def measure_humidity(self, retries=15, delay_seconds=2):
        humidity, temperature = self.measure_humidity_and_temperature(
            retries, delay_seconds)
        return humidity

    def measure_temperature(self, retries=15, delay_seconds=2):
        humidity, temperature = self.measure_humidity_and_temperature(
            retries, delay_seconds)
        return temperature

