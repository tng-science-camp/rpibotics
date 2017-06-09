#!/usr/bin/env python
# import smbus
import math
import time
from typing import NoReturn, Tuple

CONFIG_REGISTER_A = 0x00
CONFIG_REGISTER_B = 0x01
MODE_REGISTER = 0x02
X_MSB_REGISTER = 0x03
X_LSB_REGISTER = 0x04
Z_MSB_REGISTER = 0x05
Z_LSB_REGISTER = 0x06
Y_MSB_REGISTER = 0x07
Y_LSB_REGISTER = 0x08
STATUS_REGISTER = 0x09
ID_REGISTER_A = 0x10
ID_REGISTER_B = 0x11
ID_REGISTER_C = 0x12

CONTINUOUS_MEASUREMENT_MODE = 0x00
SINGLE_MEASUREMENT_MODE = 0x01
IDLE_MODE = 0x03

MEASUREMENT_MODE_BITS_OFFSET = 0
DATA_OUTPUT_RATE_BITS_OFFSET = 2
NUM_OF_SAMPLES_BITS_OFFSET = 5

MEASUREMENT_MODE_SETTING = {
    'normal'       : 0x00,
    'positive_bias': 0x01,
    'negative_bias': 0x02
}

DATA_OUTPUT_RATE_SETTING = {
    0.75: 0x00,
    1.5 : 0x01,
    3   : 0x02,
    7.5 : 0x03,
    15.0: 0x04,
    30.0: 0x05,
    75.0: 0x06
}

NUM_OF_SAMPLES_SETTING = {
    1 : 0x00,
    2 : 0x01,
    10: 0x03,
    11: 0x04
}

GAIN_CONFIG_BITS_OFFSET = 5

GAIN_SETTINGS = {
    0.88: {'gain': 0x00, 'resolution': 0.73},
    1.30: {'gain': 0x01, 'resolution': 0.92},
    1.90: {'gain': 0x02, 'resolution': 1.22},
    2.50: {'gain': 0x03, 'resolution': 1.52},
    4.00: {'gain': 0x04, 'resolution': 2.27},
    4.70: {'gain': 0x05, 'resolution': 2.56},
    5.60: {'gain': 0x06, 'resolution': 3.03},
    8.10: {'gain': 0x07, 'resolution': 4.35},
}

UNDERFLOW_OVERFLOW_OUTPUT_VALUE = -4096


def compute_twos_complement(self, val, len):
    if (val & (1 << len - 1)):
        val = val - (1 << len)
    return val


def convert_radians_to_degrees_minutes(val):
    val_in_degrees = math.degrees(val)
    degrees = math.floor(val_in_degrees)
    minutes = round((val_in_degrees - degrees) * 60)
    return degrees, minutes


class Magnetometer(object):
    def __init__(self, port=1, address=0x1E, max_gauss: float = 1.3,
                 declination=(0, 0)):
        self._bus = smbus.SMBus(port)
        self.__max_gauss = max_gauss
        self.address = address

        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = math.radians(degrees + minutes / 60)

        self.set_data_output_configuration(
            measurement_mode='normal',
            data_output_rate=15.0,
            num_of_samples=8
        )
        self.set_gain_configuration(GAIN_SETTINGS[self.__max_gauss]['gain'])
        self.set_measurement_mode(CONTINUOUS_MEASUREMENT_MODE)

    def set_data_output_configuration(
            self,
            measurement_mode: str = 'normal',
            data_output_rate: float = 15.0,
            num_of_samples: int = 1) -> NoReturn:
        self._bus.write_byte_data(
            self.address,
            CONFIG_REGISTER_A,
            MEASUREMENT_MODE_SETTING[measurement_mode] <<
            MEASUREMENT_MODE_BITS_OFFSET |
            DATA_OUTPUT_RATE_SETTING[data_output_rate] <<
            DATA_OUTPUT_RATE_BITS_OFFSET |
            NUM_OF_SAMPLES_SETTING[num_of_samples] <<
            NUM_OF_SAMPLES_BITS_OFFSET)

    def set_gain_configuration(
            self,
            gain: int = GAIN_SETTINGS[1.3]['gain']) -> NoReturn:
        self._bus.write_byte_data(self.address,
                                  CONFIG_REGISTER_B,
                                  gain << GAIN_CONFIG_BITS_OFFSET)

    def get_data(self):
        data = self._bus.read_i2c_block_data(self.address, 0x00)
        x = self.__convert_data_output(data[X_MSB_REGISTER],
                                       data[X_LSB_REGISTER])
        y = self.__convert_data_output(data[Y_MSB_REGISTER],
                                       data[Y_LSB_REGISTER])
        z = self.__convert_data_output(data[Z_MSB_REGISTER],
                                       data[Z_LSB_REGISTER])
        return x, y, z

    def set_declination(self, degrees: float, minutes: float) -> NoReturn:
        self.__declDegrees = degrees
        self.__declMinutes = minutes

    def get_declination(self) -> Tuple[float, float]:
        return self.__declDegrees, self.__declMinutes

    def set_measurement_mode(self,
                             mode: int = CONTINUOUS_MEASUREMENT_MODE) -> NoReturn:
        self._bus.write_byte_data(self.address, MODE_REGISTER, mode)

    def __convert_data_output(self, msb, lsb):
        val = compute_twos_complement(msb << 8 | lsb, 16)
        if val == UNDERFLOW_OVERFLOW_OUTPUT_VALUE:
            return None
        else:
            return val * GAIN_SETTINGS[self.__max_gauss]['resolution']

    def get_heading(self):
        (x, y, z) = self.get_data()
        heading = math.atan2(y, x) + self.__declination

        if heading < 0:
            heading += 2 * math.pi
        elif heading > 2 * math.pi:
            heading -= 2 * math.pi

        return heading


if __name__ == "__main__":
    magnetometer = Magnetometer(declination=(12, 2.0))
    while True:
        heading = magnetometer.get_heading()
        degrees, minutes = convert_radians_to_degrees_minutes(heading)
        print('Heading: {:d}\N{DEGREE SIGN} {:d}\''.format(degrees, minutes))
        time.sleep(0.5)
