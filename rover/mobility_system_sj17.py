import logging
import numpy
import time
from typing import Tuple
from rover.dc_motor import DCMotor
from rover.optocoupler_encoder import OptocouplerEncoder
from rover.pid import PID
from rover.ir_obstacle_sensor import IRObstacleSensor

MOBILITY_SYSTEM_CONFIG = {
    'motor_left'           : {'gpio_pin_ena': 12,
                              'gpio_pin_in1': 20,
                              'gpio_pin_in2': 16,
                              'frequency'   : 100.0},
    'motor_right'          : {'gpio_pin_ena': 13,
                              'gpio_pin_in1': 26,
                              'gpio_pin_in2': 19,
                              'frequency'   : 100.0},
    'encoder_left'         : {'gpio_pin'  : 8,
                              'slit_count': 20},
    'encoder_right'        : {'gpio_pin'  : 7,
                              'slit_count': 20},
    'obstacle_sensor_left' : {'gpio_pin': 17},
    'obstacle_sensor_right': {'gpio_pin': 27},
    'pid'                  : {'kp': 300.0,
                              'ki': 200.0,
                              'kd': 0.0},
    'initial_duty_cycle'   : {'duty_cycle_left' : 70.0,
                              'duty_cycle_right': 70.0}
}


class MobilitySystem(object):
    def __init__(self, config=MOBILITY_SYSTEM_CONFIG):
        """

        :type config: dict
        """
        self.motor_left = DCMotor(
            gpio_pin_ena=config['motor_left']['gpio_pin_ena'],
            gpio_pin_in1=config['motor_left']['gpio_pin_in1'],
            gpio_pin_in2=config['motor_left']['gpio_pin_in2'],
            frequency=config['motor_left']['frequency'])
        self.motor_right = DCMotor(
            gpio_pin_ena=config['motor_right']['gpio_pin_ena'],
            gpio_pin_in1=config['motor_right']['gpio_pin_in1'],
            gpio_pin_in2=config['motor_right']['gpio_pin_in2'],
            frequency=config['motor_right']['frequency'])
        self.encoder_left = OptocouplerEncoder(
            gpio_pin=config['encoder_left']['gpio_pin'],
            slit_count=config['encoder_left']['slit_count'])
        self.encoder_right = OptocouplerEncoder(
            gpio_pin=config['encoder_right']['gpio_pin'],
            slit_count=config['encoder_right']['slit_count'])
        self.obstacle_sensor_left = IRObstacleSensor(
            gpio_pin=config['obstacle_sensor_left']['gpio_pin'])
        self.obstacle_sensor_right = IRObstacleSensor(
            gpio_pin=config['obstacle_sensor_right']['gpio_pin'])
        self._pid = PID(
            kp=config['pid']['kp'],
            ki=config['pid']['ki'],
            kd=config['pid']['kd'])
        self._initial_duty_cycle = \
            ((config['initial_duty_cycle']['duty_cycle_left'],),
             (config['initial_duty_cycle']['duty_cycle_right'],))
        self._wheel_circumference = numpy.pi * 0.065  # [meter]
        self._wheel_distance = numpy.pi * 0.125  # [meter]
        self._stop = True

    def set_initial_duty_cycle(
            self,
            duty_cycle: Tuple[Tuple[float], Tuple[float]]=((70.0,), (70.0,))):
        self._initial_duty_cycle = duty_cycle

    def stop(self):
        logging.info('Stopping.')
        self._stop = True
        self.motor_left.stop()
        self.motor_right.stop()

    def reset(self):
        logging.info('Resetting.')
        self.encoder_left.reset()
        self.encoder_right.reset()
        self.obstacle_sensor_left.clear_detect_callbacks()
        self.obstacle_sensor_right.clear_detect_callbacks()

    def enable_stop_when_front_is_blocked(self):
        logging.info('Enabling stop when front is blocked')
        self.obstacle_sensor_left.add_detect_callback(self.stop)
        self.obstacle_sensor_right.add_detect_callback(self.stop)

    def front_is_blocked(self):
        logging.debug('Checking if front is blocked.')
        front_is_blocked = self.obstacle_sensor_left.obstacle_is_in_front() or \
                           self.obstacle_sensor_right.obstacle_is_in_front()
        return front_is_blocked

    def go_forward(self,
                   target_distance: float = 0.3,
                   duty_cycle: Tuple[Tuple[float],
                                     Tuple[float]] = None,
                   timeout: float = 30,
                   delta_t: float = 0.01):
        logging.info('Driving forward.')
        return self.go_straight(target_distance=target_distance,
                                direction_is_forward=True,
                                duty_cycle=duty_cycle,
                                timeout=timeout,
                                delta_t=delta_t)

    def go_backward(self,
                    target_distance: float = 0.3,
                    duty_cycle: Tuple[Tuple[float], Tuple[float]] = None,
                    timeout: float = 5,
                    delta_t: float = 0.01):
        logging.info('Driving backward.')
        return self.go_straight(target_distance=target_distance,
                                direction_is_forward=False,
                                duty_cycle=duty_cycle,
                                timeout=timeout,
                                delta_t=delta_t)

    def turn_right(self,
                   target_angle: float = 90.0,
                   duty_cycle: Tuple[Tuple[float], Tuple[float]] = None,
                   timeout: float = 30,
                   delta_t: float = 0.001):
        logging.info('Turning right.')
        return self.turn(target_angle=target_angle,
                         direction_is_right=True,
                         duty_cycle=duty_cycle,
                         timeout=timeout,
                         delta_t=delta_t)

    def turn_left(self,
                  target_angle: float = 90.0,
                  duty_cycle: Tuple[Tuple[float], Tuple[float]] = None,
                  timeout: float = 30,
                  delta_t: float = 0.001):
        logging.info('Turning left.')
        return self.turn(target_angle=target_angle,
                         direction_is_right=False,
                         duty_cycle=duty_cycle,
                         timeout=timeout,
                         delta_t=delta_t)

    def go_straight(self,
                    target_distance: float = 0.3,
                    direction_is_forward: bool = True,
                    duty_cycle: Tuple[Tuple[float], Tuple[float]] = None,
                    timeout: float = 30,
                    delta_t: float = 0.01):
        start_time = time.time()

        self.reset()
        if direction_is_forward:
            self.enable_stop_when_front_is_blocked()

        target_rotation = numpy.ones((2, 1)) * \
                          target_distance / self._wheel_circumference
        logging.debug('Target Rotation = %s',
                      numpy.array2string(target_rotation).replace('\n', ''))

        if duty_cycle is not None:
            u2 = numpy.matrix(duty_cycle)
        else:
            u2 = numpy.matrix(self._initial_duty_cycle)
        e2 = numpy.matrix([[0.0], [0.0]])
        e1 = numpy.matrix([[0.0], [0.0]])

        rotation = numpy.matrix(
            [[self.encoder_left.get_rotations()],
             [self.encoder_right.get_rotations()]])

        self._stop = False
        while not self._stop and time.time() - start_time < timeout:
            logging.debug('Measured Rotation = %s',
                          numpy.array2string(rotation).replace('\n', ''))
            if numpy.all(numpy.greater_equal(rotation, target_rotation)):
                logging.debug('Target Rotation = %s has been reached.',
                              numpy.array2string(target_rotation).replace(
                                  '\n', ''))
                self.stop()
            else:
                e0 = e1
                e1 = e2
                e2 = numpy.matrix([[rotation[1, 0] - rotation[0, 0]],
                                   [rotation[0, 0] - rotation[1, 0]]]) / 2
                logging.debug('Rotation Error = %s',
                              numpy.array2string(e2).replace('\n', ''))
                u2 += self._pid.control_delta(e0, e1, e2, delta_t)
                u2[u2 > 100] = 100.0
                u2[u2 < 30] = 30.0
                logging.debug('Control Input = %s',
                              numpy.array2string(u2).replace('\n', ''))
                if direction_is_forward:
                    self.motor_left.turn_clockwise(u2[0, 0])
                    self.motor_right.turn_clockwise(u2[1, 0])
                else:
                    self.motor_left.turn_counter_clockwise(u2[0, 0])
                    self.motor_right.turn_counter_clockwise(u2[1, 0])
                time.sleep(delta_t)
                rotation = numpy.matrix(
                    [[self.encoder_left.get_rotations()],
                     [self.encoder_right.get_rotations()]])
        distance = rotation * self._wheel_circumference
        drive_time = time.time() - start_time
        return distance, drive_time

    def turn(self,
             target_angle: float = 90.0,
             direction_is_right: bool = True,
             duty_cycle: Tuple[Tuple[float],
                               Tuple[float]] = ((70.0), (70.0)),
             timeout: float = 30,
             delta_t: float = 0.001):
        start_time = time.time()

        target_rotation = numpy.ones((2, 1)) * \
                          self._wheel_distance / (360.0 / target_angle) / \
                          self._wheel_circumference
        logging.debug('Target Rotation = %s',
                      numpy.array2string(target_rotation).replace('\n', ''))

        self.reset()

        if duty_cycle is not None:
            u2 = numpy.matrix(duty_cycle)
        else:
            u2 = numpy.matrix(self._initial_duty_cycle)
        e2 = numpy.matrix([[0.0], [0.0]])
        e1 = numpy.matrix([[0.0], [0.0]])

        rotation = numpy.matrix(
            [[self.encoder_left.get_rotations()],
             [self.encoder_right.get_rotations()]])

        self._stop = False

        while not self._stop and time.time() - start_time < timeout:
            logging.debug('Measured Rotation = %s',
                          numpy.array2string(rotation).replace('\n', ''))
            if numpy.all(numpy.greater_equal(rotation, target_rotation)):
                logging.debug('Target Rotation = %s has been reached.',
                              numpy.array2string(target_rotation).replace(
                                  '\n', ''))
                self.stop()
            else:
                e0 = e1
                e1 = e2
                e2 = numpy.matrix([[rotation[1, 0] - rotation[0, 0]],
                                   [rotation[0, 0] - rotation[1, 0]]]) / 2
                logging.debug('Rotation Error = %s',
                              numpy.array2string(e2).replace('\n', ''))

                control_delta = self._pid.control_delta(e0, e1, e2, delta_t)
                u2 += control_delta
                u2[u2 > 100] = 100.0
                u2[u2 < 0] = 0.0
                logging.debug('Control Input = %s',
                              numpy.array2string(u2).replace('\n', ''))
                if direction_is_right:
                    self.motor_left.turn_clockwise(u2[0, 0])
                    self.motor_right.turn_counter_clockwise(u2[1, 0])
                else:
                    self.motor_left.turn_counter_clockwise(u2[0, 0])
                    self.motor_right.turn_clockwise(u2[1, 0])
                time.sleep(delta_t)
                rotation = numpy.matrix(
                    [[self.encoder_left.get_rotations()],
                     [self.encoder_right.get_rotations()]])

        distance = rotation * self._wheel_circumference
        drive_time = time.time() - start_time
        return distance, drive_time
