import logging
import numpy
import time
from rover.dc_motor import DCMotor
from rover.optocoupler_encoder import OptocouplerEncoder
from rover.pid import PID
from rover.ir_obstacle_sensor import IRObstacleSensor


class MobilitySystem(object):
    # There should only be one instance
    _instances = []

    # Initialise the object
    def __init__(self):
        if len(self._instances) > 1:
            print("ERROR: You can't have more than one mobility system.")
            exit(1)
        self._wheel_circumference = numpy.pi * 0.065  # [meter]
        self._wheel_distance = numpy.pi * 0.125  # [meter]
        self._instances.append(self)
        self.motor_left = DCMotor(12, 20, 16, f=100)
        self.motor_right = DCMotor(13, 26, 19, f=100)
        self.encoder_left = OptocouplerEncoder(8, s=20)
        self.encoder_right = OptocouplerEncoder(7, s=20)
        self.obstacle_sensor_left = IRObstacleSensor(17)
        self.obstacle_sensor_right = IRObstacleSensor(27)
        self._pid = PID(kp=300.0, ki=50.0, kd=20.0)
        self._stop = True

    def stop(self):
        logging.info('MobilitySystem stopping.')
        self._stop = True
        self.motor_left.stop()
        self.motor_right.stop()

    def reset(self):
        logging.info('MobilitySystem resetting.')
        self.encoder_left.reset()
        self.encoder_right.reset()
        self.obstacle_sensor_left.clear_detect_callbacks()
        self.obstacle_sensor_right.clear_detect_callbacks()

    def initialize(self):
        logging.info('MobilitySystem initializing.')
        self.stop()
        self.reset()
        self.obstacle_sensor_left.add_detect_callback(self.stop)
        self.obstacle_sensor_right.add_detect_callback(self.stop)

    def front_is_blocked(self):
        logging.debug('MobilitySystem front is blocked.')
        return self.obstacle_sensor_left.obstacle_is_in_front() or \
               self.obstacle_sensor_right.obstacle_is_in_front()

    def go_forward(self, target_distance: float = 0.3,
                   duty_cycle: float = 70.0,
                   timeout: float = 30, delta_t: float = 0.01):
        logging.info('MobilitySystem  moving forward.')
        return self.go_straight(target_distance=target_distance,
                                direction_is_forward=True,
                                duty_cycle=duty_cycle,
                                timeout=timeout,
                                delta_t=delta_t)

    def go_backward(self, target_distance: float = 0.3,
                    duty_cycle: float = 70.0,
                    timeout: float = 5, delta_t: float = 0.01):
        logging.info('MobilitySystem moving backward.')
        return self.go_straight(target_distance=target_distance,
                                direction_is_forward=False,
                                duty_cycle=duty_cycle,
                                timeout=timeout,
                                delta_t=delta_t)

    def turn_right(self, target_angle: float = 90.0, duty_cycle: float = 70.0,
                   timeout: float = 30, delta_t: float = 0.001):
        logging.info('MobilitySystem turning right.')
        return self.turn(target_angle=target_angle,
                         direction_is_right=True,
                         duty_cycle=duty_cycle,
                         timeout=timeout,
                         delta_t=delta_t)

    def turn_left(self, target_angle: float = 90.0, duty_cycle: float = 70.0,
                  timeout: float = 30, delta_t: float = 0.001):
        logging.info('MobilitySystem turning left.')
        return self.turn(target_angle=target_angle,
                         direction_is_right=False,
                         duty_cycle=duty_cycle,
                         timeout=timeout,
                         delta_t=delta_t)

    def go_straight(self,
                    target_distance: float = 0.3,
                    direction_is_forward: bool = True,
                    duty_cycle: float = 70.0,
                    timeout: float = 30, delta_t: float = 0.01):
        start_time = time.time()

        target_rotation = numpy.ones((2, 1)) * \
                          target_distance / self._wheel_circumference
        logging.debug('Target Rotation = %s',
                      numpy.array2string(target_rotation).replace('\n', ''))
        self.initialize()
        self._stop = False

        u2 = float(duty_cycle) * numpy.ones((2, 1))
        e2 = numpy.matrix([[0.0], [0.0]])
        e1 = numpy.matrix([[0.0], [0.0]])

        rotation = numpy.matrix(
            [[self.encoder_left.get_rotations()],
             [self.encoder_right.get_rotations()]])

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

    def turn(self, target_angle: float = 90.0, direction_is_right: bool = True,
             duty_cycle: float = 70.0,
             timeout: float = 30, delta_t: float = 0.001):
        start_time = time.time()

        target_rotation = numpy.ones((2, 1)) * \
                          self._wheel_distance / (360.0 / target_angle) / \
                          self._wheel_circumference
        logging.debug('Target Rotation = %s',
                      numpy.array2string(target_rotation).replace('\n', ''))

        self.initialize()
        self._stop = False

        u2 = float(duty_cycle) * numpy.ones((2, 1))
        e2 = numpy.matrix([[0.0], [0.0]])
        e1 = numpy.matrix([[0.0], [0.0]])

        rotation = numpy.matrix(
            [[self.encoder_left.get_rotations()],
             [self.encoder_right.get_rotations()]])

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
