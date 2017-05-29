import numpy
import time
from rover.dc_motor import DCMotor
from rover.optocoupler_encoder import OptocouplerEncoder
from rover.pid import PID


class MobilitySystem(object):
    # There should only be one instance
    _instances = []

    # Initialise the object
    def __init__(self):
        if len(self._instances) > 1:
            print("ERROR: You can't have more than one mobility system.")
            exit(1)
        self._wheel_circumference = numpy.pi * 0.065  # [meter]
        self._wheel_distance = numpy.pi * 0.14        # [meter]
        self._instances.append(self)
        self.motor_left = DCMotor(12, 20, 16, f=20)
        self.motor_right = DCMotor(13, 26, 19, f=20)
        self.encoder_left = OptocouplerEncoder(8, s=20)
        self.encoder_right = OptocouplerEncoder(7, s=20)
        self._pid = PID(kp=300.0, ki=50.0, kd=20.0)
        self._stop = True

    def stop(self):
        self.motor_left.stop()
        self.motor_right.stop()

    def reset(self):
        self.encoder_left.reset()
        self.encoder_right.reset()

    def initialize(self):
        self.stop()
        self.reset()

    def go_forward(self, target_distance: float=0.3, duty_cycle: float=70.0,
                   timeout: float=30, delta_t: float=0.1):
        start_time = time.time()

        target_rotations = numpy.ones((2, 1)) * \
                           target_distance / self._wheel_circumference
        self.initialize()
        self._stop = False

        u2 = float(duty_cycle) * numpy.ones((2, 1))
        e2 = numpy.matrix([[0.0], [0.0]])
        e1 = numpy.matrix([[0.0], [0.0]])

        rotations = numpy.matrix(
            [[self.encoder_left.get_rotations()],
             [self.encoder_right.get_rotations()]])

        while not self._stop and time.time() - start_time < timeout:
            if numpy.all(numpy.greater_equal(rotations, target_rotations)):
                self._stop = True
            else:
                e0 = e1
                e1 = e2
                e2 = numpy.matrix([[rotations[1, 0] - rotations[0, 0]],
                                   [rotations[0, 0] - rotations[1, 0]]]) / 2
                u2 += self._pid.control_delta(e0, e1, e2, delta_t)
                u2[u2 > 100] = 100.0
                u2[u2 < 30] = 30.0
                self.motor_left.turn_clockwise(u2[0, 0])
                self.motor_right.turn_clockwise(u2[1, 0])
                time.sleep(delta_t)
                rotations = numpy.matrix(
                    [[self.encoder_left.get_rotations()],
                     [self.encoder_right.get_rotations()]])

        self.stop()
        distance = rotations * self._wheel_circumference
        drive_time = time.time() - start_time
        return distance, drive_time

    def turn_right(self, target_angle: float=90.0, duty_cycle: float=70.0,
                   timeout: float=30, delta_t: float=0.001):
        start_time = time.time()

        target_rotations = numpy.ones((2, 1)) * \
                           self._wheel_distance / (360.0 / target_angle) / \
                           self._wheel_circumference
        self.initialize()
        self._stop = False

        u2 = float(duty_cycle) * numpy.ones((2, 1))
        e2 = numpy.matrix([[0.0], [0.0]])
        e1 = numpy.matrix([[0.0], [0.0]])

        rotations = numpy.matrix(
            [[self.encoder_left.get_rotations()],
             [self.encoder_right.get_rotations()]])

        while not self._stop and time.time() - start_time < timeout:
            print("Rotations = {}".format(
                numpy.array2string(rotations).replace('\n', '')))
            if numpy.any(numpy.greater_equal(rotations, target_rotations)):
                self._stop = True
            else:
                e0 = e1
                e1 = e2
                e2 = numpy.matrix([[rotations[1, 0] - rotations[0, 0]],
                                   [rotations[0, 0] - rotations[1, 0]]]) / 2
                control_delta = self._pid.control_delta(e0, e1, e2, delta_t)
                u2 += control_delta
                u2[u2 > 100] = 100.0
                u2[u2 < 30] = 30.0
                print("Duty Cycle = {}".format(
                    numpy.array2string(u2).replace('\n', '')))
                self.motor_left.turn_clockwise(u2[0, 0])
                self.motor_right.turn_counter_clockwise(u2[1, 0])
                time.sleep(delta_t)
                rotations = numpy.matrix(
                    [[self.encoder_left.get_rotations()],
                     [self.encoder_right.get_rotations()]])

        self.stop()
        distance = rotations * self._wheel_circumference
        drive_time = time.time() - start_time
        return distance, drive_time