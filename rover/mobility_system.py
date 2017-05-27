import numpy
import time
from rover.dc_motor import DCMotor
from rover.optocoupler_encoder import OptocouplerEncoder


class MobilitySystem(object):
    # There should only be one instance
    _instances = []

    # Initialise the object
    def __init__(self):
        if len(self._instances) > 1:
            print("ERROR: You can't have more than one mobility system.")
            exit(1)
        self._wheel_circumference = 0.021 # meters
        self._instances.append(self)
        self.motor_right = DCMotor(13, 26, 19, f=20000)
        self.motor_left = DCMotor(12, 20, 16, f=20000)
        self.encoder_right = OptocouplerEncoder(7, s=20)
        self.encoder_left = OptocouplerEncoder(8, s=20)
        self._stop = True
        self._delta_t = 0.1

    def go_forward(self, rotations, timeout=20):
        start_time = time.time()
        P = 1
        D = 0
        self._stop = False
        r_diff_prev = None
        self.encoder_right.reset()
        self.encoder_left.reset()
        self.motor_right.turn_counter_clockwise()
        self.motor_left.turn_counter_clockwise()
        while not self._stop and time.time() - start_time < timeout:
            if self.encoder_right.get_rotations() >= rotations or self.encoder_left.get_rotations() >= rotations:
                self._stop = True
            elif r_diff_prev is not None:
                r_diff = numpy.array([self.encoder_right.get_rotations() - self.encoder_left.get_rotations(),
                                      self.encoder_right.get_rotation_rate() - self.encoder_left.get_rotation_rate()])
                r_diff_dot = (r_diff - r_diff_prev) / self._delta_t
                u = P * r_diff + D * r_diff_dot
            r_diff_prev = r_diff
            time.sleep(self._delta_t)
        self.motor_left.stop()
        self.motor_right.stop()
        print("Right  r = %f, r_dot = %f",
              self.encoder_right.get_rotations(),
              self.encoder_right.get_rotation_rate())
        print("Left   r = %f, r_dot = %f",
              self.encoder_left.get_rotations(),
              self.encoder_left.get_rotation_rate())



