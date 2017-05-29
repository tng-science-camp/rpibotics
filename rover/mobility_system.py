import numpy
import math
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
        self.motor_right = DCMotor(13, 26, 19, f=20)
        self.motor_left = DCMotor(12, 20, 16, f=20)
        self.encoder_right = OptocouplerEncoder(7, s=20)
        self.encoder_right.run()
        self.encoder_left = OptocouplerEncoder(8, s=20)
        self.encoder_left.run()
        self._stop = True

    def stop(self):
        self.motor_left.stop()
        self.motor_right.stop()

    def go_forward(self, rotations, duty_cycle=70, timeout=30, delta_t=0.1, P=0.1, I=0.0, D=0.0):
        start_time = time.time()
        u = numpy.matrix(((0),(0)))
        d_right = duty_cycle
        d_left = duty_cycle
        self._stop = False
        r_diff_prev = None
        self.encoder_right.reset()
        self.encoder_left.reset()
        self.motor_right.turn_clockwise(duty_cycle)
        self.motor_left.turn_clockwise(duty_cycle)
        while not self._stop and time.time() - start_time < timeout:
            print("Right  r = {:0.2f}, r_dot = {:0.2f}"
                  .format(self.encoder_right.get_rotations(),
                          self.encoder_right.get_rotation_rate()))
            print("Left   r = {:0.2f}, r_dot = {:0.2f}"
                  .format(self.encoder_left.get_rotations(),
                          self.encoder_left.get_rotation_rate()))
            #print("t = {:0.2f}".format(time.time()))
            if self.encoder_right.get_rotations() >= rotations or self.encoder_left.get_rotations() >= rotations:
                self._stop = True
            else:
                r_diff = numpy.matrix(((self.encoder_right.get_rotations() - self.encoder_left.get_rotations()),
                                       (self.encoder_right.get_rotation_rate() - self.encoder_left.get_rotation_rate())))
                if r_diff_prev is not None:
                    r_diff_dot = (r_diff - r_diff_prev) / delta_t
                    u = P * r_diff + D * r_diff_dot
                r_diff_prev = r_diff
            d_right = max(d_right - 50 * u[0, 0], 0)
            d_right = min(d_right, 100)
            d_left = max(d_left + 50 * u[0, 0], 0)
            d_left = min(d_left, 100)
            print("d_r = {:0.2f} d_l = {:0.2f}".format(d_right, d_left))
            self.motor_right.turn_clockwise(d_right)
            self.motor_left.turn_clockwise(d_left)
            time.sleep(delta_t)

        self.stop()
        print("Right  r = {:0.2f}, r_dot = {:0.2f}"
              .format(self.encoder_right.get_rotations(),
              self.encoder_right.get_rotation_rate()))
        print("Left   r = {:0.2f}, r_dot = {:0.2f}"
              .format(self.encoder_left.get_rotations(),
              self.encoder_left.get_rotation_rate()))
        print("Delta_t = {:0.2f}".format(time.time() - start_time))



