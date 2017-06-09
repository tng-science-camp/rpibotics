class PID(object):
    def __init__(self, kp: float, ki: float, kd: float):
        """

        :rtype: None
        """
        self._kp = kp
        self._ki = ki
        self._kd = kd

    def set_gains(self, kp: float, ki: float, kd: float):
        self._kp = kp
        self._ki = ki
        self._kd = kd

    def control_delta(self, e0, e1, e2, delta_t: float):
        delta_u = self._kp * (e2 - e1) + \
                  self._ki * delta_t * e2 + \
                  self._kd * (e2 - 2.0 * e1 + e0) / delta_t
        return delta_u
