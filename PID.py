import sys

import numpy as np


class PID:
    def __init__(self):
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.windup = 0
        self.lastIntegral = 0
        self.lastError = 0
        self.useWindup = False

    def state_space(self):
        Dm=1
        A = np.array(
            [[0, 1], [0, -(295147905179352825856 * Dm) / 1357680363825023 - 6490641434437103 / 1357680363825023]])
        return

    def regulate(self):
        # Implement controller using this function
        return 0

    def copy(self, pid):
        self.kp = pid.kp
        self.ki = pid.ki
        self.kd = pid.kd
        self.windup = pid.windup
        self.useWindup = pid.useWindup
