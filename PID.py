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
        A = np.array([[0, 1], [0, - 51.02173913]])
        B = np.array([[0], [1086.95652174]])
        C = np.array([0, 9.54929659])
        D = np.array([[0]])
        return

    def regulate(self,M_angle,rpm,angle):
        # Implement controller using this function
        A = np.array([[0, 1], [0, - 51.02173913]])
        B = np.array([[0], [1086.95652174]])
        C = np.array([0, 9.54929659])
        D = np.array([[0]])
        kp1 = 0.0979  #4.4836
        kp2 = -0.0371 #0.0290
        K = np.array([[kp1,kp2]])
        x = np.array([[M_angle],[rpm]])
        r = angle
        u = np.dot(-K,x)
        return u

    def copy(self, pid):
        self.kp = pid.kp
        self.ki = pid.ki
        self.kd = pid.kd
        self.windup = pid.windup
        self.useWindup = pid.useWindup
