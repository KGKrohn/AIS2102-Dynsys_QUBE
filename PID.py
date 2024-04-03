import sys

import numpy as np
import time


class PID:
    def __init__(self):
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.A = np.array([[0, 1], [0, - 51.02173913]])
        self.B = np.array([[0], [1086.95652174]])
        self.C = np.array([0, 9.54929659])
        self.D = np.array([[0]])
        self.x1_prev = 0.1
        self.x2_prev = 0.1
        self.windup = 0
        self.lastIntegral = 0
        self.lastError = 0
        self.useWindup = False

    def set_state_space(self, A, B, C, D):
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def get_state_space(self):
        return self.A, self.B, self.C, self.D

    def regulate(self, M_angle, rpm, set_angle, set_rpm, dt):
        # Implement controller using this function
        kp1 = 4.4836#448.3557 ##9.7897 # 0.0979
        kp2 = 0.0290#0.7124##0.0512 # 0.0371

        y1 = M_angle  # y1 = Cx + D| C = [1 0]
        y2 = rpm  # y2 = Cx+D | C = [0 1]
        r1 = set_angle
        r2 = set_rpm*30

        x1_next = self.x1_prev + (dt * (r1 - y1))
        x2_next = self.x2_prev + (dt * (r2 - y2))
        u1 = kp1 * x1_next  # u = -K*x
        u2 = kp2 * x2_next

        self.x1_prev = x1_next
        self.x2_prev = x2_next
        return u1, u2

    def copy(self, pid):
        self.x1_prev = pid.x1_prev
        self.x2_prev = pid.x2_prev
        self.A = pid.A
        self.B = pid.B
        self.C = pid.C
        self.D = pid.D
        self.kp = pid.kp
        self.ki = pid.ki
        self.kd = pid.kd
        self.windup = pid.windup
        self.useWindup = pid.useWindup

    def compute(self, systemValue):
        self.dt = time.time() - self.start_time
        self.error = self.setpoint - systemValue
        self.IntegralError += self.error * self.dt
        self.IntegralError = np.clip(self.IntegralError, a_min=-5, a_max=5)  # integral windup
        self.DerivativeError = (self.error - self.lastError) / self.dt

        output = (-self.Kp * self.error) + (-self.Ki * self.IntegralError) + (-self.Kd * self.DerivativeError)

        self.lastError = self.error
        self.start_time = time.time()

        return output

    def define_filtered_output(self, error, IntegralError, DerivativeError):
        output = (-self.Kp * error) + (-self.Ki * IntegralError) + (-self.Kd * DerivativeError)
        return output

    def updateSetpoint(self, newsetpoint):
        self.setpoint = newsetpoint

    def getSetpoint(self):
        return self.setpoint

    def resetErrors(self):
        self.lastError = 0
        self.IntegralError = 0
        self.DerivativeError = 0

    def getError(self):
        return self.error

    def getIntegralError(self):
        return self.IntegralError

    def getDerivativeError(self):
        return self.DerivativeError
