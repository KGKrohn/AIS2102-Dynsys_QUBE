import sys

import numpy as np
import time
import control as ctrl


class PID:
    def __init__(self):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.A = np.array([[0, 1], [0, -6.6225]])
        self.B = np.array([[0], [7700.3]])
        self.C = np.array([[1, 0]])
        self.D = np.array([[0]])

        self.windup = 0
        self.lastIntegral = 0
        self.IntegralError = 0
        self.lastError = 0
        self.useWindup = False
        self.x1_prev = 0.1
        self.x2_prev = 0.1
        self.x3_prev = 0.1
        self.x1_hat_prev = 0.1
        self.x2_hat_prev = 0.1
        self.x_hat_prev = np.array([[0], [0]])
        self.U_k = 0
        self.x_hat = np.array([[0], [0]])
        self.x_dot = np.array([[0], [0]])
        self.x_rpm_prev = 0

    def set_state_space(self, A, B, C, D):
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def get_state_space(self):
        return self.A, self.B, self.C, self.D

    def Controller_rpm(self, M_rpm, set_rpm):
        K = 5 / 1150
        r = 0.0052
        U = r*set_rpm - K*(M_rpm-set_rpm)
        return U

    def Controller_I_SS(self, M_angle, rpm, set_angle, set_rpm, dt):
        kp1 = 0.09
        kp2 = 0.007
        kp3 = -0.9471
        K = np.array([kp1, kp2, kp3])

        error_angle = (set_angle - M_angle)
        error_rpm = (set_rpm - rpm)
        self.IntegralError = + (error_angle * dt)
        self.IntegralError = np.clip(self.IntegralError, a_min=-5, a_max=5)
        x = np.array([[error_angle], [error_rpm], [self.IntegralError]])

        U = K @ x
        return U

    def Controller_SS_Vanilla(self, M_angle, M_rpm, set_angle, set_rpm, dt):
        kp1 = 0.0129  
        kp2 = 0.0004
        error_angle = set_angle - M_angle
        error_rpm = set_rpm - M_rpm

        K = np.array([kp1, kp2])
        x = np.array([[error_angle], [error_rpm]])

        U = (K @ x)
        return U

    #
    def Observer(self, M_angle, M_rpm, set_angle, set_rpm, dt):
        l1 = 148
        l2 = 380.0
        a11 = 0
        a12 = 1
        a21 = 0
        a22 = -6.622
        b11 = 0
        b21 = 7700.7
        c11 = 1
        c12 = 0

        x1_predict = self.x1_hat_prev + dt * (a11 * self.x1_hat_prev + a12 * self.x2_hat_prev + b11 * self.U_k)
        x2_predict = self.x2_hat_prev + dt * (a21 * self.x1_hat_prev + a22 * self.x2_hat_prev + b21 * self.U_k)

        x1_hat = x1_predict - dt * l1 * (c11 * x1_predict - M_angle)
        x2_hat = x2_predict - dt * l2 * (c12 * x2_predict - M_rpm)
        self.x1_hat_prev = x1_hat
        self.x2_hat_prev = x2_hat

        kp1 = 0.0129
        kp2 = 0.0004
        r = set_angle * 0.0145

        K = np.array([kp1, kp2])
        x = np.array([[x1_hat], [x2_hat]])

        U = -K @ x + r
        U = float(U)
        self.U_k = U

        return U

    def copy(self, pid):
        self.A = pid.A
        self.B = pid.B
        self.C = pid.C
        self.D = pid.D
        self.Kp = pid.Kp
        self.Ki = pid.Ki
        self.Kd = pid.Kd
        self.windup = pid.windup
        self.useWindup = pid.useWindup
