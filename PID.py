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
        self.x_rpm_prev =0

    def set_state_space(self, A, B, C, D):
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def get_state_space(self):
        return self.A, self.B, self.C, self.D

    def Controller_rpm(self, M_rpm, set_rpm, dt,):
        K = 5/1150
        error_rpm = ((set_rpm*2) - M_rpm)
        U = K * error_rpm
        return U


    def Controller_I_SS(self, M_angle, rpm, set_angle, set_rpm, dt):
        kp1 = 0.09  # 3.4773    # 6.1452#1.8213 #20.3709
        kp2 = 0.007  # 0.2916    # 0.3856#0.1256 # 0.2981
        kp3 = -0.9471  # -28.0603
        K = np.array([kp1, kp2, kp3])

        error_angle = (set_angle - M_angle)  # *dt
        error_rpm = (set_rpm - rpm)  # *dt
        self.IntegralError = + (error_angle * dt)
        x = np.array([[error_angle], [error_rpm], [self.IntegralError]])

        # x1_next = np.clip(x1_next,a_min = -5,a_max = 5)
        # x2_next = np.clip(x2_next, a_min=-5, a_max=5)

        U = K @ x
        return U

    def Controller_SS_Vanilla(self, M_angle, M_rpm, set_angle, set_rpm, dt):
        # Implement controller using this functionl
        kp1 = 0.0129  # 6.1452#1.8213 #20.3709
        kp2 = 0.0004  # 0.3856#0.1256 # 0.2981
        error_angle = float(set_angle - M_angle)
        error_rpm = float(set_rpm - M_rpm)

        K = np.array([kp1, kp2])
        x = np.array([[error_angle], [error_rpm]])

        U = (K @ x)
        return U

    #
    def Observer(self, M_angle, M_rpm, set_angle, set_rpm, dt):
        self.A = np.array([[0, 1], [0, -5.7471]])
        self.B = np.array([[0], [1189.7]])
        self.C = np.array([[1, 0]])
        self.D = np.array([[0]])
        print("B",self.B)
        # self.IntegralError = np.clip( self.IntegralError,a_min = -5,a_max = 5)
        l1 =  148
        l2 =  380.0
        a11 = 0
        a12 = 1
        a21 = 0
        a22 = -6.622#
        b11 = 0
        b21 = 7700.7#
        c11 = 1
        c12 = 0
        x = np.array([[self.x1_prev], [self.x2_prev]])

        # print("A",self.A)
        # x_hat = np.dot(self.A, x_hat_prev) - ((L @ self.C) * (x - x_hat_prev))
        # C = np.array([1,0])
        #x1_dot = self.A[0,1]* self.x_hat_prev[0,0] + self.B[0,0] * self.U_k + l1 * (M_angle - self.x_hat_prev[0,0])
        #x2_dot = self.A[1, 1]* self.x_hat_prev[1, 0] + self.B[1, 0] * self.U_k + l2 * (M_rpm - 0)
        x1_predict = self.x1_hat_prev + dt * (a11 * self.x1_hat_prev + a12 * self.x2_hat_prev + b11 * self.U_k)
        x2_predict = self.x2_hat_prev + dt * (a21 * self.x1_hat_prev + a22 * self.x2_hat_prev + b21 * self.U_k)
        #x1_dot = float(x1_dot)
        #x2_dot = float(x2_dot)
        print("x1_dot", x1_predict)
        print("x2_dot", x2_predict)

        # x_dot = np.dot(self.A,self.x_hat_prev) + self.B[0,1] * self.U_k + L[0,1] * (M_angle - self.C*self.x_hat_prev)
        # print("x_dot",x_dot)
        # print("self.A ", self.A)
        # print("self.x_hat_prev", self.x_hat_prev)
        # print("self.A * self.x_hat_prev:", np.dot(self.A,self.x_hat_prev))
        x1_hat = x1_predict - dt * l1 * (c11 * x1_predict - M_angle)
        x2_hat = x2_predict - dt * l2 * (c12 * x2_predict - M_rpm)
        print("x1_hat", x1_hat)
        print("x2_hat", x2_hat)
        self.x1_hat_prev = x1_hat
        self.x2_hat_prev = x2_hat
        # Implement controller using this function
        # Implement controller using this functionl
        kp1 = 0.0129
        kp2 = 0.0004
        r = set_angle * 0.0145

        K = np.array([kp1, kp2])
        x = np.array([[x1_hat], [x2_hat]])

        U = -K @ x+r
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
