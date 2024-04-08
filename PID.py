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
        self.x_hat_prev = np.array([[0],[0]])
        self.U_k = 0
        self.x_hat = np.array([[0],[0]])
        self.x_dot = np.array([[0],[0]])

    def set_state_space(self, A, B, C, D):
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def get_state_space(self):
        return self.A, self.B, self.C, self.D

    def Controller_rpm(self, M_rpm, set_rpm):
        B = 4

        error_rpm = (set_rpm - M_rpm)
        U = B * error_rpm
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

    def Controller_SS_Vanilla(self, M_angle, rpm, set_angle, set_rpm, dt):
        # Implement controller using this function
        kp1 = 0.0129  # 6.1452#1.8213 #20.3709
        kp2 = 0.0004  # 0.3856#0.1256 # 0.2981
        error_angle = float(set_angle - M_angle)
        error_rpm = float(set_rpm - rpm)

        K = np.array([kp1, kp2])
        x = np.array([[error_angle], [error_rpm]])

        U = (K @ x)
        return U

    #
    def Observer(self, M_angle, M_rpm, set_angle, set_rpm, dt):
        # self.IntegralError = np.clip( self.IntegralError,a_min = -5,a_max = 5)

        # Af = np.rot90(self.A, 3)
        # p1 = -60 + 8.7111j
        # p2 = -60 - 8.7111j
        # poles = [p1, p2]
        # L = ctrl.acker(Af, [[8255.3], [0]], poles)
        # Acl = np.array([[0, 1], [-3675.9, -120.0]])  # place poles for the system
        l1 = 122
        l2 = 3000
        L = np.array([[l1], [l2]])  # Place observer feedback
        x = np.array([[self.x1_prev], [self.x2_prev]])

        # print("A",self.A)
        # x_hat = np.dot(self.A, x_hat_prev) - ((L @ self.C) * (x - x_hat_prev))
        C = np.array([1,0])
        x1_dot = self.A[0,1]* self.x_hat_prev[0,0] + self.B[0,0] * self.U_k + L[0] * (M_angle - self.x_hat_prev[0,0])
        x2_dot = self.A[1, 1]* self.x_hat_prev[1, 0] + self.B[1, 0] * self.U_k + L[1] * (M_angle - self.x_hat_prev[0,0])
        x1_dot = float(x1_dot)
        x2_dot = float(x2_dot)
        print("x1_dot",x1_dot)
        print("x2_dot", x2_dot)

        #x_dot = np.dot(self.A,self.x_hat_prev) + self.B[0,1] * self.U_k + L[0,1] * (M_angle - self.C*self.x_hat_prev)
        x_dot=np.array([[x1_dot],[x2_dot]])
        print("x_dot", x_dot)
        #print("x_dot",x_dot)
        #print("self.A ", self.A)
        #print("self.x_hat_prev", self.x_hat_prev)
        #print("self.A * self.x_hat_prev:", np.dot(self.A,self.x_hat_prev))
        x_hat = self.x_hat_prev + dt * x_dot
        print("x_hat", x_hat)
        #print("x_hat", x_hat)
        #print(" ")
        #Acl = self.A - L*C
        #x_delta = x - x_hat_prev
        #print("x_delta",x_delta)
        #e_x  = np.dot(Acl, x_delta)
        #print("e_x", e_x)
        # np.dot(self.A, (x - x_hat_prev)) - l1 * (M_angle - (self.C @ x_hat_prev)) # X_k = A(x-x_hat)-L(y-y_hat)
        # np.dot(self.A, (x - x_hat_prev)) - l2 * (M_rpm - (self.C @ x_hat_prev))
        self.x_hat_prev = x_hat

        kp1 = 0.09  # 0.1079
        kp2 = 0.007  # 0.0085
        kp3 = -0.045  # -0.8708
        K = np.array([kp1, kp2, kp3])

        self.x1_prev = (set_angle - M_angle)  # *dt
        self.x2_prev = (set_rpm - M_rpm)  # *dt
        self.IntegralError = + (self.x1_prev * dt)

        x = np.array([[float(x_hat[0])], [float(x_hat[1])], [self.IntegralError]])

        print("x_hat[0]", x_hat[0])
        print("x_hat[1]", x_hat[1])
        U = -K @ x
        U = float(U)
        self.U_k = U
        print("U",U)

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
