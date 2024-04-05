import sys

import numpy as np
import time
import control as ctrl

class PID:
    def __init__(self):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
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
        self.x_hat_prev = 0
        self.U_k = 0

    def set_state_space(self, A, B, C, D):
        self.A = A
        self.B = B
        self.C = C
        self.D = D

    def get_state_space(self):
        return self.A, self.B, self.C, self.D

    def Controller_rpm(self,M_rpm ,set_rpm,dt):
        B = 4
        error_rpm_dt = (set_rpm - M_rpm) * dt
        U = B * error_rpm_dt
        return U

    def Controller_I_SS(self, M_angle, rpm, set_angle, set_rpm, dt):
        # Implement controller using this function
        kp1 = 7.1167  #20.3709 # # 448.3557 #9.7897 # 0.0979
        kp2 = 0.5639 # 0.2981 #  # 0.7124##0.0512 # -0.0371

        # kp1 = 7.5969
        # kp2 = 0.0667
        # kp3 = -0.0001
        K = np.array([kp1, kp2])
        # Ke = kp3
        error_angle = float(dt * (set_angle-M_angle))
        error_rpm = float(dt * (set_rpm-rpm))


        x1_next = self.x1_prev + (error_angle)
        print("error",x1_next)
        x2_next = self.x2_prev + (error_rpm)
        x = np.array([[x1_next], [x2_next]])

        U = np.dot(K,x)
        self.x1_prev = x1_next
        self.x2_prev = x2_next
        self.U_k = U
        return U


    def Controller_SS_Vanilla(self, M_angle, rpm, set_angle, set_rpm, dt):
        # Implement controller using this function
        kp1 = 7.1167    #1.8213 #20.3709 # # 448.3557 #9.7897 # 0.0979
        kp2 = 0.5639#0.1256 # 0.2981 #  # 0.7124##0.0512 # -0.0371
        error_angle_dt = float((set_angle - M_angle) * dt)
        error_rpm_dt = float((set_rpm - rpm) * dt)

        K = np.array([kp1, kp2])
        x = np.array([[error_angle_dt], [error_rpm_dt]])

        U = K @ x
        self.U_k = U
        return U

    def Observer(self, M_angle, M_rpm, set_angle, set_rpm, dt):
        # Implement controller using this function
        kp1 = 3.2379  # 1.8213 #20.3709 # # 448.3557 #9.7897 # 0.0979
        kp2 = 0.1831  # 0.1256 # 0.2981 #  # 0.7124##0.0512 # -0.037
        K = np.array([kp1, kp2])

        Af = np.rot90(self.A, 3)
        C = np.array([1, 0])  # 1 / (2 * np.pi * 1 / 60)
        p1 = -41.2689 + 56.3055j
        p2 = -41.2689 - 56.3055j
        poles = [p1, p2]
        L = ctrl.acker(Af, [[1], [0]], poles)
        #Aco = np.array([[self.A-self.B*K, B*K], [zeros(size(A))A - L*C]])
        #Bco = [B * Nbar;
        #zeros(size(B))];
        #Cco = [C zeros(size(C))];
        #Dco = 0;

        x_hat = self.x_hat_prev + dt*(self.A @ self.x_hat_prev + self.B * self.U_k + L @ (M_angle - self.C * self.x_hat_prev))


        U = K @ x_hat
        self.x_hat_prev = x_hat
        return U

    def copy(self, pid):
        self.x1_prev = pid.x1_prev
        self.x2_prev = pid.x2_prev
        self.A = pid.A
        self.B = pid.B
        self.C = pid.C
        self.D = pid.D
        self.Kp = pid.Kp
        self.Ki = pid.Ki
        self.Kd = pid.Kd
        self.windup = pid.windup
        self.useWindup = pid.useWindup

