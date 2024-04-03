import numpy as np


class PID_TF:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.error = 0
        self.lastError = 0
        self.IntegralError = 0
        self.DerivativeError = 0
        self.start_time = 0

    def compute(self, systemValue, dt):
        self.error = self.setpoint - systemValue
        self.IntegralError += self.error * dt
        #self.IntegralError = np.clip(self.IntegralError, a_min=-100, a_max=100)  # integral windup
        self.DerivativeError = (self.error - self.lastError) / dt
        print("error", self.error)
        print("IntegralError", self.IntegralError)
        output = (self.Kp * self.error) + (self.Ki * self.IntegralError) + (self.Kd * self.DerivativeError)

        self.lastError = self.error

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
