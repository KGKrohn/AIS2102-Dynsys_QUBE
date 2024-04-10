# ------------------------------------- AVAILABLE FUNCTIONS --------------------------------#
# qube.setRGB(r, g, b) - Sets the LED color of the QUBE. Color values range from [0, 999].
# qube.setMotorSpeed(speed) - Sets the motor speed. Speed ranges from [-999, 999].
# qube.setMotorVoltage(volts) - Applies the given voltage to the motor. Volts range from (-24, 24).
# qube.resetMotorEncoder() - Resets the motor encoder in the current position.
# qube.resetPendulumEncoder() - Resets the pendulum encoder in the current position.
import numpy as np

# qube.getMotorPosition() - Returns the cumulative angular positon of the motor.
# qube.getPendulumPosition() - Returns the cumulative angular position of the pendulum.
# qube.getMotorRPM() - Returns the newest rpm reading of the motor.
# qube.getMotorCurrent() - Returns the newest reading of the motor's current.
# ------------------------------------- AVAILABLE FUNCTIONS --------------------------------#

from QUBE import *
from logger import *
from com import *
from liveplot import *
from time import time
import threading
from PID_TF import PID_TF

# Replace with the Arduino port. Can be found in the Arduino IDE (Tools -> Port:)
port = "COM8"
baudrate = 115200
qube = QUBE(port, baudrate)

# Resets the encoders in their current position.
qube.resetMotorEncoder()
qube.resetPendulumEncoder()

# Enables logging - comment out to remove
enableLogging()

t_last = time()

m_target = 0
p_target = 0
pid = PID()

y = 0
x = 0
X = 0
i = 0
t_prev = 0
t_0 = time()

pid_tf_rmp = PID_TF(0.01, 0.1, 0.00001, 0)  # Normal PID
pid_tf_angle = PID_TF(0.02, 0.08, 0.003, 0)  # Normal PID


def control(data, lock):
    global m_target, p_target, pid, dt, y, x, i, x_prev, r, pid_tf_angle, t_prev

    while True:
        t_now = round(time() - t_0, 3)
        t_now_2 = time() - t_0
        dt = t_now_2 - t_prev

        # Updates the qube - Sends and receives data
        qube.update()
        # Gets the logdata and writes it to the log file
        m_target = 0
        logdata = qube.getLogData(m_target, p_target)
        save_data(logdata, t_now)
        with lock:
            doMTStuff(data)

        # pid.Observer(QUBE.getMotorAngle(qube), QUBE.getMotorRPM(qube), 60, 0, dt)
        if t_now >= 5 and not (t_now > 8):
            U = pid.Controller_rpm(QUBE.getMotorRPM(qube), 1500)
            # = pid.Observer(QUBE.getMotorAngle(qube), QUBE.getMotorRPM(qube), 60, 0, dt)
            # U = pid.Controller_rpm(QUBE.getMotorRPM(qube), 1500,dt)
            # pid_tf_angle.updateSetpoint(60)


        elif t_now >= 8:
            U = pid.Controller_rpm(QUBE.getMotorRPM(qube), 750)
            # = pid.Observer(QUBE.getMotorAngle(qube), QUBE.getMotorRPM(qube), 20, 0, dt)
            # pid_tf_angle.updateSetpoint(20)
            # U = pid.Controller_rpm(QUBE.getMotorRPM(qube), 750,dt)
        else:
            # pid_tf_angle.updateSetpoint(0)
            U = 0
        # U = pid_tf_angle.compute(QUBE.getMotorAngle(qube), dt)
        # U = pid_tf_rmp.compute(QUBE.getMotorRPM(qube), dt)

        # U = pid.Controller_rpm(QUBE.getMotorRPM(qube), 1000)
        # U = pid.Controller_SS_Vanilla(QUBE.getMotorAngle(qube), QUBE.getMotorRPM(qube), 60, 0, dt)

        # U = pid.Controller_SS_Vanilla(QUBE.getMotorAngle(qube), QUBE.getMotorRPM(qube), 60, 0, dt)
        #if t_now >= 12:
        #    break

        # print(U)
        qube.setMotorVoltage(U)
        t_prev = t_now_2


def doMTStuff(data):
    packet = data[7]
    pid.copy(packet.pid)
    if packet.resetEncoders:
        qube.resetMotorEncoder()
        qube.resetPendulumEncoder()
        packet.resetEncoders = False

    new_data = qube.getPlotData(m_target, p_target)
    for i, item in enumerate(new_data):
        data[i].append(item)


if __name__ == "__main__":
    _data = [[], [], [], [], [], [], [], Packet()]
    lock = threading.Lock()
    thread1 = threading.Thread(target=startPlot, args=(_data, lock))
    thread2 = threading.Thread(target=control, args=(_data, lock))
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
